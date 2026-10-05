#!/usr/bin/env bash
#
# verify_sysusage.sh - build-matrix verification for the sysusage module
# (firmware/sysusage.{c,h}, see docs/plan-sysusage.md).
#
# Builds all four rows of the NCAP_SYSUSAGE x NCAP_FREERTOS matrix, each in its
# own scratch BUILD= dir - never firmware/build, so an ordinary `make -C
# firmware` run is not disturbed. Isolated dirs are mandatory here: rebuilding
# one tree with different -D flags back-to-back silently reuses stale objects
# (see scripts/run_5node.sh), which would make all four rows byte-identical.
#
# Checks per row:
#   - make exits 0 and an ELF exists
#   - data+bss stays under RAM_CEILING
#   - default row: zero sysusage_ symbols (this is the real "builds with or
#     without" proof, not just a missing source file)
#   - full-feature row: sysusage_publish and sysusage_task_start are present
#     (sysusage_task_start only there - with NCAP_FREERTOS=0 it is unreachable
#     and --gc-sections correctly drops it)
# Plus one guard row: NCAP_FREERTOS=1 + NCAP_VIDEO_UDP=2 must FAIL to build.
#
# Usage:
#   ./scripts/verify_sysusage.sh [--clean]
#
# --clean removes the scratch build dirs at the end. Non-destructive otherwise.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

CLEAN=0
for a in "$@"; do
    case "$a" in
        --clean) CLEAN=1 ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "ERROR: unknown flag '$a' (usage: $0 [--clean])" >&2; exit 2 ;;
    esac
done

TARGET=ncap_f4_w5500
JOBS="${JOBS:-8}"
PREFIX="${PREFIX:-arm-none-eabi-}"

# RAM budget. The F446 has 128 KB of SRAM and the linker script declares
# RAM LENGTH = 128K minus _Min_Heap_Size (0x200) + _Min_Stack_Size (0x400).
# Ceiling is 96 KiB of data+bss, which leaves exactly 32 KiB of physical SRAM
# headroom at 131072 - 98304. 32 KiB is the acceptance target (~30 KB) with
# 2 KiB of slack, and it is deliberately NOT the plan's "91 KB" number: that
# figure came from a pre-video baseline (data+bss 82296) that no longer exists.
# Today's default build sits at 63636, so the honest ceiling is measured from
# the physical part, not from a stale snapshot.
RAM_CEILING=98304

# Baseline of the default (both toggles 0) build, captured from this tree before
# sysusage existed: text 92700  data 388  bss 63248. The feature must be free
# when off, so the default row is compared against this. BASELINE_TOLERANCE can
# be raised if unrelated firmware work legitimately changes the image.
BASE_DATA=388
BASE_BSS=63248
BASELINE_TOLERANCE="${BASELINE_TOLERANCE:-0}"

# label|build dir|NCAP_SYSUSAGE|NCAP_FREERTOS
ROWS=(
    "default|build_su_s0f0|0|0"
    "sysusage only|build_su_s1f0|1|0"
    "freertos only|build_su_s0f1|0|1"
    "full feature|build_su_s1f1|1|1"
)

SUM_LABELS=()
SUM_RESULTS=()
SUM_DETAIL=()
FAILURES=0

record() { SUM_LABELS+=("$1"); SUM_RESULTS+=("$2"); SUM_DETAIL+=("$3"); }

# build_row <dir> <sysusage> <freertos> - make in a scratch dir, log to <dir>/make.log
# -k keeps going after a failed translation unit. Without it make aborts on
# whichever .c failed first and -j8 makes that a race, so a single run would
# surface one error at random and hide the rest; -k compiles every TU and the
# log then lists all of them.
build_row() {
    local dir="$1" su="$2" fr="$3"
    # Start from an empty dir. The Makefile's object rules depend only on the
    # source and on Makefile itself, so neither a changed -D value nor a changed
    # header (FreeRTOSConfig.h, any .h) triggers a recompile. Without this rm a
    # re-run silently verifies the PREVIOUS run's binary - which is how this
    # script first reported a stale size for a config change that had in fact
    # been applied. See the build-flag footgun in README.md.
    rm -rf "$ROOT/firmware/$dir"
    mkdir -p "$ROOT/firmware/$dir"
    make -k -C "$ROOT/firmware" -j"$JOBS" BUILD="$dir" \
        NCAP_SYSUSAGE="$su" NCAP_FREERTOS="$fr" >"$ROOT/firmware/$dir/make.log" 2>&1
}

size_of() {
    # arm-none-eabi-size prints a header line whose first field is the literal
    # word "text", then one data line. Anchor on the header and take the line
    # after it. Do NOT try to recognise the data line by its own shape: gcc
    # echoes source fragments as "  507 |   ok", whose first field is a bare
    # line number, so any /^<digits>/ match also catches compiler diagnostics.
    awk '$1 == "text" { getline; print $1, $2, $3 }' \
        "$ROOT/firmware/$1/make.log" | tail -n 1
}

nm_symbols() {
    "$PREFIX"nm "$ROOT/firmware/$1/$TARGET.elf" 2>/dev/null |
        grep -o 'sysusage_[A-Za-z0-9_]*' | sort -u || true
}

check_row() {
    set +e
    local label="$1" dir="$2" su="$3" fr="$4"
    local elf="$ROOT/firmware/$dir/$TARGET.elf"
    local problems="" notes=""

    echo ""
    echo "=== $label: NCAP_SYSUSAGE=$su NCAP_FREERTOS=$fr (BUILD=$dir) ==="

    if build_row "$dir" "$su" "$fr"; then
        echo "  build: ok"
    else
        local rc=$?
        echo "  build: FAILED (make exit $rc)"
        echo "  errors/warnings from firmware/$dir/make.log:"
        grep -E 'error:|warning:|No rule to make target|undefined reference|multiple definition|ld returned|overflowed by' \
            "$ROOT/firmware/$dir/make.log" | head -n 12 | cut -c1-200 | sed 's/^/    /' \
            || echo "    (none found)"
        record "$label" "FAIL" "make exit $rc"
        FAILURES=$((FAILURES + 1))
        set -e
        return
    fi

    if [ -f "$elf" ]; then
        echo "  elf:   $dir/$TARGET.elf"
    else
        echo "  elf:   MISSING $dir/$TARGET.elf"
        record "$label" "FAIL" "no ELF"
        FAILURES=$((FAILURES + 1))
        set -e
        return
    fi

    local sizetext ram
    sizetext="$(size_of "$dir")"
    read -r _text _data _bss <<<"$sizetext"
    # Belt and braces: never feed a non-numeric field into (( )).
    case "${_text:-x}${_data:-x}${_bss:-x}" in
        *[!0-9]* | "")
            echo "  size:  NO arm-none-eabi-size LINE IN LOG (got '$sizetext')"
            record "$label" "FAIL" "no size output"
            FAILURES=$((FAILURES + 1))
            set -e
            return
            ;;
    esac
    ram=$((_data + _bss))
    echo "  size:  text=$_text data=$_data bss=$_bss  (data+bss=$ram B)"

    local warns
    warns="$(grep -c 'warning:' "$ROOT/firmware/$dir/make.log" || true)"
    echo "  warn:  $warns warning(s)"
    if [ "$warns" != "0" ]; then
        problems="$problems $warns warning(s)"
        notes="$notes warnings"
    fi

    # RAM ceiling.
    if [ "$ram" -gt "$RAM_CEILING" ]; then
        echo "  ram:   FAIL data+bss=$ram > ceiling=$RAM_CEILING"
        problems="$problems RAM over ceiling"
        notes="$notes ram=$ram"
    else
        echo "  ram:   ok, $((131072 - ram)) B of 128 KB SRAM headroom"
        notes="$notes ram=$ram"
    fi

    # Off-switch: no sysusage_ symbol may survive in the default build.
    local syms
    syms="$(nm_symbols "$dir")"
    if [ "$su" = "0" ]; then
        if [ -z "$syms" ]; then
            echo "  syms:  ok, no sysusage_ symbol (feature fully compiled out)"
        else
            echo "  syms:  FAIL sysusage symbols present with NCAP_SYSUSAGE=0:"
            echo "$syms" | sed 's/^/          /'
            problems="$problems sysusage symbols in off build"
            notes="$notes stray symbols"
        fi
    else
        # sysusage_publish is called from main.c's timer_callback, so it must
        # survive --gc-sections in every NCAP_SYSUSAGE=1 row.
        # sysusage_task_start is only called from main.c under
        # #if NCAP_FREERTOS == 1, so with NCAP_FREERTOS=0 it is correctly
        # dropped by --gc-sections; demanding it there would be a false failure.
        local want_list="sysusage_publish"
        [ "$fr" = "1" ] && want_list="$want_list sysusage_task_start"
        local missing=""
        for want in $want_list; do
            if ! printf '%s\n' "$syms" | grep -qx "$want"; then
                missing="$missing $want"
            fi
        done
        if [ -z "$missing" ]; then
            echo "  syms:  ok, required sysusage_ symbols present:$want_list"
            echo "        all sysusage_ symbols:"
            printf '%s\n' "$syms" | sed 's/^/          /'
            notes="$notes $(printf '%s\n' "$syms" | wc -l | tr -d ' ') sysusage syms"
        else
            echo "  syms:  FAIL missing:$missing"
            problems="$problems missing$missing"
            notes="$notes missing$missing"
        fi
    fi

    # Default row must be unchanged in size.
    if [ "$su" = "0" ] && [ "$fr" = "0" ]; then
        if [ "$_data" = "$BASE_DATA" ] &&
           [ $(( (_bss - BASE_BSS) < 0 ? -(_bss - BASE_BSS) : (_bss - BASE_BSS) )) -le "$BASELINE_TOLERANCE" ]; then
            echo "  base:  ok, identical to documented baseline (data=$BASE_DATA bss=$BASE_BSS)"
        else
            echo "  base:  FAIL baseline data=$BASE_DATA bss=$BASE_BSS, got data=$_data bss=$_bss"
            problems="$problems baseline drift"
            notes="$notes baseline drift"
        fi
    fi

    if [ -z "$problems" ]; then
        record "$label" "PASS" "$notes"
    else
        record "$label" "FAIL" "$notes -$problems"
        FAILURES=$((FAILURES + 1))
    fi
    set -e
}

echo "=== sysusage build-matrix verification ==="
echo "firmware: $ROOT/firmware"
echo "toolchain: $("${PREFIX}gcc" --version 2>/dev/null | head -n 1)"
echo "RAM ceiling: $RAM_CEILING B of data+bss (128 KB SRAM)"

# Preflight, informational only. The matrix can legitimately fail because the
# module or the build toggles are not landed yet; say so up front rather than
# letting every row fail with the same opaque compile error.
PRE=""
[ -f "$ROOT/firmware/sysusage.c" ] || PRE="$PRE sysusage.c"
[ -f "$ROOT/firmware/sysusage.h" ] || PRE="$PRE sysusage.h"
grep -q 'NCAP_SYSUSAGE' "$ROOT/firmware/Makefile" || PRE="$PRE makefile-NCAP_SYSUSAGE"
grep -q 'NCAP_FREERTOS' "$ROOT/firmware/Makefile" || PRE="$PRE makefile-NCAP_FREERTOS"
if [ -n "$PRE" ]; then
    echo "NOTE: not landed yet -$PRE"
    echo "      rows that need them are expected to fail below."
fi

for r in "${ROWS[@]}"; do
    IFS='|' read -r label dir su fr <<<"$r"
    check_row "$label" "$dir" "$su" "$fr"
done

# .bss delta, full feature vs default.
DEF_BSS=""
FULL_BSS=""
for r in "${ROWS[@]}"; do
    IFS='|' read -r label dir su fr <<<"$r"
    read -r _t _d b <<<"$(size_of "$dir")"
    if [ "$su" = "0" ] && [ "$fr" = "0" ]; then DEF_BSS="${b:-}"; fi
    if [ "$su" = "1" ] && [ "$fr" = "1" ]; then FULL_BSS="${b:-}"; fi
done
echo ""
if [ -n "$DEF_BSS" ] && [ -n "$FULL_BSS" ]; then
    DELTA=$((FULL_BSS - DEF_BSS))
    echo "=== .bss delta, full feature vs default: ${FULL_BSS} - ${DEF_BSS} = ${DELTA} B ==="
    if [ "$DELTA" -ge 8192 ]; then
        echo "FAIL: +${DELTA} B of .bss, budget is < 8192 B"
        record "bss delta" "FAIL" "delta=${DELTA}B >= 8192B budget"
        FAILURES=$((FAILURES + 1))
    else
        echo "ok: under the 8192 B budget"
        record "bss delta" "PASS" "delta=${DELTA}B"
    fi
else
    record "bss delta" "SKIP" "row did not build"
    echo "=== .bss delta: skipped (a row did not build) ==="
fi

# Mutual exclusion: NCAP_FREERTOS=1 with NCAP_VIDEO_UDP=2 must not build.
# FreeRTOS owns the loop, so the cooperative spin_some()/video_udp_step()
# multiplexing that mode 2 depends on cannot survive; the Makefile is expected
# to reject the combination with $(error).
#
# A non-zero make exit alone is NOT sufficient evidence: any unrelated compile
# error (e.g. sysusage.h not landed yet) would also fail the build and would
# turn this into a false PASS. $(error) emits a makefile-level diagnostic
# beginning "Makefile:<line>:", so require that specifically.
echo ""
echo "=== guard: NCAP_FREERTOS=1 NCAP_VIDEO_UDP=2 must fail to build ==="
rm -rf "$ROOT/firmware/build_su_guard"
mkdir -p "$ROOT/firmware/build_su_guard"
set +e
make -C "$ROOT/firmware" -j"$JOBS" BUILD=build_su_guard \
    NCAP_SYSUSAGE=1 NCAP_FREERTOS=1 NCAP_VIDEO_UDP=2 \
    >"$ROOT/firmware/build_su_guard/make.log" 2>&1
GUARD_RC=$?
set -e
GUARD_LOG="$ROOT/firmware/build_su_guard/make.log"
if [ "$GUARD_RC" -eq 0 ]; then
    echo "  FAIL: the combination built successfully; expected a \$(error) in firmware/Makefile"
    record "guard FRT+xUDP" "FAIL" "combination was accepted (make exit 0)"
    FAILURES=$((FAILURES + 1))
elif grep -qE '^Makefile:[0-9]+:' "$GUARD_LOG"; then
    echo "  ok, rejected at makefile level (make exit $GUARD_RC):"
    grep -E '^Makefile:[0-9]+:' "$GUARD_LOG" | head -n 5 | cut -c1-200 | sed 's/^/    /'
    record "guard FRT+xUDP" "PASS" "rejected by \$(error), exit $GUARD_RC"
else
    echo "  FAIL: build failed (exit $GUARD_RC) but NOT from a Makefile-level \$(error),"
    echo "        so the mutual exclusion is not actually enforced yet:"
    grep -nE 'error|Error' "$GUARD_LOG" | head -n 5 | cut -c1-200 | sed 's/^/    /' || true
    record "guard FRT+xUDP" "FAIL" "failed for the wrong reason (no \$(error))"
    FAILURES=$((FAILURES + 1))
fi

# DVP fit: micro-ROS + QQVGA + full sysusage must build. This is the combination
# the RAM budget exists to protect - DVP_FRAMES=1 is one 38400 B frame on top of
# the ~64 KB micro-ROS baseline, and the full feature adds 6420 B on top of
# that, landing at ~108 KB of 128 KB. A second full frame (DVP_FRAMES=2, 76800 B
# of ring) genuinely overflows and must be rejected by the Makefile guard, not
# by ld. Both directions are checked: the fitting row must link, the
# overflowing row must fail with a Makefile-level $(error).
echo ""
echo "=== dvp: micro-ROS + QQVGA + full sysusage must fit; FRAMES=2 must be rejected ==="
rm -rf "$ROOT/firmware/build_su_dvp1" "$ROOT/firmware/build_su_dvp2"
mkdir -p "$ROOT/firmware/build_su_dvp1" "$ROOT/firmware/build_su_dvp2"
set +e
make -C "$ROOT/firmware" -j"$JOBS" BUILD=build_su_dvp1 \
    NCAP_DVP=1 VIDEO_WIDTH=160 VIDEO_HEIGHT=120 DVP_FRAMES=1 \
    NCAP_SYSUSAGE=1 NCAP_FREERTOS=1 \
    >"$ROOT/firmware/build_su_dvp1/make.log" 2>&1
DVP1_RC=$?
make -C "$ROOT/firmware" -j"$JOBS" BUILD=build_su_dvp2 \
    NCAP_DVP=1 VIDEO_WIDTH=160 VIDEO_HEIGHT=120 DVP_FRAMES=2 \
    >"$ROOT/firmware/build_su_dvp2/make.log" 2>&1
DVP2_RC=$?
set -e
if [ "$DVP1_RC" -eq 0 ]; then
    DVP1_RAM=$(arm-none-eabi-size "$ROOT/firmware/build_su_dvp1/$TARGET.elf" 2>/dev/null | awk 'NR==2{print $2+$3}')
    echo "  ok, micro-ROS + QQVGA FRAMES=1 + full sysusage links (ram=${DVP1_RAM:-?} B)"
    record "dvp FRAMES=1" "PASS" "links, ram=${DVP1_RAM:-?}B"
else
    echo "  FAIL: the fitting DVP row did not build (exit $DVP1_RC):"
    grep -nE 'error|Error' "$ROOT/firmware/build_su_dvp1/make.log" | head -n 5 | cut -c1-200 | sed 's/^/    /' || true
    record "dvp FRAMES=1" "FAIL" "make exit $DVP1_RC"
    FAILURES=$((FAILURES + 1))
fi
if [ "$DVP2_RC" -eq 0 ]; then
    echo "  FAIL: DVP_FRAMES=2 built successfully; expected a \$(error) in firmware/Makefile"
    record "dvp FRAMES=2" "FAIL" "combination was accepted (make exit 0)"
    FAILURES=$((FAILURES + 1))
elif grep -qE '^Makefile:[0-9]+:' "$ROOT/firmware/build_su_dvp2/make.log"; then
    echo "  ok, DVP_FRAMES=2 rejected at makefile level (make exit $DVP2_RC)"
    record "dvp FRAMES=2" "PASS" "rejected by \$(error), exit $DVP2_RC"
else
    echo "  FAIL: FRAMES=2 failed (exit $DVP2_RC) but NOT from a Makefile-level \$(error):"
    grep -nE 'error|Error' "$ROOT/firmware/build_su_dvp2/make.log" | head -n 5 | cut -c1-200 | sed 's/^/    /' || true
    record "dvp FRAMES=2" "FAIL" "failed for the wrong reason (no \$(error))"
    FAILURES=$((FAILURES + 1))
fi

echo ""
echo "=== summary ==="
printf '%-16s %-6s %s\n' "ROW" "RESULT" "DETAIL"
for i in "${!SUM_LABELS[@]}"; do
    printf '%-16s %-6s %s\n' "${SUM_LABELS[$i]}" "${SUM_RESULTS[$i]}" "${SUM_DETAIL[$i]}"
done
echo ""
if [ "$FAILURES" -eq 0 ]; then
    echo "PASS: all rows green"
else
    echo "FAIL: $FAILURES check(s) failed"
fi

if [ "$CLEAN" = 1 ]; then
    echo ""
    echo "=== removing scratch build dirs ==="
    for d in build_su_s0f0 build_su_s1f0 build_su_s0f1 build_su_s1f1 build_su_guard build_su_dvp1 build_su_dvp2; do
        rm -rf "$ROOT/firmware/$d"
        echo "  rm firmware/$d"
    done
fi

exit "$([ "$FAILURES" -eq 0 ] && echo 0 || echo 1)"