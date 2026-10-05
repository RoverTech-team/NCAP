# Plan — STM32F4 system-usage reporting over micro XRCE-DDS, as a FreeRTOS task

## Goal

Publish live STM32F4 system-usage telemetry to ROS 2 over micro XRCE-DDS (the
existing W5500 custom UDP transport), collected by a dedicated FreeRTOS task,
implemented in **dedicated files** so the feature is:

- **modular** — one self-contained module (`firmware/sysusage.{c,h}`) that owns
  its publisher, its message buffers and its task;
- **optional at build time** — the module compiles to nothing when
  `NCAP_SYSUSAGE=0` (the default), and builds with *and without* FreeRTOS.

## Verified baseline (do not re-litigate)

- Toolchain: `arm-none-eabi-gcc` 15.2.1 present. Baseline builds clean:
  `text 31912  data 120  bss 82176` in ~1s with `make -C firmware -j8`.
- Single build file: `firmware/Makefile` (138 lines). No CMake, no CubeIDE
  project. All conditionality is currently `?=` defaults plus `-D` flags —
  **keep that style**, do not introduce `ifeq`.
- RAM: F446 has 128 KB. On the *current* working tree the default build is
  `data 388 bss 63248`, so `data+bss` = 63,636 B and **~67 KB headroom**.
  (The `bss 82176` figure below predates `ov7670_dvp.c` and `MX_SPI2_Init()`; the
  real numbers are in "Outcome" at the end.)
- `firmware/libmicroros.a` is a **prebuilt** archive (built out-of-tree for
  `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=soft`). **A new custom `.msg`
  cannot be added without rebuilding that 34.7 MB archive**, so the module must
  use a stock type already linked in.
- `diagnostic_msgs/msg/DiagnosticArray` type support **is** present in
  `libmicroros.a` (verified via `nm`). It is the right choice: ROS 2's native
  representation for arbitrary key/value health data, so no IDL rebuild.
- FreeRTOS V10.3.1 is **vendored and configured but completely unbuilt**: 34
  tracked files under `firmware/Middlewares/Third_Party/FreeRTOS/Source/`, none
  in `SRC`, no include path, `configENABLE_FPU 0` (contradicts the Makefile's
  `-mfpu=fpv4-sp-d16`), `configTOTAL_HEAP_SIZE 15360`,
  `configTICK_RATE_HZ 1000`, `configMAX_PRIORITIES 56`.
  `Core/Src/freertos.c` (830 lines, CMSIS-RTOS-v2, 5 robot-arm tasks) is
  CubeMX legacy **unrelated to NCAP** and is deliberately *not* built. Do not
  build it, do not delete it.
- `Core/Src/stm32f4xx_it.c:251` has `SysTick_Handler` calling only
  `HAL_IncTick()`. **`SVC_Handler` and `PendSV_Handler` do not exist there at
  all** (weak `Default_Handler` in `startup_stm32f446xx.s:270,276`). FreeRTOS
  will fault-loop on start without them.
- `Core/Src/main.c` is a bare-metal super-loop with **zero** task creation and no
  `vTaskStartScheduler()`. All failure paths are `while (1) { }` parks.
- **Single XRCE session**: socket 0 / one client. Two tasks must never call
  `rclc_*` concurrently — see the threading design below.

## Architecture decision: one executor, one publisher, task does sampling only

Two `rclc_executor`s spinning the same XRCE client would interleave XRCE
datagrams on one UDP session and corrupt it. So:

- The **existing** `rclc_executor_t` in `main.c` stays the only rclc caller.
  Under `NCAP_FREERTOS=1` it is spun by a dedicated `rcl` task — see the
  correction in "Outcome" below, because handing the CPU to the scheduler
  without doing this publishes nothing at all.
- The **sysusage FreeRTOS task never touches rclc**. It samples RTOS/kernel
  state into a plain-POD snapshot struct and publishes nothing.
- The existing 1 Hz `timer_callback()` in `main.c` calls
  `sysusage_publish()` on the executor's thread, which copies the latest
  snapshot into a preallocated `DiagnosticArray` and publishes it.

This is safe in both personalities: with `NCAP_FREERTOS=0` there is no task and
`sysusage_publish()` samples inline from `HAL_GetTick()`.

## Build toggles (three, orthogonal, all default-off)

| Macro | Default | Effect |
|---|---|---|
| `NCAP_SYSUSAGE` | `0` | Compile the sysusage module at all. `0` ⇒ not in `SRC`, no symbols, no RAM. |
| `NCAP_FREERTOS` | `0` | Build the FreeRTOS kernel + scheduler. Prerequisite for `NCAP_SYSUSAGE=1`'s task path. |
| `SYSUSAGE_PERIOD_MS` | `1000` | Publish period override. |

Matrix that must build (Agent D verifies):

```
NCAP_SYSUSAGE=0 NCAP_FREERTOS=0   (default: byte-identical to today)
NCAP_SYSUSAGE=1 NCAP_FREERTOS=0   (telemetry, inline sampling, no scheduler)
NCAP_SYSUSAGE=0 NCAP_FREERTOS=1   (scheduler, no telemetry)
NCAP_SYSUSAGE=1 NCAP_FREERTOS=1   (full feature: task + telemetry)
```
`sysusage.c` must compile **silently with zero warnings** in every one of these.

## RAM budget (Agent A owns this, do not exceed)

Target: **< 8 KB of additional `.bss`** in the full-feature build, keeping
`data+bss` under ~91 KB of 128 KB.

- Publish from **preallocated static buffers only**. `rosidl_runtime_c__String`
  `buffer` fields must be `static char[]` arrays, and each `*__Sequence.data`
  must point at a `static` array with `capacity` set to its true length. **Zero
  `malloc`, zero `strdup`, zero `snprintf` into heap.** `libmicroros.a` will
  call `rosidl_string_fini`/`fini` and free nothing — never hand it a heap
  pointer.
- Sequence sizes: 2 `DiagnosticStatus` (one `"system"`, one `"tasks"`),
  ≤ 24 `KeyValue` per status.
- Do **not** enable `sensor_msgs` image or anything video-adjacent here.
- `uxTaskGetSystemState` needs a `static TaskStatus_t[]`; size it 12 and clamp
  with `uxTaskGetNumberOfTasks()`.

## Threading / measurement design

CPU load: `configUSE_IDLE_HOOK 1` + a `volatile` idle-tick counter.
`idle_pct = 100 * idle_delta / tick_delta`, `load_pct = 100 - idle_pct`.
Snapshot is double-buffered; the handoff is a short
`taskENTER_CRITICAL()`/`taskEXIT_CRITICAL()` around a pointer/index flip only —
never around rclc, never around SPI/W5500.

Task-to-snapshot handoff must be a plain struct copy guarded by a critical
section, not a FreeRTOS queue, to keep RAM flat. Task is **statically
allocated** (`xTaskCreateStatic`) so it does not consume `configTOTAL_HEAP_SIZE`
— the heap should be *shrunk*, not grown, because the F446 has no RAM to spare
and micro-ROS already uses a separate static pool.

## File ownership (STRICT — this is how 4 parallel agents avoid conflicts)

| Agent | Owns — may edit | Must NOT touch |
|---|---|---|
| **A** | `firmware/sysusage.h` (new), `firmware/sysusage.c` (new) | anything else |
| **B** | `firmware/Makefile`, `firmware/Core/Inc/FreeRTOSConfig.h` | anything else |
| **C** | `firmware/Core/Src/stm32f4xx_it.c`, `firmware/Core/Src/main.c` | anything else |
| **D** | `README.md`, `scripts/verify_sysusage.sh` (new) | firmware sources |

`firmware/w5500_transport.c` was unowned by this table and turned out to be
load-bearing; the reconciler added the `ncap_cpu_relax()` hook to it. See
"Outcome: what the plan got wrong" at the end.

## API contract — `firmware/sysusage.h` (frozen; Agents B/C/D code against this verbatim)

```c
#ifndef NCAP_SYSUSAGE_H
#define NCAP_SYSUSAGE_H

#include <stdbool.h>
#include <stdint.h>

#ifndef NCAP_SYSUSAGE
#define NCAP_SYSUSAGE 0
#endif

#if NCAP_SYSUSAGE == 1

#include <rcl/allocator.h>
#include <rcl/publisher.h>
#include <rclc/node.h>
#include <rclc/support.h>
#include <diagnostic_msgs/msg/diagnostic_array.h>

#ifndef SYSUSAGE_PERIOD_MS
#define SYSUSAGE_PERIOD_MS 1000u
#endif

/* Create the /diagnostics publisher. Call once, after rclc_support_init and
   rclc_node_init_default, before rclc_executor_init is fine either way.
   topic_name NULL => "diagnostics". Returns false on failure; caller should
   keep running heartbeat-only rather than park. */
bool sysusage_publisher_init(rclc_support_t *support,
                             rcl_node_t *node,
                             rcl_allocator_t *allocator,
                             const char *topic_name);

/* Facts main.c knows and sysusage.c cannot discover. Pass NULL to clear. */
typedef struct {
    uint32_t uptime_ms;      /* HAL_GetTick() snapshot at publish time */
    uint32_t pool_size;      /* static RMW pool bytes (main.c POOL_SIZE) */
    uint32_t pool_used;      /* main.c pool_used */
    uint32_t video_fps_x100; /* achieved emulated video fps x100, 0 = n/a */
} sysusage_extra_t;
void sysusage_set_extra(const sysusage_extra_t *extra);

/* Start the sampler task. Must be called BEFORE vTaskStartScheduler().
   Returns true if a task is running. Returns false (not an error) when
   NCAP_FREERTOS == 0: sysusage_publish() then samples inline instead. */
bool sysusage_task_start(void);

/* Fill + publish one DiagnosticArray. Call ONLY from the rclc executor's
   thread (the existing timer callback). Never from a task. */
rcl_ret_t sysusage_publish(void);

/* Last published message, for tests/debug. NULL before first publish. */
const diagnostic_msgs__msg__DiagnosticArray *sysusage_last(void);

#endif /* NCAP_SYSUSAGE == 1 */
#endif /* NCAP_SYSUSAGE_H */
```

## main.c integration points (Agent C — implement exactly this shape)

1. `#include "sysusage.h"` and the explicit
   `rosidl_typesupport_microxrcedds_c__get_message_type_support_handle__diagnostic_msgs__msg__DiagnosticArray()`
   forward declaration, matching how `sensor_msgs/msg/Imu` is already declared
   at `main.c:30-33`.
2. After `rclc_node_init_default` (`main.c:294`) and before executor spin:
   `#if NCAP_SYSUSAGE == 1` → `sysusage_publisher_init(&support, &node,
   &allocator, NULL)`; on failure `printf("F4: sysusage publisher unavailable
   \r\n")` and **continue** (never park).
3. In `timer_callback` (`main.c:148`), after the IMU publish at `main.c:169`,
   inside `#if NCAP_SYSUSAGE == 1`: populate a `sysusage_extra_t` from
   `pool_used`/`POOL_SIZE`/`HAL_GetTick()` and call `sysusage_set_extra()` then
   `sysusage_publish()`. Ignore its return value except for a first-failure
   message, so a telemetry hiccup never stalls the heartbeat.
4. Immediately before the final `for (;;)` spin loop (`main.c:363`/`370`):
   `#if NCAP_FREERTOS == 1` → `sysusage_task_start()` if `NCAP_SYSUSAGE == 1`,
   then `vTaskStartScheduler()`. **FreeRTOS must own the loop** — call it there
   and let it never return; do not keep a bare-metal spin as a fallback inside
   the same `#if`, because `main.c`'s existing spin loop must remain the path
   for `NCAP_FREERTOS=0` (this also preserves the existing
   `rclc_executor_spin_some` + `video_udp_step()` cooperative mode untouched).

   Consequence Agent C must resolve: with `NCAP_FREERTOS=1`, `video_udp_step()`
   cooperation is lost unless it becomes a task. Keep it simple and in scope:
   **`NCAP_FREERTOS=1` and `NCAP_VIDEO_UDP=2` are mutually exclusive**; have
   Agent B emit `#error` if both are set, so the limitation is enforced by the
   compiler instead of silently misbehaving.
5. `stm32f4xx_it.c` — inside the existing `/* USER CODE BEGIN NCAP */` block,
   `#if NCAP_FREERTOS == 1`:
   ```c
   void SVC_Handler(void)    { vPortSVCHandler(); }
   void PendSV_Handler(void) { xPortPendSVHandler(); }
   ```
   and change `SysTick_Handler` to call **both** `xPortSysTickHandler()` and
   `HAL_IncTick()` — `w5500_transport.c:180-197` and `video_udp.c` both use
   `HAL_GetTick()` deadlines, so HAL's tick must keep advancing.
   Needs `#include "FreeRTOS.h"` + `#include "task.h"` inside that guard.

## FreeRTOSConfig.h changes (Agent B)

- `configENABLE_FPU 0` → `1`. The Makefile passes `-mfpu=fpv4-sp-d16` and
  `libmicroros.a` was built for it; leaving this 0 makes the CM4F port skip FPU
  context save while the compiler still emits FP instructions.
- `configUSE_IDLE_HOOK 0` → `1` (CPU load measurement).
- `configTOTAL_HEAP_SIZE 15360` → `4096`. Sysusage uses a *static* task; 15 KB of
  BSS for a heap nothing allocates from is 15 KB of the 48 KB we do not have.
- `configUSE_TIMERS 1` → keep (timer task already accounted for). Confirm
  `configSUPPORT_STATIC_ALLOCATION 1` and
  `INCLUDE_uxTaskGetStackHighWaterMark 1` stay on (both already are) — sysusage
  depends on the latter.
- Add the kernel sources to `SRC` **only when `NCAP_FREERTOS=1`**. Since the
  Makefile style is `-D`-driven rather than `ifeq`-driven, express this with a
  `$(if $(filter 1,$(NCAP_FREERTOS)),<sources>)` expansion — that is the one
  sanctioned exception to the no-`ifeq` style, because the source list genuinely
  cannot be `-D`-gated. Same trick for `sysusage.c`.
- New include paths:
  `-IMiddlewares/Third_Party/FreeRTOS/Source/include`,
  `-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2`.
- New `-D` flags: `-DNCAP_SYSUSAGE=$(NCAP_SYSUSAGE)`, `-DNCAP_FREERTOS=$(NCAP_FREERTOS)`,
  `-DSYSUSAGE_PERIOD_MS=$(SYSUSAGE_PERIOD_MS)`.
- Keep `-mfloat-abi=soft` untouched (matches the prebuilt archive).

## Outcome: what the plan got wrong

Recorded after implementation, so the next reader does not repeat these. The
code is correct; these are the assumptions that did not survive contact with it.

1. **The baseline numbers in "Verified baseline" are stale.** `text 31912 data
   120 bss 82176` predates `ov7670_dvp.c` and `MX_SPI2_Init()`. The real working
   -tree baseline is `text 92468 data 388 bss 63160` (verified from `git archive
   HEAD`). The "~48 KB headroom" figure is really ~67 KB. Everything measured
   against the plan's numbers was off by ~19 KB.
2. **The plan never assigned the executor to anyone.** It said "the existing
   `rclc_executor_t` stays the only rclc caller" and then handed the CPU to
   `vTaskStartScheduler()`, which of course means *nobody* calls the executor. The
   first implementation shipped a firmware that linked, started a scheduler and
   published nothing at all — with a source comment asserting that was "by
   design". Under `NCAP_FREERTOS=1` the executor must run inside a task. It is
   `rcl` in `main.c`, at priority 2.
3. **`configENABLE_FPU` is not a FreeRTOS V10.3.1 option.** The plan's rationale
   ("the CM4F port was skipping FPU context save while the compiler still emitted
   FP instructions") assumed the macro is read by the kernel. Nothing reads it.
   The actual blocker is that `port.c` assembles `vstmdbeq`/`vldmiaeq`, which
   cannot be assembled under `-mfloat-abi=soft` at any `-mfpu`. Only `port.c` is
   now built with `-mfloat-abi=softfp`; the macro stays 0 with a comment
   explaining why it is inert.
4. **`vApplicationGetIdleTaskMemory` / `vApplicationGetTimerTaskMemory` /
   `vApplicationIdleHook` are application-owned, and the plan assigned none of
   them.** They are plain `extern`, not `__weak`, and `cmsis_os2.c` (which does
   carry weak fallbacks) is excluded from `SRC`. The `NCAP_SYSUSAGE=0
   NCAP_FREERTOS=1` row has no `sysusage.c` to own them, so they must live in
   `main.c` and be weak to avoid colliding with `sysusage.c`'s idle hook.
5. **`configUSE_TIMERS 0` requires `INCLUDE_xTimerPendFunctionCall 0`.**
   `timers.c:42` hard-errors otherwise. Cheaper than the ~1356 B of `.bss` the
   timer daemon costs, and nothing in the tree uses it.
6. **Two more config trims were needed to fit the RAM budget:** the feature's own
   `rcl` task pushed `.bss` to +8736 B against an 8192 B cap. `configMAX_PRIORITIES
   56 → 8` (this firmware has 3 priorities) and `configUSE_TIMERS 1 → 0` together
   gave back 2440 B, landing at +6296 B.
7. **The header staleness footgun is worse than "Makefile variables".** The object
   rules depend on the source and on `Makefile` only, so editing
   `FreeRTOSConfig.h` recompiles nothing. `scripts/verify_sysusage.sh` was
   silently verifying the previous run's binary because it reused its scratch
   dirs; it now `rm -rf`s each one first.
8. **`xTaskCreateStatic`'s V10.3.1 argument order is `(fn, name, depth, params,
   prio, stack, tcb)`** — not the CMSIS-v2 `(fn, name, stack, tcb, attrs)` order
   used by `Core/Src/freertos.c`. It returns a `TaskHandle_t`, so it is checked
   against `NULL`, not `pdPASS`.
9. **The plan's `SVC_Handler`/`PendSV_Handler` recipe is a duplicate-symbol error.**
   `FreeRTOSConfig.h:172,173` already `#define vPortSVCHandler SVC_Handler` and
   `xPortPendSVHandler PendSV_Handler`, so `port.c` already *is* the kernel.
   Defining them again in `stm32f4xx_it.c` both duplicates the symbol and,
   because the macro rewrites the call into itself, compiles as infinite
   recursion. The definitions are guarded by `#if !defined(vPortSVCHandler)`.
10. **The plan's `<rclc/support.h>` include does not exist**; `rclc_support_t`
    lives in `rclc/types.h`.

## Acceptance criteria — result

1. `make -C firmware` with both defaults: **63,636 B of `data+bss`**, and the
   feature costs +248 B text / +88 B bss, which is the weak `ncap_cpu_relax()`
   stub and its call site.
2. All four matrix rows build with zero new warnings under `-Wall`.
3. Full feature adds **+6,296 B** of `.bss`, leaving 61,216 B of SRAM free.
4. `diagnostic_msgs__msg__DiagnosticArray` is in the full-feature ELF and no
   `sysusage_*` symbol is in the default ELF.
5. README documents both macros, all four modes, and the gotchas.

Verified by `./scripts/verify_sysusage.sh` (all rows PASS). **Not** verified in
Renode: no `/diagnostics` message has been observed on the wire yet, so the
runtime behaviour remains designed-and-compiled rather than proven.

## Out of scope

- Rebuilt `.msg`/IDL (blocked by the prebuilt `libmicroros.a`).
- `Core/Src/freertos.c` and the CubeMX robot-arm tree — leave untouched.
- Enabling lwIP, or a second XRCE client.
- Renode-side changes or a new peripheral model.
