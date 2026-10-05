# NCAP STM32F4 + W5500 firmware build (milestone: W5500 SPI bring-up + agent probe)

TARGET  = ncap_f4_w5500
BUILD   = build

TOOLCHAIN_PREFIX = arm-none-eabi-
CC     = $(TOOLCHAIN_PREFIX)gcc
AS     = $(TOOLCHAIN_PREFIX)gcc -x assembler-with-cpp
CP     = $(TOOLCHAIN_PREFIX)objcopy
SZ     = $(TOOLCHAIN_PREFIX)size

# Cortex-M4F. -mfloat-abi=soft matches the prebuilt libmicroros.a (built
# soft-float for ARMv7E-M); mixing hard/soft float ABIs will not link.
CPU     = -mcpu=cortex-m4
FPU     = -mfpu=fpv4-sp-d16
FLOAT_ABI = -mfloat-abi=soft
MCU     = $(CPU) -mthumb $(FPU) $(FLOAT_ABI)

MICRO_AGENT_IP ?= 192.168.0.213
MICRO_AGENT_PORT ?= 8888
MICRO_CLIENT_PORT ?= 5000

# Set W5500_TDEBUG=1 to hex-dump every XRCE datagram on the UART (verbose!).
W5500_TDEBUG ?= 0
# Set NCAP_VIDEO_UDP=1 to stream synthetic luma over UDP instead of micro-ROS.
NCAP_VIDEO_UDP ?= 0
VIDEO_HOST_IP ?= 127.0.0.1
VIDEO_WIDTH ?= 320
VIDEO_HEIGHT ?= 240
# Target frame RATE (not a delay). video_udp_step() emits one row block per
# call and the serve loop paces to a deadline, so send cost is accounted for.
VIDEO_FPS ?= 24
VIDEO_LOG_EVERY ?= 30
# Print the achieved emulated frame rate once per simulated second.
VIDEO_FPS_REPORT ?= 0
VIDEO_TX_BUF_KB ?= 4
I2C_SCL_HZ ?= 400000
# DVP capture. YUV422 puts 2 bytes/pixel on the bus, so a frame is
#   VIDEO_WIDTH * VIDEO_HEIGHT * 2
# and DVP_FRAMES * that must fit the F446's 128 KB of RAM:
#   160x120 ->  38400 B/frame   x2 =  76800 B   verified working
#   320x240 -> 153600 B/frame            impossible, does not fit at all
# 320x240 full-width YUV422 therefore needs external SDRAM. YUV420 is not an
# option (the OV7670 has no such output mode) and neither is a DCMI crop window
# (Renode's DMA has no circular mode, so frame alignment cannot be held).
# 1 = stream real OV7670 pixels over DVP/DCMI; 0 = synthetic luma.
NCAP_DVP ?= 0
DVP_FRAMES ?= 2
# Rows captured via DCMI crop; 0 = the sensor's full output height.
DVP_CROP_ROWS ?= 0
# Row-0 coherence check. DISABLED by default: it was measured against the crop
# path and cannot work there, because the misalignment is consistent frame to
# frame (row-0 means 78.3, 79.1, 79.1, 79.1...), so a misaligned frame looks just
# as temporally coherent as an aligned one. Useful only if the capture offset
# varies. See ov7670_dvp.h.
DVP_ALIGN_CHECK ?= 0
# 1 = ask the DCMI to drop chroma in hardware (byte select), so a frame is
# luma only at 1 byte/pixel. UNVERIFIED on F4 - see ov7670_dvp.h.
DVP_LUMA_ONLY ?= 0
DVP_ALIGN_TOLERANCE ?= 48
# Unverified on F4: see ov7670_dvp.h. RM0090 marks CR 31:15 reserved.
NCAP_DCMI_TRY_BSM ?= 0
DVP_POLL_ONLY ?= 0
VIDEO_SRC_PORT ?= 5000
NCAP_OV7670 ?= 0
NODE_ID ?= 0
# System-usage telemetry published to ROS 2 /diagnostics as a DiagnosticArray.
# NCAP_SYSUSAGE=1 compiles sysusage.c. 0 (default) leaves it out of SRC
# entirely, so the feature costs nothing when off: no symbols, no .bss, and
# sysusage.c is absent from the ELF. It builds standalone *and* on FreeRTOS:
#   NCAP_SYSUSAGE=1 NCAP_FREERTOS=0 -> sysusage_publish() samples inline from
#       HAL_GetTick(), called on the rclc executor's own 1 Hz timer.
#   NCAP_SYSUSAGE=1 NCAP_FREERTOS=1 -> a statically allocated sampler task
#       samples the kernel; publish still happens on the executor's thread, so
#       the single XRCE session is never touched from two tasks.
# Verified: 1 (needs sysusage.c, i.e. docs/plan-sysusage.md). 0 = default.
NCAP_SYSUSAGE ?= 0
# NCAP_FREERTOS=1 builds the FreeRTOS V10.3.1 kernel that is vendored under
# Middlewares/Third_Party/FreeRTOS/Source/ but has never been compiled (not in
# SRC, no include path) and makes main.c hand the CPU to vTaskStartScheduler()
# instead of running its bare-metal spin loop. This is not free: on a 128 KB
# part with data+bss already ~63 KB, so FreeRTOSConfig.h drops
# configTOTAL_HEAP_SIZE 15360 -> 4096 to pay for it, and the sysusage task is
# xTaskCreateStatic so it never touches the heap at all.
# Verified: 1 needs SVC_Handler/PendSV_Handler/SysTick dispatch in
# stm32f4xx_it.c; without them the first context switch lands in the weak
# Default_Handler and spins. 0 = default.
NCAP_FREERTOS ?= 0
# Publish cadence for sysusage_publish(), consumed by the sysusage module only.
# 1000 matches main.c's 1 Hz executor timer, so a snapshot is at most one
# period old. Smaller values just spend UART bandwidth on a topic nobody polls
# that often. Anything >= 1 works; verified at 1000.
SYSUSAGE_PERIOD_MS ?= 1000
# TCP port for the video stream. Pick something not already bound on the host.
VIDEO_PORT ?= 18080
C_DEFS  = -DUSE_HAL_DRIVER -DSTM32F446xx -DNR_FPU_USED=1 -DW5500_TDEBUG=$(W5500_TDEBUG) -DNCAP_VIDEO_UDP=$(NCAP_VIDEO_UDP) -DVIDEO_PORT=$(VIDEO_PORT) -DVIDEO_HOST_IP=\"$(VIDEO_HOST_IP)\" -DVIDEO_WIDTH=$(VIDEO_WIDTH) -DVIDEO_HEIGHT=$(VIDEO_HEIGHT) -DVIDEO_FPS=$(VIDEO_FPS) -DVIDEO_LOG_EVERY=$(VIDEO_LOG_EVERY) -DVIDEO_FPS_REPORT=$(VIDEO_FPS_REPORT) -DVIDEO_TX_BUF_KB=$(VIDEO_TX_BUF_KB) -DI2C_SCL_HZ=$(I2C_SCL_HZ) -DDVP_FRAMES=$(DVP_FRAMES) -DDVP_CROP_ROWS=$(DVP_CROP_ROWS) -DDVP_LUMA_ONLY=$(DVP_LUMA_ONLY) -DDVP_ALIGN_CHECK=$(DVP_ALIGN_CHECK) -DDVP_ALIGN_TOLERANCE=$(DVP_ALIGN_TOLERANCE) -DNCAP_DCMI_TRY_BSM=$(NCAP_DCMI_TRY_BSM) -DDVP_POLL_ONLY=$(DVP_POLL_ONLY) -DVIDEO_SRC_PORT=$(VIDEO_SRC_PORT) -DNCAP_DVP=$(NCAP_DVP) -DNCAP_OV7670=$(NCAP_OV7670) -DNODE_ID=$(NODE_ID) -DNCAP_SYSUSAGE=$(NCAP_SYSUSAGE) -DNCAP_FREERTOS=$(NCAP_FREERTOS) -DSYSUSAGE_PERIOD_MS=$(SYSUSAGE_PERIOD_MS) \
          -DMICRO_AGENT_IP=\"$(MICRO_AGENT_IP)\" -DW5500_AGENT_PORT=$(MICRO_AGENT_PORT) -DW5500_TRANSPORT_PORT=$(MICRO_CLIENT_PORT)

C_INCLUDES = \
  -ICore/Inc \
  -IDrivers/STM32F4xx_HAL_Driver/Inc \
  -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
  -IDrivers/CMSIS/Include \
  -I. \
  -IioLibrary \
  -IioLibrary/W5500 \
  -Imicroros/include \
  $(FREERTOS_INC) \
  -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2

# The kernel's own include/ is gated because it holds generically named headers
# (task.h, queue.h, list.h, atomic.h ...) that have no business shadowing the
# rest of the tree on a build that is not using FreeRTOS. CMSIS_RTOS_V2 above is
# unconditional: headers only, adds no code, and it keeps sysusage.c free to
# include cmsis_os2.h in either matrix row without a build-file round trip.
# ARM_CM4F is not optional: include/portable.h does a bare #include "portmacro.h"
# and the only portmacro.h in this tree sits next to the ARM_CM4F port, so
# without it all six kernel .c files die with
# "portable.h:52: fatal error: portmacro.h: No such file or directory".
FREERTOS_INC = $(if $(filter 1,$(NCAP_FREERTOS)),\
  -IMiddlewares/Third_Party/FreeRTOS/Source/include \
  -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F)

SRC = \
  Core/Src/main.c \
  Core/Src/gpio.c \
  Core/Src/spi.c \
  Core/Src/usart.c \
  Core/Src/dma.c \
  Core/Src/tim.c \
  video_udp.c \
  i2c1.c \
  imu.c \
  ov7670.c \
  ov7670_dvp.c \
  Core/Src/stm32f4xx_hal_msp.c \
  Core/Src/stm32f4xx_it.c \
  Core/Src/syscalls.c \
  Core/Src/sysmem.c \
  Core/Src/system_stm32f4xx.c \
  Core/Src/system_clock.c \
  Core/Src/posix_time.c \
  Core/Src/io_uart.c \
  wizchip_port.c \
  w5500_transport.c \
  ioLibrary/socket.c \
  ioLibrary/W5500/w5500.c \
  $(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c)

# Conditional sources. A -D flag gates code *inside* a .c file, but it cannot
# gate a source list: -DNCAP_FREERTOS=1 still leaves Make compiling exactly the
# files named above. That is the single case the ?.=/-D style genuinely cannot
# express, so it is written as $(if $(filter 1,$(VAR)),<list>) - still no
# ifeq, still overridable from the command line, and it expands to nothing at
# all when the toggle is 0, which is what keeps the default link byte-identical.

# Enumerated one by one, deliberately NOT $(wildcard Source/*.c): a wildcard
# would silently sweep in CMSIS_RTOS_V2/cmsis_os2.c and the two dead-weight
# files below, none of which are wanted. Dropping any file here is a link error
# at build time, never a silent behaviour change.
#
# The FreeRTOS kernel itself - the minimum that links and runs a task:
FREERTOS_SRC = \
  Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
  Middlewares/Third_Party/FreeRTOS/Source/queue.c \
  Middlewares/Third_Party/FreeRTOS/Source/list.c \
  Middlewares/Third_Party/FreeRTOS/Source/timers.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c

# What is in Source/ and deliberately NOT in FREERTOS_SRC:
#   CMSIS_RTOS_V2/cmsis_os2.c  excluded on purpose. sysusage.c uses the native
#     kernel API (xTaskCreateStatic, uxTaskGetSystemState) and main.c keeps
#     calling rclc bare-metal, so nothing needs the CMSIS-RTOS-v2 wrapper. It is
#     not small (osThreadNew/osThreadGetState and friends) and it adds an
#     osPThreads/idle-thread of its own. Core/Src/freertos.c, which does use
#     it, is CubeMX robot-arm code and is not built at all.
#   event_groups.c  xEventGroup* is unused. tasks.c does not reference it, so
#     omitting it cannot break the link.
#   croutine.c      configUSE_CO_ROUTINES is 0, so it would compile to an empty
#     object anyway.
#   stream_buffer.c  no message buffers / streams are used.
FREERTOS_EXCLUDED = \
  Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
  Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
  Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
  Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c

# The sysusage module lives at firmware/sysusage.c (firmware/ is already -I.).
SYSUSAGE_SRC = sysusage.c

SRC += $(if $(filter 1,$(NCAP_SYSUSAGE)),$(SYSUSAGE_SRC))
SRC += $(if $(filter 1,$(NCAP_FREERTOS)),$(FREERTOS_SRC))

# NCAP_FREERTOS=1 gives the CPU to vTaskStartScheduler(), which means main.c's
# cooperative rclc_executor_spin_some() + video_udp_step() multiplexing simply
# stops happening: with NCAP_VIDEO_UDP=2, video_udp_step() *is* the main loop, so
# the video would silently stop after the scheduler starts. The two personalities
# cannot be combined without turning video_udp_step() into a task, which is out
# of scope. Enforce it at build time instead of shipping firmware that quietly
# stops streaming. Written with a simply-expanded $(if ...) rather than ifeq so
# the file keeps its ?=/-D-only style; it evaluates during parsing, so the error
# fires before any rule runs. Expands to nothing when the pair is not both set.
CHECK_FREERTOS_X_VIDEO := $(if $(filter 1,$(NCAP_FREERTOS)),\
  $(if $(filter 2,$(NCAP_VIDEO_UDP)),\
    $(error NCAP_FREERTOS=1 and NCAP_VIDEO_UDP=2 are mutually exclusive. A FreeRTOS scheduler owns the CPU loop, so main.c's cooperative rclc_executor_spin_some() + video_udp_step() multiplexing is lost and the video would never be stepped. Build with NCAP_FREERTOS=1 (micro-ROS + /diagnostics, no UDP video) or with NCAP_VIDEO_UDP=2 (UDP video, bare-metal loop, no FreeRTOS); NCAP_SYSUSAGE works fine with either on its own. To get both you need video_udp_step() driven from a task, which is not implemented.)))

# DVP ring size in bytes, from the actual build parameters - the same arithmetic
# ov7670_dvp.h uses: DVP_FRAME_BYTES is WIDTH x stream-height x (2, or 1 when
# DVP_LUMA_ONLY drops chroma in hardware), times DVP_FRAMES. The stream height
# is DVP_CROP_ROWS when a crop is set, else the full VIDEO_HEIGHT.
DVP_BPP := $(if $(filter 1,$(DVP_LUMA_ONLY)),1,2)
DVP_H := $(if $(filter 0,$(DVP_CROP_ROWS)),$(VIDEO_HEIGHT),$(DVP_CROP_ROWS))
DVP_RING := $(shell echo $$(( $(VIDEO_WIDTH) * $(DVP_H) * $(DVP_BPP) * $(DVP_FRAMES) )))

# Baseline RAM (data + bss, which already includes the 1536 B heap/stack reserve)
# measured per personality with arm-none-eabi-size. VIDEO_WIDTH/HEIGHT were
# verified to not move .bss in any personality - only the ring scales - so these
# numbers are exact, not conservative. F446 SRAM is 131072 B.
DVP_BASE := $(if $(filter 1,$(NCAP_VIDEO_UDP)),\
  $(if $(filter 1,$(NCAP_FREERTOS)),5568,5368),\
  $(if $(filter 2,$(NCAP_VIDEO_UDP)),\
    $(if $(filter 1,$(NCAP_SYSUSAGE)),67748,66380),\
    $(if $(filter 1,$(NCAP_SYSUSAGE)),\
      $(if $(filter 1,$(NCAP_FREERTOS)),70056,65004),\
      $(if $(filter 1,$(NCAP_FREERTOS)),65712,63636))))
DVP_TOTAL := $(shell echo $$(( $(DVP_BASE) + $(DVP_RING) )))

# Reject only the combinations that genuinely overflow. The old version of this
# check banned NCAP_DVP=1 next to any micro-ROS personality outright, which was
# wrong: micro-ROS + QQVGA at DVP_FRAMES=1 is 63636 + 38400 = 102036 B and fits
# with room, and even +sysusage +freertos lands at 108456 B. What does not fit
# is a second frame (76800 B of ring on top of the ~63 KB RMW pool = ~140 KB),
# and that is what this catches. 1024 B of margin covers section alignment -
# the estimate was verified within 8 B of the linked size on every row.
# NCAP_FREERTOS=1 + NCAP_VIDEO_UDP=2 never reaches here: the pair is rejected by
# CHECK_FREERTOS_X_VIDEO above for scheduling reasons, not RAM.
CHECK_DVP_FIT := $(if $(filter 1,$(NCAP_DVP)),\
  $(if $(shell [ $(DVP_TOTAL) -gt 130048 ] && echo yes),\
    $(error NCAP_DVP=1 does not fit in RAM in this configuration: baseline $(DVP_BASE) B + ring $(DVP_RING) B (VIDEO_WIDTH=$(VIDEO_WIDTH) x $(DVP_H) x $(DVP_BPP)B/px x DVP_FRAMES=$(DVP_FRAMES)) = $(DVP_TOTAL) B against 131072 B of F446 SRAM. The F446 has no SDRAM on this board. Fit it by lowering DVP_FRAMES (one QQVGA YUV422 frame is 38400 B and fits next to micro-ROS with 22 KB to spare, even with NCAP_SYSUSAGE=1 NCAP_FREERTOS=1), setting DVP_LUMA_ONLY=1 (halves the ring, unverified on F4 - see ov7670_dvp.h), cropping with DVP_CROP_ROWS, or dropping to NCAP_DVP=0 for synthetic luma.)))

LDSCRIPT = STM32F446XX_FLASH.ld
LDFLAGS  = $(MCU) -specs=nano.specs -T$(LDSCRIPT) \
           -Wl,-Map=$(BUILD)/$(TARGET).map,--cref -Wl,--gc-sections \
           -L. -lmicroros -lm

OBJ  = $(addprefix $(BUILD)/,$(SRC:.c=.o))
vpath %.c $(sort $(dir $(SRC)))

all: $(BUILD)/$(TARGET).elf size

$(BUILD)/%.o: %.c Makefile
	@mkdir -p $(dir $@)
	$(CC) -c $(C_DEFS) $(C_INCLUDES) $(MCU) $(OPT) -Wall -fdata-sections -ffunction-sections $< -o $@

$(BUILD)/%.o: %.s Makefile
	@mkdir -p $(dir $@)
	$(AS) -c $(C_DEFS) $(C_INCLUDES) $(MCU) $< -o $@

# The ARM_CM4F port is the only Cortex-M4 port vendored here, and it is the one
# file in the kernel with hand-written VFP assembly: xPortPendSVHandler saves and
# restores the "high" half of the FPU bank with
#   vstmdbeq r0!, {s16-s31} / vldmiaeq r0!, {s16-s31}
# Those cannot be assembled under this build's -mfloat-abi=soft, whatever
# -mfpu says: with the soft ABI the driver hands gas no VFP at all, so gas
# rejects them with
#   Error: selected FPU does not support instruction -- `vstmdbeq r0!,{s16-s31}'
# (verified for -mfpu=fpv4-sp-d16, fpv5-d16 and vfpv4-d16; all three assemble
# only once -mfloat-abi is softfp or hard.) So port.c alone is built softfp.
# Blast radius: one file, and it is the smallest possible change:
#   - port.c passes and returns no float, so softfp vs soft changes no
#     signature and no calling convention. readelf -A on the resulting port.o
#     shows "Tag_FP_arch: VFPv4-D16" and "Tag_ABI_HardFP_use: SP only" but *no*
#     Tag_ABI_VFP_args, which is what a caller-visible ABI difference would
#     show; a -mfloat-abi=soft object shows neither. It links against the
#     soft-float libmicroros.a and the actual NCAP_FREERTOS=1 link is clean.
#   - -mfpu is unchanged, so it is still fpv4-sp-d16 here;
#   - at run time the two instructions sit behind a "tst r14, #0x10"
#     lazy-stacking test on the stacked xPSR, and under -mfloat-abi=soft no
#     other translation unit emits an FP instruction at all (every float op is
#     an __aeabi_fadd/__aeabi_dadd call that preserves the VFP itself), so lazy
#     stacking never happens and neither instruction is ever reached.
PORT_FLOAT_ABI = -mfloat-abi=softfp

$(BUILD)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.o: $(BUILD)/%.o: %.c Makefile
	@mkdir -p $(dir $@)
	$(CC) -c $(C_DEFS) $(C_INCLUDES) $(MCU) $(PORT_FLOAT_ABI) $(OPT) -Wall -fdata-sections -ffunction-sections $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJ) startup_stm32f446xx.s
	$(CC) $(OBJ) startup_stm32f446xx.s $(LDFLAGS) -o $@
	$(CP) -O binary $< $(BUILD)/$(TARGET).bin
	$(CP) -O ihex   $< $(BUILD)/$(TARGET).hex

size: $(BUILD)/$(TARGET).elf
	$(SZ) $<

clean:
	rm -rf $(BUILD)

.PHONY: all clean size
