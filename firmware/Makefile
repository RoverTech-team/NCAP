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
VIDEO_FPS_MS ?= 200
VIDEO_SRC_PORT ?= 5000
NCAP_OV7670 ?= 0
NODE_ID ?= 0
# TCP port for the video stream. Pick something not already bound on the host.
VIDEO_PORT ?= 18080
C_DEFS  = -DUSE_HAL_DRIVER -DSTM32F446xx -DNR_FPU_USED=1 -DW5500_TDEBUG=$(W5500_TDEBUG) -DNCAP_VIDEO_UDP=$(NCAP_VIDEO_UDP) -DVIDEO_PORT=$(VIDEO_PORT) -DVIDEO_HOST_IP=\"$(VIDEO_HOST_IP)\" -DVIDEO_WIDTH=$(VIDEO_WIDTH) -DVIDEO_HEIGHT=$(VIDEO_HEIGHT) -DVIDEO_FPS_MS=$(VIDEO_FPS_MS) -DVIDEO_SRC_PORT=$(VIDEO_SRC_PORT) -DNCAP_OV7670=$(NCAP_OV7670) -DNODE_ID=$(NODE_ID) \
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
  -Imicroros/include

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

$(BUILD)/$(TARGET).elf: $(OBJ) startup_stm32f446xx.s
	$(CC) $(OBJ) startup_stm32f446xx.s $(LDFLAGS) -o $@
	$(CP) -O binary $< $(BUILD)/$(TARGET).bin
	$(CP) -O ihex   $< $(BUILD)/$(TARGET).hex

size: $(BUILD)/$(TARGET).elf
	$(SZ) $<

clean:
	rm -rf $(BUILD)

.PHONY: all clean size
