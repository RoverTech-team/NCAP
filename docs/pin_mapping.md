# Pin mapping — NUCLEO-F446RE + W5500 + OV7670

Pin assignment implemented by the current firmware on a **NUCLEO-F446RE**
(MB1136, LQFP64, STM32F446RET6). This is a bring-up and wiring reference, not
a replacement for the STM32F446 datasheet.

The physical header positions are in
[`pin_mapping_gre.md`](pin_mapping_gre.md). The board pin map is centralized in
[`firmware/board_config.h`](../firmware/board_config.h).

| Area | File |
|---|---|
| Board pin definitions | [`firmware/board_config.h`](../firmware/board_config.h) |
| GPIO / board glue | [`firmware/Core/Src/gpio.c`](../firmware/Core/Src/gpio.c), [`firmware/Core/Inc/main.h`](../firmware/Core/Inc/main.h) |
| SPI2 | [`firmware/Core/Src/spi.c`](../firmware/Core/Src/spi.c) |
| USART2 | [`firmware/Core/Src/usart.c`](../firmware/Core/Src/usart.c) |
| I2C2 | [`firmware/i2c_bus.c`](../firmware/i2c_bus.c) |
| DCMI / DVP | [`firmware/ov7670_dvp.c`](../firmware/ov7670_dvp.c) |
| W5500 control lines | [`firmware/wizchip_port.c`](../firmware/wizchip_port.c) |
| Renode board model | [`renode_configs/platforms/boards/stm32f4_w5500.repl`](../renode_configs/platforms/boards/stm32f4_w5500.repl) |

Clock tree in the current build: SYSCLK 180 MHz, HCLK 180 MHz, PCLK1 45 MHz,
PCLK2 90 MHz.

## At a glance

| Peripheral | Pins | Bus / configuration |
|---|---|---|
| WIZnet W5500 Ethernet | PB13, PB14, PB15, PB12, PB2 (PB1 optional INTn) | SPI2, mode 0, PCLK1/8 = 5.6 MHz default (PCLK1/4 = 11.25 MHz with `W5500_SPI_PRESCALER=SPI_BAUDRATEPRESCALER_4`); active-low SCSn/RSTn |
| OV7670 camera — pixels | PA6, PA4, PB7, PC6–PC9, PC11, PB6, PB8, PB9 | DCMI AF13, 8-bit DVP |
| OV7670 camera — clock / power / reset | PA8, PA11, PB0 | MCO1 16 MHz XCLK; PWDN and RET as GPIO |
| OV7670 camera — control + LSM9DS1 IMU | PB10, PB3 | I2C2 (SCCB), 400 kHz |
| Debug console | PA2, PA3 | USART2, 115200 8N1, RX DMA |
| Button / LED / SWD | PC13, PA5, PA13, PA14 | B1 input; LD2 on PA5; SWD debug |
| Legacy powerSTEP01 / robot arm | none | Not configured by the current firmware |

## W5500 Ethernet — SPI2

| Pin | Signal | Mode | Notes |
|---|---|---|---|
| PB13 | `SPI2_SCK` | AF5, push-pull | 5.6 MHz default (PCLK1 45 MHz / 8); /4 = 11.25 MHz with short wiring |
| PB14 | `SPI2_MISO` | AF5, input | |
| PB15 | `SPI2_MOSI` | AF5, push-pull | |
| PB12 | `W5500_SCSn` | GPIO output PP | Idle high, driven low for each SPI frame |
| PB2 | `W5500_RSTn` | GPIO output PP | Idle high; held low 10 ms during `w5500_hw_init()` |
| PB1 | `W5500_INTn` | not configured | Wired in the Renode model only. Firmware polls the socket RX size and never uses the interrupt. |

SPI2 is configured as mode 0 (`CPOL=0`, `CPHA=1 edge`), MSB-first, software
NSS. SCSn and RSTn are both active low.

PB11, the original RSTn, is **not bonded** on the LQFP64 package. The reset
line moved to PB2 so the firmware can actually drive it on a NUCLEO-F446RE.

The W5500 is deliberately **not** on SPI1. DCMI owns PA4 and PA6 with no
remap, and those were the original SPI1 SCSn/MISO pins. `MX_SPI1_Init()` is no
longer called.

## OV7670 camera — DVP pixels + SCCB control

### Sensor module pinout (physical header)

The bare OV7670 module (no FIFO) breaks out 18 pins. Every one of them is
accounted for below; the names are the silkscreen labels on the module.

| Module pin | MCU pin | Firmware signal | Direction (MCU view) | Notes |
|---|---|---|---|---|
| `D0` | PC6 | `DCMI_D0` | input | DCMI AF13 |
| `D1` | PC7 | `DCMI_D1` | input | |
| `D2` | PC8 | `DCMI_D2` | input | |
| `D3` | PC9 | `DCMI_D3` | input | |
| `D4` | PC11 | `DCMI_D4` | input | |
| `D5` | PB6 | `DCMI_D5` | input | |
| `D6` | PB8 | `DCMI_D6` | input | |
| `D7` | PB9 | `DCMI_D7` | input | |
| `PWDN` | PA11 | `OV7670_PWDN` | output | Active-high power-down; driven low at init (low = awake) |
| `RET` | PB0 | `OV7670_RET` | output | Active-low reset; pulsed low 2 ms, then high |
| `SCL` | PB10 | `I2C2_SCL` | output (open-drain) | SCCB clock, 400 kHz |
| `SDA` | PB3 | `I2C2_SDA` | bidirectional (open-drain) | SCCB data |
| `VS` | PB7 | `DCMI_VSYNC` | input | Frame sync, active high |
| `HS` | PA4 | `DCMI_HSYNC` | input | Line valid / HREF, active high |
| `PLK` | PA6 | `DCMI_PIXCLK` | input | Pixel clock from the sensor |
| `XLK` | PA8 | `OV7670_XCLK` | output | 16 MHz master clock, MCO1 = HSI |
| `3.3` | 3V3 | — | power | 3.3 V only; do not connect to 5 V |
| `GND` | GND | — | power | Common ground with the Nucleo |

`XLK` (also written `XCLK`) is an **input** to the sensor: the STM32 has to
generate it. The firmware routes MCO1 to PA8 in `ov7670_power_init()` with
`HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1)`, giving 16 MHz
from the HSI. 8 MHz (HSI/2) is below the OV7670's 10 MHz XCLK minimum, so it is
not enough even though SCCB may still answer. The sensor needs XCLK running
before its SCCB registers reply, which is why the clock is enabled before the
first I2C2 access.

Silkscreen names vary between module vendors: `PLK` = `PCLK`, `HS` = `HREF`,
`VS` = `VSYNC`, `XLK` = `XCLK`, `RET` = `RESET` (active low), `SCL`/`SDA` =
`SIOC`/`SIOD`, and the supply pin may be marked `3V3`, `VCC` or `VDD`. The
wiring is the same regardless of the label.

`RET` is the sensor reset, separate from the SCCB `COM7[7]` soft reset. PA8 was
previously free and PB0 was unused, so no other peripheral is displaced.

### Parallel pixel bus (DCMI, AF13)

| Pin | DCMI signal | Direction | Notes |
|---|---|---|---|
| PA6 | `DCMI_PIXCLK` | input | Pixel clock from the sensor |
| PA4 | `DCMI_HSYNC` | input | Line valid / HREF; 3.3 V-only pin |
| PB7 | `DCMI_VSYNC` | input | Frame sync |
| PC6 | `DCMI_D0` | input | |
| PC7 | `DCMI_D1` | input | |
| PC8 | `DCMI_D2` | input | |
| PC9 | `DCMI_D3` | input | |
| PC11 | `DCMI_D4` | input | |
| PB6 | `DCMI_D5` | input | |
| PB8 | `DCMI_D6` | input | |
| PB9 | `DCMI_D7` | input | |
| PA11 | `OV7670_PWDN` | GPIO output PP | Driven low at init; high = sensor powered down |

The camera also has two control lines that are not part of the DCMI bus:

| Pin | Signal | Mode | Notes |
|---|---|---|---|
| PA8 | `OV7670_XCLK` (MCO1) | AF0 alternate function | 16 MHz from HSI; sensor master clock |
| PB0 | `OV7670_RET` | GPIO output PP | Active-low reset; low 2 ms, then high |

PA8 is MCO1 and must be configured as an alternate-function pin, not as a plain
GPIO. Both lines are set up in `ov7670_power_init()`, before any SCCB traffic.

This is the real STM32F446 AF13 map. The DCMI is configured for active-high
HSYNC/HREF and VSYNC, and samples on the rising PCLK edge.

The previous firmware used PA17 for HREF and PD6 for D7. Neither pin exists on
the LQFP64 package, so the camera could never have worked on a
NUCLEO-F446RE with that map.

### SCCB / I2C2

| Pin | Signal | Mode | Notes |
|---|---|---|---|
| PB10 | `I2C2_SCL` | AF4, open-drain | Internal pull-up enabled; external pull-ups recommended |
| PB3 | `I2C2_SDA` | AF4, open-drain | Shared with SWO via SB15; leave SB15 open if it loads the bus |

I2C2 runs at 400 kHz by default (`I2C_SCL_HZ`; builds under bring-up use
`I2C_SCL_HZ=100000`) and is shared by:

| Device | 7-bit address | Purpose |
|---|---|---|
| OV7670 | `0x21` | SCCB register configuration (pixels do **not** come over this bus) |
| LSM9DS1 accel/gyro | `0x6B` | IMU |
| LSM9DS1 magnetometer | `0x1E` | IMU |

I2C moved from I2C1/PB6/PB7 to I2C2/PB10/PB3 because PB6 and PB7 are
`DCMI_D5` and `DCMI_VSYNC` on the F446. Camera capture and the IMU cannot
share those pins.

## Debug console — USART2

| Pin | Signal | Mode | Notes |
|---|---|---|---|
| PA2 | `USART2_TX` | AF7 | 115200 8N1, no flow control |
| PA3 | `USART2_RX` | AF7 | RX uses DMA1 Stream 5, channel 4 |

Both are routed to the ST-LINK virtual COM port by default (SB13/SB14 ON,
SB62/SB63 OFF). The old stale W5500 RSTn block that briefly drove PA3 as an
output has been removed.

## Button, LED, SWD

| Pin | Function | Configuration | Notes |
|---|---|---|---|
| PC13 | `B1` user button | EXTI falling edge | Configured but no EXTI15_10 handler and the IRQ is not enabled; currently unused |
| PA5 | Green user LED LD2 | Not configured by firmware | 3.3 V-only pin |
| PA13 | `TMS` / SWDIO | Reset default (AF0) | Debug port; do not repurpose |
| PA14 | `TCK` / SWCLK | Reset default (AF0) | Debug port; do not repurpose |

PD12 and PD14 do not exist on the LQFP64 package; the old Renode LEDs there
have been replaced by LD2 on PA5.

## Legacy powerSTEP01 / robot-arm pins (not built)

`Core/Src/freertos.c`, `Core/powerSTEP01/ps01.c` and `Core/RArm/rarm.c` are
CubeMX robot-arm code that is **not in the Makefile source list**. `MX_GPIO_Init()`
no longer configures their pins, and `MX_SPI1_Init()` is no longer called.

| Pin | Old signal | Current status |
|---|---|---|
| PA5 | `SPI1_SCK` | Free; also LD2 |
| PA6 | `SPI1_MISO` | `DCMI_PIXCLK` |
| PA7 | `SPI1_MOSI` | Free |
| PB6 | `DRV_CS` | `DCMI_D5` |
| PB10 | `BK1` | `I2C2_SCL` |
| PA8 | `BK2` | `OV7670_XCLK` (MCO1) |
| PA9 | `DRV_RST` | Free |

If the motor stack is ever re-enabled, it needs a new pin map; the old one
collides with the camera and the IMU bus.

## Timer channels configured in `tim.c` (not active)

`tim.c` is compiled, but `MX_TIM1_Init()`, `MX_TIM2_Init()`, `MX_TIM4_Init()`
and `MX_TIM8_Init()` are never called from `main()`. The pin assignments below
exist only in the MSP init code and are not claimed by the running firmware.

| Pin | Timer signal | Status / conflict |
|---|---|---|
| PA0 | `TIM2_CH1` | Encoder input, unused |
| PA1 | `TIM2_CH2` | Encoder input, unused |
| PA10 | `TIM1_CH3` | Input capture, unused |
| PA11 | `TIM1_CH4` | Input capture, unused; PA11 is the camera PWDN output |
| PB7 | `TIM4_CH2` | Input capture, unused; PB7 is `DCMI_VSYNC` |
| PB8 | `TIM4_CH3` | Input capture, unused; PB8 is `DCMI_D6` |
| PB9 | `TIM4_CH4` | Input capture, unused; PB9 is `DCMI_D7` |
| PC6 | `TIM8_CH1` | Input capture, unused; PC6 is `DCMI_D0` |
| PC8 | `TIM8_CH3` | Input capture, unused; PC8 is `DCMI_D2` |
| PB5 | `TIM3_CH2` | Not configured by `gpio.c` any more |

Do not enable these timer initialisers without first moving the conflicting
pins above.

## Pin ownership

The current map has no unmanaged conflicts. The remaining shared pins are
intentional:

| Pin | Shared by | Resolution |
|---|---|---|
| PA5 | LD2 / free GPIO | LED loads the pin; use a buffer if you need a clean signal |
| PB3 | `I2C2_SDA` / SWO | SWO is not used; SB15 can be opened if the ST-LINK loads SDA |
| PA13 / PA14 | SWD | Keep free for debug |
| PC13 | B1 button | Do not reuse for DCMI D0 |

Resolved from the old map:

| Old pin | Old use | New use |
|---|---|---|
| PB11 | W5500 RSTn | Not bonded; RSTn moved to PB2 |
| PA17 | DCMI HREF | Not bonded; HREF is PA4 (`DCMI_HSYNC`) |
| PD6 | DCMI D7 | Not bonded; D7 is PB9 |
| PB6 | I2C1_SCL / DRV_CS | DCMI_D5 |
| PB7 | I2C1_SDA | DCMI_VSYNC |
| PB10 | BK1 | I2C2_SCL |
| PA4 | W5500 SCSn / DCMI VSYNC | DCMI_HSYNC |
| PA6 | SPI1_MISO | DCMI_PIXCLK |
| PA8 | Motor `BK2` | `OV7670_XCLK` (MCO1, 16 MHz) |
| PB0 | free | `OV7670_RET` (active-low reset) |

## Renode board mapping

The simulator mirrors the physical map:

| Renode model | Connection |
|---|---|
| `w5500 @ spi2` | `gpioPortB@12` → SCSn, `gpioPortB@2` → RSTn, `gpioPortB@1` → INTn |
| `ov7670 @ i2c2 0x21` | PCLK → `dcmi@0`, VSYNC → `dcmi@1`, HREF → `dcmi@2`, D0–D7 → `dcmi@3..10` |
| `lsm9ds1_imu @ i2c2 0x6b` | LSM9DS1 accelerometer + gyroscope |
| `lsm9ds1_mag @ i2c2 0x1e` | LSM9DS1 magnetometer |
| `GreenLED @ gpioPortA@5` | LD2; firmware does not drive it |

The Renode board does not model the B1 button, the camera PWDN/XCLK/RET
lines, or the legacy motor pins. DCMI/OV7670 signal routing in Renode is
abstract: the model connects the sensor bus directly, so the firmware pin
configuration is validated by the real STM32F446 AF map rather than by the
simulator.

## Bring-up notes

- **W5500:** 3.3 V only. SCSn and RSTn are active low and idle high. Verify
  `VERSIONR == 0x04` on the debug UART. RSTn is PB2, not PB11.
- **Wiring self-test:** `make -C firmware NCAP_PIN_TEST=1`, flash, then run
  `python3 scripts/test_camera_pins.py`. It prints pull-up/pull-down duty and
  edges per data pin and flags `OPEN`, `STUCK LOW` or `STUCK HIGH` lines.
- **DCMI sync mode:** `ESS` must stay 0 (hardware sync). The OV7670's VSYNC is
  a frame-start pulse, so the capture needs `VSPOL=0`; HREF is active high
  (`HSPOL=1`) and data settles on the rising PCLK edge (`PCKPOL=1`).
- **Camera:** wire all four control lines: `XLK` → PA8 (16 MHz MCO1), `PWDN` →
  PA11 (drive low to run; the pin is active-high power-down), `RET` → PB0
  (active low, pulse low then high), `SCL`/`SDA` → PB10/PB3. Without XCLK the
  sensor never answers on SCCB. Pixels only work over the parallel DVP bus,
  not over SCCB. Open SB15 if I2C2 SDA is unreliable.
- **I2C:** PB10/PB3 are open-drain. The firmware enables the internal
  pull-ups, but external 2.2–4.7 kΩ pull-ups are recommended for a real bus
  with three devices.
- **Motor stack:** not built. Re-enabling it requires a new pin map because
  its old pins are now DCMI/I2C2.
- **Debug:** keep PA13/PA14 free for SWD. USART2 on PA2/PA3 is the firmware
  console.
