# Physical pin mapping — NUCLEO-F446RE

Physical connector mapping for the NCAP firmware on a **NUCLEO-F446RE**
(MB1136, LQFP64, STM32F446RET6).

> **Board assumption:** “NUCLEO-F446GRE” is not an ST order code. The closest
> Nucleo-64 board is **NUCLEO-F446RE**, which is what this document assumes.
> If you actually have a **NUCLEO-F446ZE** (LQFP144), the header pin numbers are
> different; the firmware map in `board_config.h` still targets the RE.

The logical assignment is in [`pin_mapping.md`](pin_mapping.md) and
[`firmware/board_config.h`](../firmware/board_config.h). This document adds the
physical **CN5 / CN6 / CN7 / CN8 / CN9 / CN10** positions.

References:

- ST UM1724, *STM32 Nucleo-64 boards (MB1136)* — connector tables and board
  schematic.
- ST DS10693, *STM32F446xC/E* — package pinout and alternate-function table.

## Header orientation

| Connector | Type | Pins | Notes |
|---|---|---|---|
| CN7 | ST morpho, 2×19 | 1–38 | Pin 1 at the ST-LINK end; odd pins on the outer column |
| CN10 | ST morpho, 2×19 | 1–38 | Pin 1 at the ST-LINK end; odd pins on the inner column |
| CN5 | Arduino digital | 1–8 | D8–D13, GND, AREF |
| CN6 | Arduino power | 1–8 | NC, IOREF, RESET, 3V3, 5V, GND, GND, VIN |
| CN8 | Arduino analog | 1–6 | A0–A5 |
| CN9 | Arduino digital | 1–8 | D0–D7 |

Where a signal appears on both a morpho and an Arduino header, both positions
are listed below.

## Firmware pins in port order

| MCU pin | Firmware use | ST morpho | Arduino | Notes |
|---|---|---|---|---|
| PA0 | `TIM2_CH1` (unused) | CN7-28 | CN8-1 / A0 | Timer init never called |
| PA1 | `TIM2_CH2` (unused) | CN7-30 | CN8-2 / A1 | Timer init never called |
| PA2 | `USART2_TX` | CN10-35 | CN9-2 / D1 | ST-LINK virtual COM port |
| PA3 | `USART2_RX` | CN10-37 | CN9-1 / D0 | ST-LINK virtual COM port |
| PA4 | `DCMI_HSYNC` | CN7-32 | CN8-3 / A2 | 3.3 V-only pin |
| PA5 | Green LED LD2 | CN10-11 | CN5-6 / D13 | 3.3 V-only; firmware does not drive it |
| PA6 | `DCMI_PIXCLK` | CN10-13 | CN5-5 / D12 | |
| PA7 | free | CN10-15 | CN5-4 / D11 | Legacy SPI1 pin, no longer configured |
| PA8 | `OV7670_XCLK` (MCO1) | CN10-23 | CN9-8 / D7 | 16 MHz master clock for the sensor |
| PA9 | free | CN10-21 | CN5-1 / D8 | |
| PA10 | `TIM1_CH3` (unused) | CN10-33 | CN9-3 / D2 | Timer init never called |
| PA11 | `OV7670_PWDN`, `TIM1_CH4` | CN10-14 | — | Camera power-down output |
| PA13 | SWDIO | CN7-13 | — | Debug port |
| PA14 | SWCLK | CN7-15 | — | Debug port |
| PB0 | `OV7670_RET` | CN7-34 | CN8-4 / A3 | Active-low camera reset |
| PB1 | `W5500_INTn` (optional) | CN10-24 | — | Not used by firmware |
| PB2 | `W5500_RSTn` | CN10-22 | — | Active low; new reset pin |
| PB3 | `I2C2_SDA` | CN10-31 | CN9-4 / D3 | Shared with SWO via SB15 |
| PB5 | free | CN10-29 | CN9-5 / D4 | Timer pin no longer configured |
| PB6 | `DCMI_D5` | CN10-17 | CN5-3 / D10 | |
| PB7 | `DCMI_VSYNC` | CN7-21 | — | |
| PB8 | `DCMI_D6` | CN10-3 | CN5-10 / D15 | |
| PB9 | `DCMI_D7` | CN10-5 | CN5-9 / D14 | |
| PB10 | `I2C2_SCL` | CN10-25 | CN9-7 / D6 | |
| PB12 | `W5500_SCSn` | CN10-16 | — | Active low |
| PB13 | `SPI2_SCK` | CN10-30 | — | |
| PB14 | `SPI2_MISO` | CN10-28 | — | |
| PB15 | `SPI2_MOSI` | CN10-26 | — | |
| PC6 | `DCMI_D0` | CN10-4 | — | |
| PC7 | `DCMI_D1` | CN10-19 | CN5-2 / D9 | |
| PC8 | `DCMI_D2` | CN10-2 | — | |
| PC9 | `DCMI_D3` | CN10-1 | — | |
| PC11 | `DCMI_D4` | CN7-2 | — | |
| PC13 | `B1` user button | CN7-23 | — | Active low; weak pin |
| PD2 | free | CN7-4 | — | |
| PC0–PC5, PC10, PC12 | free | various | — | Not claimed by the firmware |

Pins that do **not** exist on the LQFP64 package and are no longer used:
**PB11**, **PA17**, **PD6**, **PD12**, **PD14**.

## W5500 Ethernet

| Signal | MCU pin | Physical pin | Status |
|---|---|---|---|
| `SPI2_SCK` | PB13 | CN10-30 | OK |
| `SPI2_MISO` | PB14 | CN10-28 | OK |
| `SPI2_MOSI` | PB15 | CN10-26 | OK |
| `W5500_SCSn` | PB12 | CN10-16 | OK, active low |
| `W5500_RSTn` | PB2 | CN10-22 | OK, active low |
| `W5500_INTn` | PB1 | CN10-24 | Optional; firmware does not use it |

The W5500 is 3.3 V logic. Do not drive its SPI/control pins with 5 V levels
(a 5 V-input module powers its own regulator, but the bus is still 3.3 V).
SCSn and RSTn idle high; the firmware holds RSTn low for 10 ms during
`w5500_hw_init()`.

> **CN10‑10 and CN10‑18 are not connected** on the MB1136 morpho header
> (UM1724 Table 29). SCSn must go to CN10‑16 (PB12); a wire on CN10‑10 leaves
> the W5500 permanently deselected and `VERSIONR` reads `0xFF`.

## OV7670 camera

### Module pinout (physical header)

| Sensor signal | MCU pin | Physical pin | Note |
|---|---|---|---|
| `PLK` / `PIXCLK` | PA6 | CN5-5 / CN10-13 | `DCMI_PIXCLK` |
| `HS` / `HREF` (line valid) | PA4 | CN8-3 / CN7-32 | `DCMI_HSYNC`; 3.3 V-only |
| `VS` / `VSYNC` (frame valid) | PB7 | CN7-21 | `DCMI_VSYNC` |
| `D0` | PC6 | CN10-4 | |
| `D1` | PC7 | CN5-2 / CN10-19 | |
| `D2` | PC8 | CN10-2 | |
| `D3` | PC9 | CN10-1 | |
| `D4` | PC11 | CN7-2 | |
| `D5` | PB6 | CN5-3 / CN10-17 | |
| `D6` | PB8 | CN5-10 / CN10-3 | |
| `D7` | PB9 | CN5-9 / CN10-5 | |
| `PWDN` | PA11 | CN10-14 | Drive low to run (active-high power-down) |
| `XLK` / `XCLK` | PA8 | CN10-23 / CN9-8 | 16 MHz MCO1 (HSI); sensor master clock |
| `RET` | PB0 | CN7-34 / CN8-4 | Active-low reset; pulse low 2 ms, then high |
| `3.3` / `VCC` | 3V3 | CN6-4 | 3.3 V supply; do not use 5 V |
| `GND` | GND | CN6-6 or CN6-7 | Common ground with the Nucleo |

The left column uses the module silkscreen names; the firmware signal is the
STM32 name. Aliases seen on other modules: `PLK` = `PCLK`, `HS` = `HREF`,
`VS` = `VSYNC`, `XLK` = `XCLK`, `RET` = `RESET`, `SCL`/`SDA` = `SIOC`/`SIOD`.

This is the real STM32F446 DCMI AF13 map. The DCMI samples on the rising PCLK
edge and expects active-high HREF and VSYNC.

### SCCB / I2C2

| Signal | MCU pin | Physical pin | Notes |
|---|---|---|---|
| `I2C2_SCL` | PB10 | CN10-25 / CN9-7 | |
| `I2C2_SDA` | PB3 | CN10-31 / CN9-4 | Shared with SWO via SB15 |

The OV7670 SCCB interface and the LSM9DS1 IMU share this bus. Fit external
2.2–4.7 kΩ pull-ups to 3.3 V. If I2C2 SDA is unreliable with the ST-LINK
attached, open solder bridge **SB15**, which connects PB3 to SWO.

There is a separate hardware reset on `RET`/PB0, plus the SCCB `COM7[7]` soft
reset. Pixels only travel over the parallel DVP bus.

## USART2 debug console

| Signal | MCU pin | Physical pin | Notes |
|---|---|---|---|
| `USART2_TX` | PA2 | CN9-2 / CN10-35 | Routed to ST-LINK virtual COM port |
| `USART2_RX` | PA3 | CN9-1 / CN10-37 | Routed to ST-LINK virtual COM port |

The ST-LINK USB connection provides the console at 115200 8N1. By default
SB13/SB14 are ON (ST-LINK VCP) and SB62/SB63 are OFF (Arduino D0/D1).

## Button, LED, and SWD

| Function | MCU pin | Physical pin | Notes |
|---|---|---|---|
| User button B1 | PC13 | CN7-23 | Active low; EXTI configured but IRQ not enabled |
| Green user LED LD2 | PA5 | CN5-6 / CN10-11 | Not driven by the current firmware |
| SWDIO | PA13 | CN7-13 | ST-LINK debug |
| SWCLK | PA14 | CN7-15 | ST-LINK debug |

The only user LED on the NUCLEO-F446RE is LD2 on PA5. PD12/PD14 are not bonded
on the LQFP64 package.

## Free pins

These are not claimed by the current firmware and are available for expansion:

`PA0`, `PA1`, `PA7`, `PA9`, `PA10`, `PA15`, `PB4`, `PB5`,
`PC0`–`PC5`, `PC10`, `PC12`, `PD2`.

Keep `PA13`/`PA14` for SWD and `PC13` for the user button. `PB3` is used for
I2C2 SDA, and `PB4` is NJTRST (usable as GPIO only if JTAG is not needed).

## Renode board mapping

The simulator mirrors this map:

| Renode model | Connection |
|---|---|
| `w5500 @ spi2` | `gpioPortB@12` → SCSn, `gpioPortB@2` → RSTn, `gpioPortB@1` → INTn |
| `ov7670 @ i2c2 0x21` | PCLK/VSYNC/HREF/D0–D7 → `dcmi@0..10` |
| `lsm9ds1_imu @ i2c2 0x6b` | LSM9DS1 accelerometer + gyroscope |
| `lsm9ds1_mag @ i2c2 0x1e` | LSM9DS1 magnetometer |
| `GreenLED @ gpioPortA@5` | LD2; firmware does not drive it |

The DCMI/OV7670 signal routing in Renode is abstract: the sensor model connects
directly to the DCMI model, so the firmware pin configuration is validated by
the real STM32F446 AF map rather than by the simulator.

## Bring-up notes

1. **W5500:** wire SCSn to PB12 (CN10-16) and RSTn to PB2 (CN10-22). Do not
   use the old PB11 position; CN10-18 is not connected.
2. **Camera:** wire the DCMI bus exactly as in the table, plus `XLK` to PA8,
   `RET` to PB0, `PWDN` to PA11 and `SCL`/`SDA` to PB10/PB3. The old PA17/PD6
   positions do not exist on this package.
3. **I2C:** SCL is PB10, SDA is PB3. This is not the Arduino default I2C
   position (D15/D14), so do not use the board's default I2C header labels.
4. **Power:** PA4 and PA5 are 3.3 V-only. The W5500 and camera are 3.3 V
   devices; use a common ground.
5. **Debug:** USART2 is on the ST-LINK VCP; SWD is PA13/PA14.
