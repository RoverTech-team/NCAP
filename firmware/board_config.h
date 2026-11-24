// Board pin map for the NUCLEO-F446RE (MB1136, LQFP64, STM32F446RET6).
//
// This is the single place where the firmware's physical wiring is defined.
// The Renode board description (renode_configs/platforms/boards/
// stm32f4_w5500.repl) mirrors these assignments:
//
//   W5500  SPI2: PB13=SCK, PB14=MISO, PB15=MOSI, PB12=SCSn, PB2=RSTn
//   I2C2   SCL=PB10, SDA=PB3 (OV7670 SCCB + LSM9DS1 IMU)
//   DCMI   PIXCLK=PA6, HSYNC=PA4, VSYNC=PB7,
//          D0..D3=PC6..PC9, D4=PC11, D5=PB6, D6=PB8, D7=PB9
//   CAM    XCLK=PA8 (MCO1 = HSI = 16 MHz), PWDN=PA11, RET=PB0
//   UART2  PA2=TX, PA3=RX (ST-LINK virtual COM port)
//   B1     PC13
//
// Pins that do NOT exist on the LQFP64 package are deliberately absent:
// PB11, PA17 and PD6. The old firmware used PB11 for W5500 RSTn, PA17 for
// DCMI_HREF and PD6 for DCMI_D7; none of those can be wired on a
// NUCLEO-F446RE. See docs/pin_mapping_gre.md.

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "main.h"

// ---- W5500 Ethernet (SPI2) ----

#define W5500_CS_GPIO_PORT   GPIOB
#define W5500_CS_PIN         GPIO_PIN_12
#define W5500_RST_GPIO_PORT  GPIOB
#define W5500_RST_PIN        GPIO_PIN_2
#define W5500_INT_GPIO_PORT  GPIOB
#define W5500_INT_PIN        GPIO_PIN_1

// ---- I2C2: OV7670 SCCB and LSM9DS1 ----

#define NCAP_I2C             I2C2
#define NCAP_I2C_GPIO_PORT   GPIOB
#define NCAP_I2C_SCL_PIN     GPIO_PIN_10
#define NCAP_I2C_SDA_PIN     GPIO_PIN_3
#define NCAP_I2C_GPIO_AF     GPIO_AF4_I2C2
#define NCAP_I2C_CLK_ENABLE() do { __HAL_RCC_I2C2_CLK_ENABLE(); } while (0)

// PB3 is also SWO on the Nucleo. The firmware does not use SWO; open SB15
// if the ST-LINK's SWO input loads the I2C2 SDA line.

// ---- OV7670 DCMI (AF13) ----

#define DVP_PIXCLK_GPIO_PORT GPIOA
#define DVP_PIXCLK_PIN       GPIO_PIN_6
#define DVP_HSYNC_GPIO_PORT  GPIOA
#define DVP_HSYNC_PIN        GPIO_PIN_4
#define DVP_VSYNC_GPIO_PORT  GPIOB
#define DVP_VSYNC_PIN        GPIO_PIN_7
#define DVP_D0_GPIO_PORT     GPIOC
#define DVP_D0_PIN           GPIO_PIN_6
#define DVP_D1_GPIO_PORT     GPIOC
#define DVP_D1_PIN           GPIO_PIN_7
#define DVP_D2_GPIO_PORT     GPIOC
#define DVP_D2_PIN           GPIO_PIN_8
#define DVP_D3_GPIO_PORT     GPIOC
#define DVP_D3_PIN           GPIO_PIN_9
#define DVP_D4_GPIO_PORT     GPIOC
#define DVP_D4_PIN           GPIO_PIN_11
#define DVP_D5_GPIO_PORT     GPIOB
#define DVP_D5_PIN           GPIO_PIN_6
#define DVP_D6_GPIO_PORT     GPIOB
#define DVP_D6_PIN           GPIO_PIN_8
#define DVP_D7_GPIO_PORT     GPIOB
#define DVP_D7_PIN           GPIO_PIN_9
#define DVP_PWDN_GPIO_PORT   GPIOA
#define DVP_PWDN_PIN         GPIO_PIN_11
#define DVP_RESET_GPIO_PORT  GPIOB
#define DVP_RESET_PIN        GPIO_PIN_0
#define DVP_XCLK_GPIO_PORT   GPIOA
#define DVP_XCLK_PIN         GPIO_PIN_8
#define DVP_XCLK_HZ          16000000u  /* MCO1 = HSI, no divider */

#endif /* BOARD_CONFIG_H */
