// WIZnet ioLibrary port layer for the NCAP STM32F4 + W5500 target.
// SPI2: PB13=SCK, PB14=MISO, PB15=MOSI (AF5). SCSn=PB12, RSTn=PB11.
//
// The W5500 moved from SPI1 to SPI2: the STM32F446 hardwires DCMI_VSYNC to
// PA4 and DCMI_PIXCLK to PA6 with no remap option, and those were SPI1_MISO
// and SPI1_SCSn. Camera capture and the Ethernet PHY cannot share those pins.
//
// The current ioLibrary master no longer ships wizchip_init()/wizchip_setnetinfo(),
// so the device is configured here by writing the common registers directly.

#include "wizchip_conf.h"

#include "main.h"

#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi2;

// Common register addresses (BSB = 0), same encoding as the WIZCHIP_WRITE macro.
#define W5500_REG_MR        0x0000
#define W5500_REG_GAR       0x0001
#define W5500_REG_SUBR      0x0005
#define W5500_REG_SHAR      0x0009
#define W5500_REG_SIPR      0x000F
#define W5500_REG_RTR       0x0019
#define W5500_REG_RCR       0x001B
#define W5500_REG_PHYCFGR   0x002E
#define W5500_REG_VERSIONR  0x0039

#define W5500_BSB_COMMON    0x00
#define W5500_ADDR(off)     (((uint32_t)(off) << 8) | ((W5500_BSB_COMMON) << 3))

static uint8_t w5500_spi_read_byte(void)
{
    uint8_t tx = 0xFF, rx = 0;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 100);
    return rx;
}

static void w5500_spi_write_byte(uint8_t wb)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi2, &wb, &rx, 1, 100);
}

static void w5500_spi_read_burst(uint8_t *rbuf, uint16_t rlen)
{
    // The stack dummy buffer is small on purpose; loop so arbitrarily large
    // reads are honoured. Dropping (the old `if (rlen > 256) return;`) meant
    // large recvfrom() payloads silently came back as zeros.
    uint8_t dummy[64];
    memset(dummy, 0xFF, sizeof(dummy));
    uint16_t done = 0;
    while (done < rlen)
    {
        uint16_t chunk = (uint16_t)(rlen - done);
        if (chunk > sizeof(dummy))
        {
            chunk = (uint16_t)sizeof(dummy);
        }
        HAL_SPI_TransmitReceive(&hspi2, dummy, &rbuf[done], chunk, 100);
        done += chunk;
    }
}

static void w5500_spi_write_burst(uint8_t *wbuf, uint16_t wlen)
{
    // Same chunking as the read side. CS stays low across the whole burst
    // because the caller asserts it for the full WIZCHIP_WRITE_BUF call, so
    // splitting here does not break framing.
    uint8_t rx_sink[64];
    uint16_t done = 0;
    while (done < wlen)
    {
        uint16_t chunk = (uint16_t)(wlen - done);
        if (chunk > sizeof(rx_sink))
        {
            chunk = (uint16_t)sizeof(rx_sink);
        }
        HAL_SPI_TransmitReceive(&hspi2, wbuf + done, rx_sink, chunk, 100);
        done += chunk;
    }
}

static void w5500_cs_low(void)  { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); }
static void w5500_cs_high(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  }

// Bare-metal build: no RTOS, so the critical section only needs to keep SPI
// byte sequences contiguous, which the CS line already guarantees.
static void w5500_cris_enter(void) { }
static void w5500_cris_exit(void)  { }

WIZCHIP_T WIZCHIP;

static void w5500_bind_callbacks(void)
{
    WIZCHIP.IF.SPI._read_byte   = w5500_spi_read_byte;
    WIZCHIP.IF.SPI._write_byte  = w5500_spi_write_byte;
    WIZCHIP.IF.SPI._read_burst  = w5500_spi_read_burst;
    WIZCHIP.IF.SPI._write_burst = w5500_spi_write_burst;
    WIZCHIP.CS._select          = w5500_cs_low;
    WIZCHIP.CS._deselect        = w5500_cs_high;
    WIZCHIP.CRIS._enter         = w5500_cris_enter;
    WIZCHIP.CRIS._exit          = w5500_cris_exit;
}

void wizchip_cs_low(void)  { w5500_cs_low(); }
void wizchip_cs_high(void) { w5500_cs_high(); }

void wizchip_reset_low(void)  { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); }
void wizchip_reset_high(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); }

// SCSn/RSTn are ordinary push-pull outputs on PB12/PB11.
static void w5500_control_gpio_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_12 | GPIO_PIN_11;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
}

static uint8_t w5500_read_common(uint16_t off)
{
    // Control byte = (BSB << 3) | (RWB << 2) | OM. RWB=0 -> read, OM=00 -> VDM.
    uint32_t addr = W5500_ADDR(off);
    WIZCHIP.CS._select();
    w5500_spi_write_byte((uint8_t)((addr >> 16) & 0xFF));
    w5500_spi_write_byte((uint8_t)((addr >> 8) & 0xFF));
    w5500_spi_write_byte((uint8_t)(addr & 0xFF));
    uint8_t v = w5500_spi_read_byte();
    WIZCHIP.CS._deselect();
    return v;
}

static void w5500_write_common(uint16_t off, uint8_t val)
{
    uint32_t addr = W5500_ADDR(off);
    WIZCHIP.CS._select();
    w5500_spi_write_byte((uint8_t)((addr >> 16) & 0xFF));
    w5500_spi_write_byte((uint8_t)((addr >> 8) & 0xFF));
    w5500_spi_write_byte((uint8_t)((addr & 0xFF) | 0x04)); // RWB=1 -> write
    w5500_spi_write_byte(val);
    WIZCHIP.CS._deselect();
}

uint8_t w5500_version(void)
{
    return w5500_read_common(W5500_REG_VERSIONR);
}

void w5500_hw_init(const uint8_t mac[6], const uint8_t ip[4],
                   const uint8_t mask[4], const uint8_t gw[4])
{
    w5500_control_gpio_init();
    w5500_bind_callbacks();

    w5500_cs_high();
    wizchip_reset_high();
    HAL_Delay(10);

    wizchip_reset_low();
    HAL_Delay(10);
    wizchip_reset_high();
    HAL_Delay(10);

    w5500_write_common(W5500_REG_MR, 0x80);      // software reset
    HAL_Delay(10);
    w5500_write_common(W5500_REG_MR, 0x00);

    w5500_write_common(W5500_REG_RTR, 0x07);      // 2000 x 100us
    w5500_write_common(W5500_REG_RCR, 0x08);      // retry count 8

    for (int i = 0; i < 4; i++)
    {
        w5500_write_common(W5500_REG_GAR + i, gw[i]);
        w5500_write_common(W5500_REG_SUBR + i, mask[i]);
        w5500_write_common(W5500_REG_SIPR + i, ip[i]);
    }
    for (int i = 0; i < 6; i++)
    {
        w5500_write_common(W5500_REG_SHAR + i, mac[i]);
    }
}
