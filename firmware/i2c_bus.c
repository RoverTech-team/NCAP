#include "i2c_bus.h"

#include "board_config.h"
#include "main.h"

// Generous: Renode advances the simulated I2C bus far slower than the
// CPU burns through this counter, so a tight timeout fails spuriously.
#define I2C_TIMEOUT  2000000u

static bool wait_set_timeout(volatile uint32_t *reg, uint32_t mask, uint32_t timeout)
{
    uint32_t t = timeout;
    while (((*reg) & mask) == 0u)
    {
        if (--t == 0u)
        {
            return false;
        }
    }
    return true;
}

static bool wait_set(volatile uint32_t *reg, uint32_t mask)
{
    return wait_set_timeout(reg, mask, I2C_TIMEOUT);
}

static void wait_bus_idle(void)
{
    uint32_t t = I2C_TIMEOUT;
    while ((NCAP_I2C->SR2 & I2C_SR2_BUSY) != 0u)
    {
        if (--t == 0u)
        {
            break;
        }
    }
}

static uint32_t pclk1_hz(void)
{
    static const uint8_t div[8] = {1, 1, 1, 1, 2, 4, 8, 16};
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7u;
    return SystemCoreClock / div[ppre1];
}

// SCL target for SCCB/IMU traffic. 100 kHz standard mode was pinned here;
// 400 kHz fast mode is the highest speed that is reliably in spec for both
// the OV7670 and the LSM9DS1 sharing this bus.
//
// Note this does NOT make the camera path fast enough for 24 fps: a QVGA
// luma frame is 76,800 bytes = 691,200 SCL pulses, which is 1.73 s/frame at
// 400 kHz (0.58 fps). The F446 I2C peripheral tops out near 1 MHz, which
// still only reaches ~1.45 fps. Reaching tens of fps needs the OV7670's
// parallel DVP output, not a faster SCCB clock.
#ifndef I2C_SCL_HZ
#define I2C_SCL_HZ  400000u
#endif

static void i2c_hw_config(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    NCAP_I2C_CLK_ENABLE();

    // I2C2 on PB10 (SCL) / PB3 (SDA): alternate function, open drain,
    // internal pull-up for bench bring-up. External 2.2-4.7 kOhm pull-ups
    // are still required for a real bus.
    GPIO_InitTypeDef g = {0};
    g.Pin = NCAP_I2C_SCL_PIN | NCAP_I2C_SDA_PIN;
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = NCAP_I2C_GPIO_AF;
    HAL_GPIO_Init(NCAP_I2C_GPIO_PORT, &g);

    NCAP_I2C->CR1 &= ~I2C_CR1_PE;

    uint32_t pclk1 = pclk1_hz();
    uint32_t freq_mhz = pclk1 / 1000000u;
    if (freq_mhz < 2u)
    {
        freq_mhz = 2u;
    }
    if (freq_mhz > 50u)
    {
        freq_mhz = 50u;
    }
    NCAP_I2C->CR2 = freq_mhz & 0x3Fu;
    // CCR selects SCL from PCLK1. Duty cycle 2 (Tlow = 2*Thigh), which is
    // valid for both standard (CCR >= 50) and fast mode (CCR >= 13).
    NCAP_I2C->CCR = pclk1 / (2u * I2C_SCL_HZ);
    // TRISE is the max rise-time allowance in PCLK1 cycles. Standard mode
    // uses 300ns * f_PCLK(MHz) + 1; fast mode uses the RM0090 expression
    // 300ns / (1/SCL - 300ns) + 1, floored at 1.
    uint32_t trise = (300u * freq_mhz) / 1000u + 1u;
    if (I2C_SCL_HZ > 100000u)
    {
        uint32_t scl_ns = 1000000000u / I2C_SCL_HZ;
        uint32_t tcyc_ns = 1000000000u / (freq_mhz * 1000000u);
        uint32_t fast = (scl_ns > 300u) ? (300u * tcyc_ns / (scl_ns - 300u) + 1u) : 1u;
        if (fast < trise) trise = fast;
    }
    NCAP_I2C->TRISE = (trise > 0x3Fu) ? 0x3Fu : (uint8_t)trise;
    NCAP_I2C->CR1 |= I2C_CR1_ACK;
    NCAP_I2C->CR1 |= I2C_CR1_PE;
}

void i2c_bus_init(void)
{
    i2c_hw_config();
}

void i2c_bus_recover(void)
{
    NCAP_I2C->CR1 |= I2C_CR1_SWRST;
    NCAP_I2C->CR1 &= ~I2C_CR1_SWRST;
    i2c_hw_config();
}

bool i2c_bus_probe(uint8_t dev_addr)
{
    NCAP_I2C->SR1 = 0;   // clear AF/BERR/ARLO left by an earlier attempt
    NCAP_I2C->CR1 |= I2C_CR1_START;
    if (!wait_set_timeout(&NCAP_I2C->SR1, I2C_SR1_SB, 20000u))
    {
        NCAP_I2C->CR1 |= I2C_CR1_STOP;
        return false;
    }
    NCAP_I2C->DR = (uint8_t)(dev_addr << 1);
    if (wait_set_timeout(&NCAP_I2C->SR1, I2C_SR1_ADDR, 20000u))
    {
        (void)NCAP_I2C->SR2;
        NCAP_I2C->CR1 |= I2C_CR1_STOP;
        return true;
    }
    NCAP_I2C->SR1 = 0;
    NCAP_I2C->CR1 |= I2C_CR1_STOP;
    return false;
}

static bool start_and_select(uint8_t dev_addr, bool read)
{
    // SCCB needs a bus-free time after the previous STOP before a new START.
    // The F4 clears SR2.BUSY in well under a microsecond, which is too soon
    // for the OV7670 at 400 kHz, so give it a millisecond.
    HAL_Delay(1);
    NCAP_I2C->CR1 |= I2C_CR1_START;
    if (!wait_set(&NCAP_I2C->SR1, I2C_SR1_SB))
    {
        return false;
    }
    NCAP_I2C->DR = (uint8_t)((dev_addr << 1) | (read ? 1u : 0u));
    if (!wait_set(&NCAP_I2C->SR1, I2C_SR1_ADDR))
    {
        return false;
    }
    (void)NCAP_I2C->SR2; // clear ADDR
    return true;
}

bool i2c_bus_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val)
{
    // Renode's I2C controller (and real hardware) forwards queued transmit
    // bytes to the peripheral on STOP. Waiting BTF before STOP stalls, because
    // BTF for the trailing bytes is only produced as part of that flush, so
    // this stages both bytes on TXE and then issues STOP.
    if (!start_and_select(dev_addr, false))
    {
        NCAP_I2C->CR1 |= I2C_CR1_STOP;
        return false;
    }
    NCAP_I2C->DR = reg;
    if (!wait_set(&NCAP_I2C->SR1, I2C_SR1_TXE))
    {
        NCAP_I2C->CR1 |= I2C_CR1_STOP;
        return false;
    }
    NCAP_I2C->DR = val;
    if (!wait_set(&NCAP_I2C->SR1, I2C_SR1_BTF))
    {
        // Fall through to STOP anyway; the queued bytes still flush.
    }
    NCAP_I2C->CR1 |= I2C_CR1_STOP;
    // Wait for the bus to go idle so back-to-back transactions don't overlap.
    wait_bus_idle();
    return true;
}

bool i2c_bus_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (buf == NULL || len == 0u)
    {
        return false;
    }
    // Phase 1: send the register address (write, no stop).
    if (!start_and_select(dev_addr, false))
    {
        NCAP_I2C->CR1 |= I2C_CR1_STOP;
        return false;
    }
    NCAP_I2C->DR = reg;
    if (!wait_set(&NCAP_I2C->SR1, I2C_SR1_BTF))
    {
        NCAP_I2C->CR1 |= I2C_CR1_STOP;
        return false;
    }
    // Phase 2: SCCB does not implement repeated START. Close the write phase
    // with STOP, wait for idle, then start a new transfer for the read.
    NCAP_I2C->CR1 |= I2C_CR1_STOP;
    wait_bus_idle();
    NCAP_I2C->CR1 |= I2C_CR1_START;
    if (!wait_set(&NCAP_I2C->SR1, I2C_SR1_SB))
    {
        return false;
    }
    NCAP_I2C->DR = (uint8_t)((dev_addr << 1) | 1u);
    if (!wait_set(&NCAP_I2C->SR1, I2C_SR1_ADDR))
    {
        NCAP_I2C->CR1 |= I2C_CR1_STOP;
        return false;
    }
    if (len == 1u)
    {
        NCAP_I2C->CR1 &= ~I2C_CR1_ACK;
        (void)NCAP_I2C->SR2;
        NCAP_I2C->CR1 |= I2C_CR1_STOP;
        if (!wait_set(&NCAP_I2C->SR1, I2C_SR1_RXNE))
        {
            NCAP_I2C->CR1 |= I2C_CR1_ACK;
            return false;
        }
        buf[0] = (uint8_t)NCAP_I2C->DR;
        NCAP_I2C->CR1 |= I2C_CR1_ACK;
        return true;
    }
    (void)NCAP_I2C->SR2; // ACK stays set for all but the last byte
    for (uint8_t i = 0; i < len; i++)
    {
        if (i == len - 1u)
        {
            NCAP_I2C->CR1 &= ~I2C_CR1_ACK;
        }
        if (!wait_set(&NCAP_I2C->SR1, I2C_SR1_RXNE))
        {
            NCAP_I2C->CR1 |= I2C_CR1_STOP | I2C_CR1_ACK;
            return false;
        }
        buf[i] = (uint8_t)NCAP_I2C->DR;
    }
    NCAP_I2C->CR1 |= I2C_CR1_STOP | I2C_CR1_ACK;
    return true;
}