#include "i2c1.h"

#include "main.h"

// Generous: Renode advances the simulated I2C bus far slower than the
// CPU burns through this counter, so a tight timeout fails spuriously.
#define I2C_TIMEOUT  2000000u

static bool wait_set(volatile uint32_t *reg, uint32_t mask)
{
    uint32_t t = I2C_TIMEOUT;
    while (((*reg) & mask) == 0u)
    {
        if (--t == 0u)
        {
            return false;
        }
    }
    return true;
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

void i2c1_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB6/PB7: alternate function, open drain, pull-up, high speed, AF4.
    GPIOB->MODER &= ~((0x3u << 12) | (0x3u << 14));
    GPIOB->MODER |= ((0x2u << 12) | (0x2u << 14));
    GPIOB->OTYPER |= (1u << 6) | (1u << 7);
    GPIOB->OSPEEDR |= ((0x3u << 12) | (0x3u << 14));
    GPIOB->PUPDR &= ~((0x3u << 12) | (0x3u << 14));
    GPIOB->PUPDR |= ((0x1u << 12) | (0x1u << 14));
    GPIOB->AFR[0] &= ~(0xFFu << 24);
    GPIOB->AFR[0] |= ((0x4u << 24) | (0x4u << 28));

    I2C1->CR1 &= ~I2C_CR1_PE;

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
    I2C1->CR2 = freq_mhz & 0x3Fu;
    // CCR selects SCL from PCLK1. Duty cycle 2 (Tlow = 2*Thigh), which is
    // valid for both standard (CCR >= 50) and fast mode (CCR >= 13).
    I2C1->CCR = pclk1 / (2u * I2C_SCL_HZ);
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
    I2C1->TRISE = (trise > 0x3Fu) ? 0x3Fu : (uint8_t)trise;
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_PE;
}

static bool start_and_select(uint8_t dev_addr, bool read)
{
    I2C1->CR1 |= I2C_CR1_START;
    if (!wait_set(&I2C1->SR1, I2C_SR1_SB))
    {
        return false;
    }
    I2C1->DR = (uint8_t)((dev_addr << 1) | (read ? 1u : 0u));
    if (!wait_set(&I2C1->SR1, I2C_SR1_ADDR))
    {
        return false;
    }
    (void)I2C1->SR2; // clear ADDR
    return true;
}

bool i2c1_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val)
{
    // Renode's I2C controller (and real hardware) forwards queued transmit
    // bytes to the peripheral on STOP. Waiting BTF before STOP stalls, because
    // BTF for the trailing bytes is only produced as part of that flush, so
    // this stages both bytes on TXE and then issues STOP.
    if (!start_and_select(dev_addr, false))
    {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }
    I2C1->DR = reg;
    if (!wait_set(&I2C1->SR1, I2C_SR1_TXE))
    {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }
    I2C1->DR = val;
    if (!wait_set(&I2C1->SR1, I2C_SR1_BTF))
    {
        // Fall through to STOP anyway; the queued bytes still flush.
    }
    I2C1->CR1 |= I2C_CR1_STOP;
    // Wait for the bus to go idle so back-to-back transactions don't overlap.
    uint32_t t = I2C_TIMEOUT;
    while ((I2C1->SR2 & I2C_SR2_BUSY) != 0u)
    {
        if (--t == 0u)
        {
            break;
        }
    }
    return true;
}

bool i2c1_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (buf == NULL || len == 0u)
    {
        return false;
    }
    // Phase 1: send the register address (write, no stop).
    if (!start_and_select(dev_addr, false))
    {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }
    I2C1->DR = reg;
    if (!wait_set(&I2C1->SR1, I2C_SR1_BTF))
    {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }
    // Phase 2: repeated start, then read.
    I2C1->CR1 |= I2C_CR1_START;
    if (!wait_set(&I2C1->SR1, I2C_SR1_SB))
    {
        return false;
    }
    I2C1->DR = (uint8_t)((dev_addr << 1) | 1u);
    if (!wait_set(&I2C1->SR1, I2C_SR1_ADDR))
    {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }
    if (len == 1u)
    {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        (void)I2C1->SR2;
        I2C1->CR1 |= I2C_CR1_STOP;
        if (!wait_set(&I2C1->SR1, I2C_SR1_RXNE))
        {
            I2C1->CR1 |= I2C_CR1_ACK;
            return false;
        }
        buf[0] = (uint8_t)I2C1->DR;
        I2C1->CR1 |= I2C_CR1_ACK;
        return true;
    }
    (void)I2C1->SR2; // ACK stays set for all but the last byte
    for (uint8_t i = 0; i < len; i++)
    {
        if (i == len - 1u)
        {
            I2C1->CR1 &= ~I2C_CR1_ACK;
        }
        if (!wait_set(&I2C1->SR1, I2C_SR1_RXNE))
        {
            I2C1->CR1 |= I2C_CR1_STOP | I2C_CR1_ACK;
            return false;
        }
        buf[i] = (uint8_t)I2C1->DR;
    }
    I2C1->CR1 |= I2C_CR1_STOP | I2C_CR1_ACK;
    return true;
}