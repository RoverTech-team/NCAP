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
    // Standard mode 100 kHz: CCR = PCLK1 / (2 * 100000).
    I2C1->CCR = pclk1 / (2u * 100000u);
    I2C1->TRISE = freq_mhz + 1u;
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