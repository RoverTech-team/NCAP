// Minimal polled I2C master for STM32F4, wired to I2C2 on the NUCLEO-F446RE
// (SCL PB10 / SDA PB3). See board_config.h.
//
// The tree has no HAL I2C driver, and only a master that writes a register
// address then reads back bytes is needed for sensor bring-up. This uses the
// F1-compatible register set (CR1/CR2/DR/SR1/SR2/CCR/TRISE), so it works
// against Renode's STM32F1_I2C model as well as real F4 silicon. PCLK1 is
// derived from RCC at runtime, so no clock-tree assumptions are baked in.
// Every wait is bounded; all functions return false on timeout.

#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdbool.h>
#include <stdint.h>

void i2c_bus_init(void);

// Reset and reconfigure the I2C peripheral (recovers a stuck BUSY/AF state).
void i2c_bus_recover(void);

// Address-only probe: true if the device ACKs its 7-bit address.
bool i2c_bus_probe(uint8_t dev_addr);

// Write one register.
bool i2c_bus_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val);

// Read `len` bytes starting at `reg` (repeated start, auto-increment safe).
bool i2c_bus_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len);

#endif
