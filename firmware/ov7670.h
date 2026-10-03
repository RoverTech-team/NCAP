// OmniVision OV7670 camera over SCCB (I2C1, 7-bit address 0x21).
//
// Bring-up subset: PID/VER identification, reset, and a fixed QVGA/YUV422
// configuration matching this project's streaming pipeline. Pixel data is
// fetched through the model's test readout registers (0xF0-0xF3); see
// renode_configs/peripherals/OV7670.cs for why this stands in for the
// parallel bus. The returned bytes are the frame's luma (Y of YUYV).

#ifndef OV7670_H
#define OV7670_H

#include <stdbool.h>
#include <stdint.h>

#define OV7670_ADDR   0x21u

bool ov7670_init(void);

// Fetch `count` luma rows starting at `start_row` into `buf` (width bytes
// per row). Returns false on any transfer error.
bool ov7670_read_luma_rows(uint16_t start_row, uint8_t count,
                            uint8_t *buf, uint16_t width);

#endif