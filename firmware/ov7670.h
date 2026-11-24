// OmniVision OV7670 camera over SCCB (I2C2, 7-bit address 0x21).
//
// Bring-up subset: PID/VER identification, reset, and a fixed QVGA/YUV422
// configuration matching this project's streaming pipeline. The 0xF0-0xF3
// readout registers are a Renode model test aid only; real silicon delivers
// pixels over the DVP parallel bus (see ov7670_dvp.c).

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