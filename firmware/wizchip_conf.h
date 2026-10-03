#ifndef WIZCHIP_CONF_H
#define WIZCHIP_CONF_H

// WIZnet ioLibrary board configuration for the NCAP STM32F4 + W5500 target.
// SPI1: PA5=SCK, PA6=MISO, PA7=MOSI (AF5). SCSn=PA4, RSTn=PA3.

#include <stdint.h>

#define _WIZCHIP_               5500
#define _WIZCHIP_SOCK_NUM_      8
#define _WIZCHIP_SOCK_MAX_      (_WIZCHIP_SOCK_NUM_ - 1)

#define SOCK_CHUNK_SIZE         2048
#define SPI_CLOCK               42000000

// ioLibrary control callbacks (Socket APIs V3.2.0).
typedef struct
{
    void (*_select)(void);
    void (*_deselect)(void);
} WIZCHIP_CS_T;

typedef struct
{
    void (*_enter)(void);
    void (*_exit)(void);
} WIZCHIP_CRIS_T;

typedef struct
{
    uint8_t (*_read_byte)(void);
    void (*_write_byte)(uint8_t wb);
    void (*_read_burst)(uint8_t *rbuf, uint16_t rlen);
    void (*_write_burst)(uint8_t *wbuf, uint16_t wlen);
} WIZCHIP_SPI_T;

typedef struct
{
    WIZCHIP_SPI_T SPI;
} WIZCHIP_IF_T;

typedef struct
{
    WIZCHIP_IF_T  IF;
    WIZCHIP_CS_T  CS;
    WIZCHIP_CRIS_T CRIS;
} WIZCHIP_T;

extern WIZCHIP_T WIZCHIP;

// Chip register map (Sn_MR_*, getSn_SR(), setSn_MR(), WIZCHIP_READ/WRITE...).
#include "w5500.h"

#endif /* WIZCHIP_CONF_H */
