#ifndef __W25__H__
#define __W25__H__

#include "cmsis_os2.h"
#include <stdint.h>

// change these defines based on your semaphore and mutex names
#define spiSemaphore flashSPISemaphoreHandle
#define flashMutex FlashMutexHandle
#define FLASH_SPI_HANDLE hspi2

#define W25_DUMMY 0x00  // dummy byte is "don't care", so it can be whatever (from W25X documentation)
#define W25_SR_BUSY_BIT 0x01

#define W25_FREAD 0x0B
#define W25_READ_JEDEC 0x9F
#define W25_WRITE_ENABLE 0x06
#define W25_PAGE_PROGRAM 0x02
#define W25_READ_SR 0x05
#define W25_SECTOR_ERASE 0x20
#define W25_64KB_ERASE 0xD8
#define W25_CHIP_ERASE 0x60

#define W25_PAGE_SIZE 0x100      // 256 B
#define W25_SECTOR_SIZE 0x1000   // 4 KB
#define W25_BLOCK_SIZE 0x10000   // 64 KB
#define W25_LAST_ADDR 0x7FFFF

#define WINBOND_ID 0xEF
#define W25X40_ID 0x3013

#define W25_RECORD_MARKER 0xD7F9

typedef enum
{
    W25_OK,
    W25_UNKNOWN,
    W25_FULL,
    W25_EMPTY,
    W25_SIZE_MISMATCH,
    W25_NOT_FOUND
} W25_Status;

typedef struct
{
    uint8_t mfg_id;
    uint8_t mem_type;
    uint8_t capacity;
    uint32_t first_free_addr;
    uint32_t last_record_addr;
} w25_info_t;

typedef struct __attribute__((packed)) {
    uint16_t marker;
    uint32_t length;
    uint16_t id;
} w25_header_t;

void W25_Select();
void W25_Deselect();
void W25_Transmit(uint8_t *data, uint16_t size);
void W25_Receive(uint8_t *data, uint16_t size);
void W25_Transmit_DMA(uint8_t *data, uint16_t size);
void W25_Receive_DMA(uint8_t *data, uint16_t size);
void W25_WritePage(uint32_t addr, uint8_t *data, uint16_t size);
void W25_SendWriteEnable();

W25_Status W25_Init();
W25_Status W25_ReadLast(uint16_t save_id, uint8_t *data, uint32_t size);
W25_Status W25_Append(uint16_t save_id, uint8_t *data, uint32_t size);
void W25_Read(uint32_t addr, uint8_t *data, uint32_t size);
void W25_Write(uint32_t addr, uint8_t *data, uint32_t size);
void W25_WaitForWriteEnded();
void W25_EraseSector(uint8_t sector_index);
void W25_EraseBlock64KB(uint8_t block_index);
void W25_EraseChip();
W25_Status W25_ReadJEDEC();

#endif
