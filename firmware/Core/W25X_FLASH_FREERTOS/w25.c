#include "w25.h"
#include "spi.h"
#include <string.h>

extern osSemaphoreId_t spiSemaphore;
extern osMutexId_t flashMutex;
w25_info_t flash_info;

void W25_Select()
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
}

void W25_Deselect()
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

void W25_Transmit(uint8_t *data, uint16_t size)
{
    HAL_SPI_Transmit(&FLASH_SPI_HANDLE, data, size, HAL_MAX_DELAY);
}

void W25_Receive(uint8_t *data, uint16_t size)
{
    HAL_SPI_Receive(&FLASH_SPI_HANDLE, data, size, HAL_MAX_DELAY);
}

void W25_Transmit_DMA(uint8_t *data, uint16_t size)
{
    HAL_SPI_Transmit_DMA(&FLASH_SPI_HANDLE, data, size);
}

void W25_Receive_DMA(uint8_t *data, uint16_t size)
{
    HAL_SPI_Receive_DMA(&FLASH_SPI_HANDLE, data, size);
}

// requires WRITE_ENABLE instruction to be sent beforehand. yields while data is being sent
void W25_WritePage(uint32_t addr, uint8_t *data, uint16_t size)
{
    uint8_t buf[4];
    buf[0] = W25_PAGE_PROGRAM;
    buf[1] = (addr >> 16) & 0xFF;
    buf[2] = (addr >> 8) & 0xFF;
    buf[3] = addr & 0xFF;
    W25_Select();
    W25_Transmit(buf, 4);
    
    W25_Transmit_DMA(data, size);
    // semaphore gets released by SPI tx complete callback
    osSemaphoreAcquire(spiSemaphore, osWaitForever);
    
    W25_Deselect();
}

// W25X allows a maximum of a page (256 bytes) to be written at a time. if we have less than 256 bytes,
// we can program them directy. otherwise, we have to write more than one page
void W25_Write(uint32_t addr, uint8_t *data, uint32_t size)
{
    uint32_t inpage_i, written = 0;

    // index inside page to write
    inpage_i = addr & (W25_PAGE_SIZE - 1);

    osMutexAcquire(flashMutex, osWaitForever);

    while ((size - written) > (W25_PAGE_SIZE - inpage_i))
    {
        W25_SendWriteEnable();

        W25_WritePage(addr + written, data + written, W25_PAGE_SIZE - inpage_i);
        written += W25_PAGE_SIZE - inpage_i;

        inpage_i = 0;
        W25_WaitForWriteEnded();
    }

    // write the last page
    if (size - written)
    {
        W25_SendWriteEnable();
        W25_WritePage(addr + written, data + written, size - written);
        W25_WaitForWriteEnded();
    }

    osMutexRelease(flashMutex);
}

void W25_Read(uint32_t addr, uint8_t* data, uint32_t size)
{
    uint8_t buf[5];
    uint16_t to_receive;

    buf[0] = W25_FREAD;
    buf[1] = (addr >> 16) & 0xFF;
    buf[2] = (addr >> 8) & 0xFF;
    buf[3] = addr & 0xFF;
    buf[4] = W25_DUMMY;

    osMutexAcquire(flashMutex, osWaitForever);
    W25_Select();

    W25_Transmit(buf, 5);

    // HAL_SPI_Receive has a 16-bit max receive value, while we accept 32-bit values. so we have to
    // split the reading
    while (size)
    {
        to_receive = ((size > 0xFFFF) ? 0xFFFF : size);
        W25_Receive_DMA(data, to_receive);
        data += to_receive;
        size -= to_receive;
        
        // yield to other tasks while waiting for W25 to finish reading
        // semaphore gets released by SPI rx complete callback
        osSemaphoreAcquire(spiSemaphore, osWaitForever);
    }
    
    W25_Deselect();
    osMutexRelease(flashMutex);
}

// write command has been sent, so wait for the flash internal circuitry to finish writing
void W25_WaitForWriteEnded()
{
    W25_Select();
    uint8_t temp = W25_READ_SR;
    W25_Transmit(&temp, 1);
    temp = 0;
    while (1)
    {
        W25_Receive(&temp, 1);
        // yield to other tasks while waiting
        if (temp & W25_SR_BUSY_BIT)
            osDelay(1);
        else
            break;
    }
    W25_Deselect();
}

// requires the chip to be already selected
void W25_SendWriteEnable()
{
    uint8_t w_enable = W25_WRITE_ENABLE;
    W25_Select();
    W25_Transmit(&w_enable, 1);
    W25_Deselect();
}

void W25_EraseSector(uint8_t sector_index)
{
    uint32_t s_addr = sector_index * W25_SECTOR_SIZE;
    uint8_t buf[4] = {W25_SECTOR_ERASE, (s_addr >> 16) & 0xFF, (s_addr >> 8) & 0xFF, s_addr & 0xFF};
    osMutexAcquire(flashMutex, osWaitForever);
    W25_SendWriteEnable();
    W25_Select();
    W25_Transmit(buf, 4);
    W25_Deselect();
    W25_WaitForWriteEnded();
    osMutexRelease(flashMutex);
}

void W25_EraseBlock64KB(uint8_t block_index)
{
    uint32_t s_addr = block_index * W25_BLOCK_SIZE;
    uint8_t buf[4] = {W25_64KB_ERASE, (s_addr >> 16) & 0xFF, (s_addr >> 8) & 0xFF, s_addr & 0xFF};
    osMutexAcquire(flashMutex, osWaitForever);
    W25_SendWriteEnable();
    W25_Select();
    W25_Transmit(buf, 4);
    W25_Deselect();
    W25_WaitForWriteEnded();
    osMutexRelease(flashMutex);
}

void W25_EraseChip()
{
    uint8_t cmd = W25_CHIP_ERASE;
    osMutexAcquire(flashMutex, osWaitForever);
    W25_SendWriteEnable();
    W25_Select();
    W25_Transmit(&cmd, 1);
    W25_Deselect();
    W25_WaitForWriteEnded();
    flash_info.first_free_addr = 0;
    flash_info.last_record_addr = 0;
    osMutexRelease(flashMutex);
}

// returns whether the device is actually a W25X40
W25_Status W25_ReadJEDEC()
{
    uint8_t buf[4] = {W25_READ_JEDEC, 0, 0, 0};
    osMutexAcquire(flashMutex, osWaitForever);
    W25_Select();
    W25_Transmit(buf, 1);
    W25_Receive(buf + 1, 3);
    W25_Deselect();
    flash_info.mfg_id = buf[1];
    flash_info.mem_type = buf[2];
    flash_info.capacity = buf[3];
    osMutexRelease(flashMutex);
    return (flash_info.mfg_id == WINBOND_ID && ((flash_info.mem_type << 8) | flash_info.capacity) == W25X40_ID) ? W25_OK : W25_UNKNOWN;
}

W25_Status W25_Init()
{
    W25_Status flash_status = W25_ReadJEDEC();
    if (flash_status != W25_OK) return flash_status;

    flash_info.first_free_addr = 0;
    flash_info.last_record_addr = 0; 
    w25_header_t header;

    while (flash_info.first_free_addr <= W25_LAST_ADDR - sizeof(w25_header_t))
    {
        W25_Read(flash_info.first_free_addr, (uint8_t*)&header, sizeof(w25_header_t));

        if (header.marker == W25_RECORD_MARKER)
        {
            flash_info.last_record_addr = flash_info.first_free_addr;
            if (header.length > W25_LAST_ADDR) break;
            // the next free address will be the current address + this data block (header + data + length)
            flash_info.first_free_addr += sizeof(w25_header_t) + header.length + sizeof(header.length); 
        }
        else
            break; 
    }

    return W25_OK;
}

// header (marker, length, id) , data , length
W25_Status W25_Append(uint16_t save_id, uint8_t *data, uint32_t size)
{
    if ((W25_LAST_ADDR - flash_info.first_free_addr + 1) < (sizeof(w25_header_t) + size)) 
        return W25_FULL;

    w25_header_t header = {W25_RECORD_MARKER, size, save_id};
    flash_info.last_record_addr = flash_info.first_free_addr;

    // write header (marker, length, id)
    W25_Write(flash_info.first_free_addr, (uint8_t*)&header, sizeof(w25_header_t));
    flash_info.first_free_addr += sizeof(w25_header_t);
    
    // write data
    W25_Write(flash_info.first_free_addr, data, size);
    flash_info.first_free_addr += size;

    // write length
    W25_Write(flash_info.first_free_addr, (uint8_t*)&(header.length), sizeof(header.length));
    flash_info.first_free_addr += sizeof(header.length);

    return W25_OK;
}

// torna indietro da last_record_addr fino a 0 cercando l'id richiesto
W25_Status W25_ReadLast(uint16_t save_id, uint8_t *data, uint32_t size)
{
    if (flash_info.first_free_addr == 0) return W25_EMPTY;

    w25_header_t temp_header;
    uint32_t addr = flash_info.last_record_addr;
    uint32_t previous_size;

    while (1)
    {
        W25_Read(addr, (uint8_t*)&temp_header, sizeof(w25_header_t));
        if (temp_header.id == save_id)
        {
            // we found the correct data
            if (temp_header.length != size)
                return W25_SIZE_MISMATCH;
            W25_Read(addr + sizeof(w25_header_t), data, size);
            return W25_OK;
        }

        if (addr == 0) return W25_NOT_FOUND;

        // we didn't find the correct id. read the previous data block; just before the current
        // data block, we have the length of the previous data
        if (addr < sizeof(temp_header.length) + sizeof(w25_header_t))
            return W25_NOT_FOUND;
        
        W25_Read(addr - sizeof(temp_header.length), (uint8_t*)&previous_size, sizeof(temp_header.length));
        addr = addr - sizeof(temp_header.length) - previous_size - sizeof(w25_header_t);
    }
}