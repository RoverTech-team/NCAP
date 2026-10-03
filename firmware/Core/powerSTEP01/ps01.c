#include "ps01.h"
#include "cfg_structs.h"
#include "cmsis_os2.h"
#include "ps01calc.h"
#include "spi.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_spi.h"
#include <string.h>

extern osMutexId_t ps01SPIMutex;
extern osSemaphoreId_t ps01SPISemaphore;
StepperBank_t *mot_bank;
uint8_t MOT_NUMBER;

void ps01SetBank(StepperBank_t *bank, uint8_t n_motors)
{
    mot_bank = bank;
    MOT_NUMBER = n_motors;
}

void _writebyte_chain(uint8_t byte)
{
    uint8_t tx[MOT_NUMBER];
    memset(tx, 0, MOT_NUMBER);
    tx[MOT_NUMBER - 1 - mot_bank->active] = byte;

    osMutexAcquire(ps01SPIMutex, osWaitForever);
    
    HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_RESET);
    
    HAL_SPI_Transmit_DMA(&hspi1, tx, MOT_NUMBER);
    osSemaphoreAcquire(ps01SPISemaphore, osWaitForever); // wait until DMA is done sending

    while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY)) { osDelay(1); } // wait until SPI is done, for safety
    
    HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_SET);
    
    osMutexRelease(ps01SPIMutex);
    
    osDelay(1);
}

uint8_t _readbyte_chain(void)
{
    uint8_t tx[MOT_NUMBER];
    uint8_t rx[MOT_NUMBER];
    memset(tx, 0, MOT_NUMBER);
    memset(rx, 0, MOT_NUMBER);

    osMutexAcquire(ps01SPIMutex, osWaitForever);
    
    HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_RESET);
    
    HAL_SPI_TransmitReceive_DMA(&PS01_SPI_HANDLE, tx, rx, MOT_NUMBER);
    osSemaphoreAcquire(ps01SPISemaphore, osWaitForever); // wait until DMA is done sending

    while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY)) { osDelay(1); } // wait until SPI is done, for safety

    HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_SET);
    
    osMutexRelease(ps01SPIMutex);
    
    osDelay(1);
    
    return rx[MOT_NUMBER - 1 - mot_bank->active];
}

void _xferbits_chain(uint32_t value, uint8_t bitlen)
{
    uint8_t nbytes = bitlen / 8;
    if (bitlen % 8 != 0) nbytes++;

    for (uint8_t i = 0; i < nbytes; i++)
        _writebyte_chain((uint8_t)(value >> ((nbytes - i - 1) * 8)));
}

uint32_t _rxbits_chain(uint8_t bitlen)
{
    uint32_t retval = 0;
    uint8_t nbytes = bitlen / 8;
    if (bitlen % 8 != 0) nbytes++;

    for (uint8_t i = 0; i < nbytes; i++)
    {
        retval <<= 8;
        retval |= _readbyte_chain();
    }

    return retval;
}

uint16_t ps01GetStatus_chain()
{
    _writebyte_chain(0xD0);
    return _rxbits_chain(16);
}

void ps01SetParam_chain(uint8_t param, uint32_t value)
{
    _writebyte_chain(param);

    switch (param)
    {
        case ABS_POS:
        _xferbits_chain(value, 22);
        break;
        case EL_POS:
        _xferbits_chain(value, 9);
        break;
        case MARK:
        _xferbits_chain(value, 22);
        break;
        case ACC:
        _xferbits_chain(value, 12);
        break;
        case DEC:
        _xferbits_chain(value, 12);
        break;
        case MAX_SPEED:
        _xferbits_chain(value, 10);
        break;
        case MIN_SPEED:
        _xferbits_chain(value, 12);
        break;
        case OCD_TH:
        _xferbits_chain(value, 5);
        break;
        case FS_SPD:
        _xferbits_chain(value, 11);
        break;
        case STEP_MODE:
        _xferbits_chain(value, 8);
        break;
        case ALARM_EN:
        _xferbits_chain(value, 8);
        break;
        case GATECFG1:
        _xferbits_chain(value, 11);
        break;
        case GATECFG2:
        _xferbits_chain(value, 8);
        break;
        case CONFIG:
        _xferbits_chain(value, 16);
        break;
        case KVAL_HOLD:
        _xferbits_chain(value, 8);
        break;
        case KVAL_RUN:
        _xferbits_chain(value, 8);
        break;
        case KVAL_ACC:
        _xferbits_chain(value, 8);
        break;
        case KVAL_DEC:
        _xferbits_chain(value, 8);
        break;
        case INT_SPEED:
        _xferbits_chain(value, 14);
        break;
        case ST_SLP:
        _xferbits_chain(value, 8);
        break;
        case FN_SLP_ACC:
        _xferbits_chain(value, 8);
        break;
        case FN_SLP_DEC:
        _xferbits_chain(value, 8);
        break;
        case K_THERM:
        _xferbits_chain(value, 4);
        break;
        case STALL_TH:
        _xferbits_chain(value, 5);
        break;
    }
}

uint32_t ps01GetParam_chain(uint8_t param)
{
    _writebyte_chain(0x20 | param);
    switch (param)
    {
        case ABS_POS:
        return _rxbits_chain(22);
        case EL_POS:
        return _rxbits_chain(9);
        case MARK:
        return _rxbits_chain(22);
        case SPEED:
        return _rxbits_chain(20);
        case ACC:
        return _rxbits_chain(12);
        case DEC:
        return _rxbits_chain(12);
        case MAX_SPEED:
        return _rxbits_chain(10);
        case MIN_SPEED:
        return _rxbits_chain(12);
        case ADC_OUT:
        return _rxbits_chain(5);
        case OCD_TH:
        return _rxbits_chain(5);
        case FS_SPD:
        return _rxbits_chain(11);
        case STEP_MODE:
        return _rxbits_chain(8);
        case ALARM_EN:
        return _rxbits_chain(8);
        case GATECFG1:
        return _rxbits_chain(11);
        case GATECFG2:
        return _rxbits_chain(8);
        case CONFIG:
        return _rxbits_chain(16);
        case KVAL_HOLD:
        return _rxbits_chain(8);
        case KVAL_RUN:
        return _rxbits_chain(8);
        case KVAL_ACC:
        return _rxbits_chain(8);
        case KVAL_DEC:
        return _rxbits_chain(8);
        case INT_SPEED:
        return _rxbits_chain(14);
        case ST_SLP:
        return _rxbits_chain(8);
        case FN_SLP_ACC:
        return _rxbits_chain(8);
        case FN_SLP_DEC:
        return _rxbits_chain(8);
        case K_THERM:
        return _rxbits_chain(4);
        case STALL_TH:
        return _rxbits_chain(5);
        default:
        return 0;
    }
}

void ps01SetConfig_chain()
{
    ps01SetParam_chain(CONFIG, mot_bank->motors[mot_bank->active].config.reg);
}

void ps01SetStepMode_chain()
{
    ps01SetParam_chain(STEP_MODE, mot_bank->motors[mot_bank->active].stepmode.reg);
}

void ps01SetKVALs_chain()
{
    ps01SetParam_chain(KVAL_ACC,  mot_bank->motors[mot_bank->active].kvals.acceleration);
    ps01SetParam_chain(KVAL_DEC,  mot_bank->motors[mot_bank->active].kvals.deceleration);
    ps01SetParam_chain(KVAL_RUN,  mot_bank->motors[mot_bank->active].kvals.run);
    ps01SetParam_chain(KVAL_HOLD, mot_bank->motors[mot_bank->active].kvals.hold);
}

void ps01SetMaxSpeed_chain(uint16_t steps_s)
{
    ps01SetParam_chain(MAX_SPEED, calculateMaxMinSpeed(steps_s));
}

void ps01SetMinSpeed_chain(uint16_t steps_s)
{
    ps01SetParam_chain(MIN_SPEED, calculateMaxMinSpeed(steps_s));
}

void ps01SetAcceleration_chain(uint16_t steps_s2)
{
    ps01SetParam_chain(ACC, calculateAcceleration(steps_s2));
}

void ps01SetDeceleration_chain(uint16_t steps_s2)
{
    ps01SetParam_chain(DEC, calculateAcceleration(steps_s2));
}

void ps01SetFullStepSpeed_chain(uint16_t steps_s)
{
    ps01SetParam_chain(FS_SPD, calculateMaxMinSpeed(steps_s));
}

// OCD_TH and STALL_TH functions calculate the register value based on:
// Rds(ON) of the internal MOSFET at 125° (23mΩ)
// Vth = trip_amps * 0.023f
// reg_val = Vth / 0.03125f (0.03125 is the register "step")
// 
// so if trip_amps is 2A, reg_val will be 1 which is 62.5mA.
// this means that it will trigger at I = V/R = 0.0625 / 0.023 =~ 2.7A
// this is all based on Rds(ON) when the driver is HOT (125°C), so when at
// ambient temperature, the trip current will be HIGHER
void ps01SetOcThreshold_chain(float trip_amps)
{
    float v_threshold = trip_amps * PS01_RDS_ON_HOT_OHM;
    uint8_t reg_val = v_threshold / PS01_OCD_STEP_V;

    if (reg_val < PS01_OCD_REG_MIN) reg_val = PS01_OCD_REG_MIN;
    if (reg_val > PS01_OCD_REG_MAX) reg_val = PS01_OCD_REG_MAX;

    ps01SetParam_chain(OCD_TH, reg_val);
}

// OCD_TH and STALL_TH functions calculate the register value based on the
// Rds(ON) of the internal MOSFET
// Vth = trip_amps * 0.0195f
// reg_val = Vth / 0.03125f (0.03125 is the register "step")
// 
// so if trip_amps is 2A, reg_val will be 1 which is 62.5mA.
// this means that it will trigger at I = V/R = 0.0625 / 0.023 =~ 3.2A
// this is all based on a mean value of Rds(ON), between 125°C and 25°C (75°C)
void ps01SetStallThreshold_chain(float trip_amps)
{
    float v_threshold = trip_amps * PS01_RDS_ON_HOT_OHM;
    uint8_t reg_val= v_threshold / PS01_OCD_STEP_V;

    if (reg_val < PS01_OCD_REG_MIN) reg_val = PS01_OCD_REG_MIN;
    if (reg_val > PS01_OCD_REG_MAX) reg_val = PS01_OCD_REG_MAX;

    ps01SetParam_chain(STALL_TH, reg_val);
}

void ps01SetAlarms_chain(uint8_t alarm_bits)
{
    ps01SetParam_chain(ALARM_EN, alarm_bits);
}

void ps01WaitBusy_chain()
{
    uint16_t status;
    do {
        osDelay(10);
        status = ps01GetParam_chain(STATUS);
    } while (!(status & 0x0002));  // BUSY bit (bit 1) goes high when done
}

int32_t ps01GetPosition_chain()
{
    // upper bits are not dirty (_rxbits_chain sets the return value to 0 beforehand)
    int32_t raw = (int32_t)ps01GetParam_chain(ABS_POS);
    // extend the SIGNED number from 22 bits (2's complement) to 32 bits
    // if the number is negative, right shifting will fill the bits from
    // the left with 1 (otherwise 0)
    return (raw << 10) >> 10;
}

void ps01Run_chain(uint8_t dir, uint16_t steps_s)
{
    uint32_t spd = calculateSpeed(steps_s);
    _writebyte_chain(RUN_CMD | (dir & 0x01));
    _writebyte_chain((uint8_t)(spd >> 16));
    _writebyte_chain((uint8_t)(spd >> 8));
    _writebyte_chain((uint8_t)spd);
}

void ps01StepClock_chain(uint8_t dir)
{
    _writebyte_chain(STEPCLK_CMD | (dir & 0x01));
}

void ps01Move_chain(uint8_t dir, uint32_t n_steps)
{
    _writebyte_chain(MOVE_CMD | (dir & 0x01));
    _writebyte_chain((uint8_t)(n_steps >> 16));
    _writebyte_chain((uint8_t)(n_steps >> 8));
    _writebyte_chain((uint8_t)n_steps);
}

void ps01MoveDegrees_chain(uint8_t dir, uint16_t deg)
{
    ps01Move_chain(dir, getStepsFromAngle(deg, &mot_bank->motors[mot_bank->active]));
}

void ps01GoTo_chain(int32_t abs_pos)
{
    _writebyte_chain(GOTO_CMD);
    _writebyte_chain((uint8_t)(abs_pos >> 16));
    _writebyte_chain((uint8_t)(abs_pos >> 8));
    _writebyte_chain((uint8_t)abs_pos);
}

void ps01GoToDegrees_chain(int32_t abs_deg)
{
    int32_t pos = getStepsFromAngle(abs_deg, &mot_bank->motors[mot_bank->active]);
    ps01GoTo_chain(pos);
}

void ps01GoTo_DIR_chain(uint8_t dir, int32_t abs_pos)
{
    _writebyte_chain(GOTO_DIR_CMD | (dir & 0x01));
    _writebyte_chain((uint8_t)(abs_pos >> 16));
    _writebyte_chain((uint8_t)(abs_pos >> 8));
    _writebyte_chain((uint8_t)abs_pos);
}

void ps01GoToDegrees_DIR_chain(uint8_t dir, int32_t abs_deg)
{
    int32_t pos = getStepsFromAngle(abs_deg, &mot_bank->motors[mot_bank->active]);
    ps01GoTo_DIR_chain(dir, pos);
}

void ps01GoUntil_chain(uint8_t act, uint8_t dir, uint16_t steps_s)
{
    uint32_t spd = calculateSpeed(steps_s);
    _writebyte_chain(GOUNTIL_CMD | ((act & 0x01) << 3) | (dir & 0x01));
    _writebyte_chain((uint8_t)(spd >> 16));
    _writebyte_chain((uint8_t)(spd >> 8));
    _writebyte_chain((uint8_t)spd);
}

void ps01ReleaseSW_chain(uint8_t act, uint8_t dir)
{
    _writebyte_chain(RELEASESW_CMD | ((act & 0x01) << 3) | (dir & 0x01));
}

void ps01GoHome_chain()
{
    _writebyte_chain(GOHOME_CMD);
}

void ps01GoMark_chain()
{
    _writebyte_chain(GOMARK_CMD);
}

void ps01ResetPos_chain()
{
    _writebyte_chain(RESETPOS_CMD);
}

void ps01ResetDevice_chain()
{
    _writebyte_chain(RESETDEV_CMD);
}

void ps01SoftStop_chain()
{
    _writebyte_chain(SOFTSTOP_CMD);
}

void ps01HardStop_chain()
{
    _writebyte_chain(HARDSTOP_CMD);
}

void ps01SoftHiZ_chain()
{
    _writebyte_chain(SOFTHIZ_CMD);
}

void ps01HardHiZ_chain()
{
    _writebyte_chain(HARDHIZ_CMD);
}