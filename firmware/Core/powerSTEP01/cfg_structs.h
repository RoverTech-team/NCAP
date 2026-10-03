#ifndef __CFG_H__
#define __CFG_H__

#define ps01SPIMutex MotorDriverMutexHandle
#define ps01SPISemaphore driverSPISemaphoreHandle
#define PS01_SPI_HANDLE hspi1

#define PS01_RDS_ON_HOT_OHM     0.0195f  // 16mΩ at 25°C, 23mΩ at 125°C
#define PS01_OCD_STEP_V         0.03125f // 31.25mV per step, table 25
#define PS01_OCD_REG_MIN        0
#define PS01_OCD_REG_MAX        31

typedef union
{
    uint16_t reg;
    struct
    {
        uint16_t OSC_SEL : 3;
        uint16_t EXT_CLK : 1;
        uint16_t SW_MODE : 1;
        uint16_t EN_VSCOMP : 1;
        uint16_t : 1;   // unused bit
        uint16_t OC_SD : 1;
        uint16_t UVLOVAL : 1;
        uint16_t VCCVAL : 1;
        uint16_t F_PWM_DEC : 3;
        uint16_t F_PWM_INT : 3;
    } bits;
} PS01ConfigVoltageMode_t;

typedef union
{
    uint16_t reg;
    struct
    {
        uint16_t STEP_SEL : 3;  // 0 = full, 1 = half, 2 = 1/4, ... , 7 = 1/128
        uint16_t CM_VM : 1;     // 0 = voltage mode, 1 = current mode
        uint16_t SYNC_SEL : 3;  // 0 = full, 1 = half, 2 = 1/4, ... , 7 = 1/128
        uint16_t SYNC_EN : 1;   // 0 = disable, 1 = enable
    } bits;
} PS01StepMode_t;

typedef struct
{
    uint8_t acceleration;
    uint8_t deceleration;
    uint8_t hold;
    uint8_t run;
} PS01KVALs_t;

typedef union
{
    uint16_t reg;
    struct
    {
        uint16_t HIZ : 1;
        uint16_t BUSY : 1;
        uint16_t SW_F : 1;
        uint16_t SW_EVN : 1;
        uint16_t DIR : 1;
        uint16_t MOT_STATUS : 2;
        uint16_t CMD_ERROR : 1;
        uint16_t STCK_MOD : 1;
        uint16_t UVLO : 1;
        uint16_t UVLO_ADC : 1;
        uint16_t TH_STATUS : 2;
        uint16_t OCD : 1;
        uint16_t STALL_B : 1;
        uint16_t STALL_A : 1;
    } bits;
} PS01Status_t;

typedef struct
{
    uint32_t steps_rev;
    float reduction_ratio;
    int32_t min_degs;
    int32_t max_degs;
    PS01ConfigVoltageMode_t config;
    PS01StepMode_t stepmode;
    PS01KVALs_t kvals;
} Stepper_t;

typedef struct
{
    Stepper_t *motors;
    uint8_t active;
} StepperBank_t;

#define RUN_CMD 0x50
#define STEPCLK_CMD 0x58
#define MOVE_CMD 0x40
#define GOTO_CMD 0x60
#define GOTO_DIR_CMD 0x68
#define GOUNTIL_CMD 0x82
#define RELEASESW_CMD 0x92
#define GOHOME_CMD 0x70
#define GOMARK_CMD 0x78
#define RESETPOS_CMD 0xD8
#define RESETDEV_CMD 0xC0
#define SOFTSTOP_CMD 0xB0
#define HARDSTOP_CMD 0xB8
#define SOFTHIZ_CMD 0xA0
#define HARDHIZ_CMD 0xA8

// these defines enable the related alarm
#define ALARM_OVC 1
#define ALARM_THRM_SD 2
#define ALARM_THRM_WARN 4
#define ALARM_UVLO 8
#define ALARM_ADC_UVLO 16
#define ALARM_STALL 32
#define ALARM_SW_EVENT 64
#define ALARM_CMD_ERR 128

// masks for the STATUS register
#define STATUS_MASK_HIZ 0x01
#define STATUS_MASK_BUSY 0x02
#define STATUS_MASK_SW_F 0x04
#define STATUS_MASK_SW_EVN 0x08
#define STATUS_MASK_DIR 0x10
#define STATUS_MASK_MOT_STATUS 0x90
#define STATUS_MASK_CMD_ERROR 0x80
#define STATUS_MASK_STCK_MOD 0x100
#define STATUS_MASK_UVLO 0x200
#define STATUS_MASK_UVLO_ADC 0x400
#define STATUS_MASK_TH_STATUS 0x1800
#define STATUS_MASK_OCD 0x2000
#define STATUS_MASK_STALL_B 0x4000
#define STATUS_MASK_STALL_A 0x8000
// ----- mot status bits -----
#define MOT_STATUS_STOPPED 0x00
#define MOT_STATUS_ACCELERATION 0x01
#define MOT_STATUS_DECELERATION 0x02
#define MOT_STATUS_CONSTANT_SPEED 0x03
// ---------------------------

#define SM_FULLSTEP 0
#define SM_HALFSTEP 1
#define SM_4_MICROSTEP 2
#define SM_8_MICROSTEP 3
#define SM_16_MICROSTEP 4
#define SM_32_MICROSTEP 5
#define SM_64_MICROSTEP 6
#define SM_128_MICROSTEP 7
#define VOLTAGE_MODE 0
#define CURRENT_MODE 1
#define SYNC_DISABLED 0
#define SYNC_ENABLED 1

#define SW_DISABLED 0
#define SW_HARDSTOP 1
#define OC_NOSHUTDOWN 0
#define OC_SHUTDOWN 1
#define VCC_7V5 0
#define VCC_15V 1
#define UVLOVAL_6V3 0
#define UVLOVAL_10V 1
#define VSCOMP_DISABLE 0
#define VSCOMP_ENABLE 1
#define TRQ_REG_DISABLE 0
#define TRQ_REG_ENABLE 1
#define PRED_VSCOMP_DISABLE 0
#define PRED_VSCOMP_ENABLE 1

typedef enum
{
    ABS_POS = 0x01,
    EL_POS = 0x02,
    MARK = 0x03,
    SPEED = 0x04,
    ACC = 0x05,
    DEC = 0x06,
    MAX_SPEED = 0x07,
    MIN_SPEED = 0x08,
    ADC_OUT = 0x12,
    OCD_TH = 0x13,
    FS_SPD = 0x15,
    STEP_MODE = 0x16,
    ALARM_EN = 0x17,
    GATECFG1 = 0x18,
    GATECFG2 = 0x19,
    STATUS = 0x1B,
    CONFIG = 0x1A,
    KVAL_HOLD = 0x09,
    KVAL_RUN = 0x0A,
    KVAL_ACC = 0x0B,
    KVAL_DEC = 0x0C,
    INT_SPEED = 0x0D,
    ST_SLP = 0x0E,
    FN_SLP_ACC = 0x0F,
    FN_SLP_DEC = 0x10,
    K_THERM = 0x11,
    STALL_TH = 0x14,
    TVAL_HOLD = 0x09,
    TVAL_RUN = 0x0A,
    TVAL_ACC = 0x0B,
    TVAL_DEC = 0x0C,
    T_FAST = 0x0E,
    TON_MIN = 0x0F,
    TOFF_MIN = 0x10,
} PARAM_t;

#endif