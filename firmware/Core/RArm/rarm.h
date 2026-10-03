#ifndef _RARM_H_

#define N_JOINTS 6

#define J1_INDEX 0
#define J2_INDEX 1
#define J3_INDEX 2
#define J4_INDEX 3
#define J5_INDEX 4
#define J6_INDEX 5

#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 0

#include "../powerSTEP01/ps01.h"

extern StepperBank_t *motor_bank;

typedef struct
{
    uint8_t OVERCURRENT_SD;
    uint8_t VSCOMP;
    uint8_t STEP_MODE;
    uint32_t steps_rev;
    uint16_t reduction_ratio;
    float supply_voltage;
    float run_acc_dec_voltage;
    float hold_voltage;

    uint16_t max_speed;
    uint16_t min_speed;
    uint16_t acceleration;
    uint16_t deceleration;
    uint16_t fullstep_speed;

    float oc_threshold;
    float stall_threshold;
    int32_t min_degs;
    int32_t max_degs;
} RARM_SimpleConfig_t;

typedef struct
{
    uint8_t mot1_index;
    uint8_t mot2_index;
    float rotation_reduction_ratio; // reduction ratio between rotation gear and motor driving gear
} RARM_Gearbox_t;

void RARM_SetBank               (StepperBank_t *bank);
void RARM_SetConfig             (uint8_t joint_index, RARM_SimpleConfig_t *config);

int32_t RARM_GetPositionDegrees (uint8_t joint_index);

void RARM_GearboxMoveDegrees    (RARM_Gearbox_t *gearbox, int16_t degs);
void RARM_GearboxRotateDegrees  (RARM_Gearbox_t *gearbox, int16_t degs);
void RARM_MoveDegrees           (uint8_t joint_index, int16_t degs);
void RARM_Run                   (uint8_t joint_index, uint8_t dir, uint16_t rpm);
void RARM_HardBrake             (uint8_t joint_index);
void RARM_SoftBrake             (uint8_t joint_index);
void RARM_HardHiZ               (uint8_t joint_index);
void RARM_SoftHiZ               (uint8_t joint_index);

#endif