#include "rarm.h"
#include "main.h"

StepperBank_t *motor_bank;

void RARM_SetBank(StepperBank_t *bank)
{
    motor_bank = bank;
    ps01SetBank(bank, N_JOINTS);
}

void RARM_SetConfig(uint8_t joint_index, RARM_SimpleConfig_t *config)
{
    mot_bank->active = joint_index;

    mot_bank->motors[joint_index].steps_rev = config->steps_rev; // steps/revolution
    mot_bank->motors[joint_index].reduction_ratio = config->reduction_ratio; // 1:reduction_ratio
    mot_bank->motors[joint_index].min_degs = config->min_degs;
    mot_bank->motors[joint_index].max_degs = config->max_degs;

    // just leave these two settings like this...
    // https://www.st.com/resource/en/datasheet/powerstep01.pdf
    // Gate driver: table 11, targeting ~520 V/us slew rate
    // IGATE=110 (64mA), TCC=01000 (1125ns), TBOOST=000, WD_EN=0
    ps01SetParam_chain(GATECFG1, 0x0C8);

    // TBLANK=010 (375ns), TDT=001 (250ns)
    ps01SetParam_chain(GATECFG2, 0x41);

    mot_bank->motors[joint_index].config.bits.OSC_SEL = 0;
    mot_bank->motors[joint_index].config.bits.EXT_CLK = 0;
    mot_bank->motors[joint_index].config.bits.SW_MODE = SW_DISABLED;
    mot_bank->motors[joint_index].config.bits.EN_VSCOMP = config->VSCOMP;
    mot_bank->motors[joint_index].config.bits.OC_SD = config->OVERCURRENT_SD;
    mot_bank->motors[joint_index].config.bits.UVLOVAL = UVLOVAL_6V3;
    mot_bank->motors[joint_index].config.bits.VCCVAL = VCC_7V5;
    mot_bank->motors[joint_index].config.bits.F_PWM_DEC = 0b011;
    mot_bank->motors[joint_index].config.bits.F_PWM_INT = 0b000;
    ps01SetConfig_chain();

    mot_bank->motors[joint_index].stepmode.bits.STEP_SEL = config->STEP_MODE;
    mot_bank->motors[joint_index].stepmode.bits.CM_VM = VOLTAGE_MODE;    // CURRENT_MODE is NOT supported
    mot_bank->motors[joint_index].stepmode.bits.SYNC_EN = SYNC_DISABLED;
    mot_bank->motors[joint_index].stepmode.bits.SYNC_SEL = 0;
    ps01SetStepMode_chain();

    // to calculate target voltage to apply to motor windings:
    // target = amps_phase * resistance_phase
    // example: 17E1K-07 motor. target = 2 * 1.75 = 3.5V
    mot_bank->motors[joint_index].kvals.acceleration = calculateKVAL(config->supply_voltage, config->run_acc_dec_voltage);
    mot_bank->motors[joint_index].kvals.deceleration = calculateKVAL(config->supply_voltage, config->run_acc_dec_voltage);
    mot_bank->motors[joint_index].kvals.run = calculateKVAL(config->supply_voltage, config->run_acc_dec_voltage);
    mot_bank->motors[joint_index].kvals.hold = calculateKVAL(config->supply_voltage, config->hold_voltage);
    ps01SetKVALs_chain();

    // all speed/acceleration values in FULL steps/s or steps/s²
    ps01SetMaxSpeed_chain(config->max_speed);
    ps01SetMinSpeed_chain(config->min_speed);
    ps01SetAcceleration_chain(config->acceleration);
    ps01SetDeceleration_chain(config->deceleration);
    ps01SetFullStepSpeed_chain(config->fullstep_speed);

    ps01SetOcThreshold_chain(config->oc_threshold);
    ps01SetStallThreshold_chain(config->stall_threshold);
    ps01SetAlarms_chain(ALARM_STALL | ALARM_CMD_ERR | ALARM_OVC | ALARM_THRM_SD | ALARM_THRM_WARN | ALARM_UVLO);
    ps01GetStatus_chain();  // clear error flags
}

int32_t RARM_GetPositionDegrees(uint8_t joint_index)
{
    int32_t steps = ps01GetPosition_chain(joint_index);
    int64_t steps_per_rev = mot_bank->motors[joint_index].steps_rev;
    int64_t ratio = mot_bank->motors[joint_index].reduction_ratio;
    int64_t microsteps = mot_bank->motors[joint_index].stepmode.bits.STEP_SEL;

    // ABS_POS is in microsteps, so full rev = steps_per_rev * microstep_mode
    return (int32_t)((int64_t)steps * 360 / (steps_per_rev << microsteps) / ratio);
}

void RARM_GearboxMoveDegrees(RARM_Gearbox_t *gearbox, int16_t degs)
{
    // when the gearbox moves left/right, the gears have the same direction.
    // the motors are mounted in anti-parallel position, so the two motors
    // will rotate in the opposite direction
    RARM_MoveDegrees(gearbox->mot1_index, degs);
    RARM_MoveDegrees(gearbox->mot2_index, -degs);
}

void RARM_GearboxRotateDegrees(RARM_Gearbox_t *gearbox, int16_t degs)
{
    // when the gearbox rotates (end effector rotates on its axis) the gears have the opposite direction
    // the motors are mounted in anti-parallel position, so the two motors
    // will rotate in the same direction
    degs *= gearbox->rotation_reduction_ratio;
    RARM_MoveDegrees(gearbox->mot1_index, degs);
    RARM_MoveDegrees(gearbox->mot2_index, degs);
}

void RARM_MoveDegrees(uint8_t joint_index, int16_t degs)
{
    if (joint_index >= N_JOINTS)
        return;

    mot_bank->active = joint_index;

    int32_t current = RARM_GetPositionDegrees(joint_index);
    int32_t target = current + (int32_t)degs;

    if (target < mot_bank->motors[joint_index].min_degs)
        target = mot_bank->motors[joint_index].min_degs;
    else if (target > mot_bank->motors[joint_index].max_degs)
        target = mot_bank->motors[joint_index].max_degs;

    int32_t delta = target - current;

    uint8_t dir = CLOCKWISE;
    if (delta < 0)
    {
        delta *= -1;
        dir = COUNTER_CLOCKWISE;
    }

    ps01MoveDegrees_chain(dir, (uint32_t)delta);
}

void RARM_Run(uint8_t joint_index, uint8_t dir, uint16_t rpm)
{
    mot_bank->active = joint_index;
    uint32_t steps_s = (uint32_t)rpm * mot_bank->motors[joint_index].steps_rev / 60;
    ps01Run_chain(dir, steps_s);
}

void RARM_HardBrake(uint8_t joint_index)
{
    mot_bank->active = joint_index;
    ps01HardStop_chain();
}

void RARM_SoftBrake(uint8_t joint_index)
{
    mot_bank->active = joint_index;
    ps01SoftStop_chain();
}

void RARM_HardHiZ(uint8_t joint_index)
{
    mot_bank->active = joint_index;
    ps01HardHiZ_chain();
}

void RARM_SoftHiZ(uint8_t joint_index)
{
    mot_bank->active = joint_index;
    ps01SoftHiZ_chain();
}