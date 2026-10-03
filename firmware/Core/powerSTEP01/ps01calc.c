#include "ps01calc.h"

// calculate KVAL based on supply (VS) voltage and target voltage to be applied to the motor
uint8_t calculateKVAL(float supply, float target)
{
    if (supply <= 0.0f) return 0;
    float kval = (target / supply) * 256.0f;
    if (kval >= 255.0f) return 255;
    if (kval <= 0.0f)   return 0;
    return (uint8_t)kval;
}

// steps_s = SPEED * 2^-18 / tick
// where tick is 250ns (2.5^-7s)
// SPEED = steps_s * tick / 2^-18 = steps_s * 0.065536
uint16_t calculateMaxMinSpeed(uint16_t steps_s)
{
    if (steps_s > 15610) return 1023;
    return steps_s * 0.065536;
}

uint16_t calculateAcceleration(uint16_t steps_s2)
{
    uint16_t acc_val = steps_s2 * 0.06871948;
    if (acc_val >= 0xFFF) return 0xFFE;
    return acc_val; 
}

uint32_t calculateSpeed(uint16_t steps_s)
{
    if (steps_s > 15625) return 1048576; 
    return steps_s * 67.108864;
}

int32_t getStepsFromAngle(int32_t deg, Stepper_t *motor)
{
    int32_t excess = 0;
    uint8_t isnegative = (deg < 0) ? 1 : 0;
    if (isnegative)
        deg *= -1;

    if (deg > 360)
    {
        excess = (motor->steps_rev * (deg / 360)) << motor->stepmode.bits.STEP_SEL;
        deg -= (deg / 360) * 360;
    }

    if (!isnegative)
        return (((int32_t)((float)motor->steps_rev / (360.0 / (float)deg)) << motor->stepmode.bits.STEP_SEL) + excess) * motor->reduction_ratio;
    else
        return (((int32_t)((float)motor->steps_rev / (360.0 / (float)deg)) << motor->stepmode.bits.STEP_SEL) * -1 + excess) * motor->reduction_ratio;

}