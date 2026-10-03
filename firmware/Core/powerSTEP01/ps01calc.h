#ifndef __PS01CALC_H__
#define __PS01CALC_H__

#include <inttypes.h>
#include "cfg_structs.h"

uint8_t calculateKVAL(float supply, float target);
uint16_t calculateMaxMinSpeed(uint16_t steps_s);
uint16_t calculateAcceleration(uint16_t steps_s2);
uint32_t calculateSpeed(uint16_t steps_s);
int32_t getStepsFromAngle(int32_t deg, Stepper_t *motor);

#endif