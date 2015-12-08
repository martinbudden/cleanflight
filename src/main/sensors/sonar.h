/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "drivers/sonar_hcsr04.h"
#include "drivers/sonar.h"
#include "sensors/battery.h"

typedef enum {
    SONAR_HCSR04 = 0,
    SONAR_SRF10,
    SONAR_LIDAR_LITE
} sonarHardwareType_e;

typedef void (*sonarInitFunctionPtr)(sonarRange_t *sonarRange);
typedef void (*sonarUpdateFunctionPtr)(void);
typedef int32_t (*sonarReadFunctionPtr)(void);

typedef struct sonarFunctionPointers_s {
    sonarInitFunctionPtr init;
    sonarUpdateFunctionPtr update;
    sonarReadFunctionPtr read;
} sonarFunctionPointers_t;

extern int16_t sonarMaxRangeCm;
extern int16_t sonarCfAltCm;
extern int16_t sonarMaxAltWithTiltCm;

const sonarGPIOConfig_t *sonarConfigureHardwareForType(sonarHardwareType_e sonarHardware, currentSensor_e currentSensor);
const sonarGPIOConfig_t *sonarConfigureHardware(currentSensor_e currentSensor);
int16_t sonarCalculateTiltAngle(int16_t rollDeciDegrees, int16_t pitchDeciDegrees);
int32_t sonarCalculateAltitude(int32_t sonarDistance, int16_t rollDeciDegrees, int16_t pitchDeciDegrees);
int32_t sonarGetLatestAltitude(void);
void sonarInit();
void sonarUpdate(void);
int32_t sonarRead(void);

