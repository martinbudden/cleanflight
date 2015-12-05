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

#include "sensors/battery.h"

typedef enum {
    SONAR_HCSR04 = 0,
    SONAR_SRF10
} sonarHardwareType_e;

typedef struct sonarGPIOConfig_s {
    GPIO_TypeDef *gpio;
    const uint16_t trigger_pin;
    const uint16_t echo_pin;
} sonarGPIOConfig_t;

typedef struct sonarRange_s {
    int16_t maxRangeCm;
    // these are full detection cone angles, maximum tilt is half of this
    int16_t detectionConeDeciDegrees; // detection cone angle as in HC-SR04 device spec
    int16_t detectionConeExtendedDeciDegrees; // device spec is conservative, in practice have slightly larger detection cone
} sonarRange_t;

typedef void (*sonarInitFunctionPtr)(sonarRange_t *sonarRange);
typedef void (*sonarUpdateFunctionPtr)(void);
typedef int32_t (*sonarReadFunctionPtr)(void);

typedef struct sonarFunctionPointers_s {
    sonarInitFunctionPtr init;
    sonarUpdateFunctionPtr update;
    sonarReadFunctionPtr read;
} sonarFunctionPointers_t;

const sonarGPIOConfig_t *sonarGetHardwareConfiguration(currentSensor_e currentSensor);
const sonarGPIOConfig_t *sonarGetHardwareConfigurationForType(sonarHardwareType_e sonarHardware, currentSensor_e currentSensor);
void sonarInit();
