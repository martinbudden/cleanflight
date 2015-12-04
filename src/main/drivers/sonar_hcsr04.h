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

#include "platform.h"
#include "sensors/sonar.h"
#include "sensors/sonar_init.h"

#define HCSR04_MAX_RANGE_CM 400 // 4m, from HC-SR04 spec sheet
#define HCSR04_DETECTION_CONE_DECIDEGREES 300 // recommended cone angle 30 degrees, from HC-SR04 spec sheet
#define HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES 450 // in practice 45 degrees seems to work well

const sonarGPIOConfig_t *hcsr04_get_hardware_configuration(currentSensor_e currentSensor);
void hcsr04_init(sonarRange_t *sonarRange, sonarFunctionPointers_t *sonarFunctionPointers);
void hcsr04_start_reading(void);
int32_t hcsr04_get_distance(void);
