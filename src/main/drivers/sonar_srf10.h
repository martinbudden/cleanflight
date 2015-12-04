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

#define SRF10_MAX_RANGE_CM 600 // 6m, from SFR10 spec sheet, see http://www.robot-electronics.co.uk/htm/srf10tech.htm
// see http://www.robot-electronics.co.uk/htm/sonar_faq.htm for cone angles
// FAQ states 55 degrees, conservatively reduced to 50 degrees here
#define SRF10_DETECTION_CONE_DECIDEGREES 500
#define SRF10_DETECTION_CONE_EXTENDED_DECIDEGREES 500

const sonarGPIOConfig_t *srf10_get_hardware_configuration();
void srf10_init(sonarRange_t *sonarRange, sonarFunctionPointers_t* sonarFunctionPointers);
void srf10_start_reading(void);
int32_t srf10_get_distance(void);

