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

#define SRF10_MAX_RANGE_CM 600 // 6m, from SFR10 spec sheet
// the two below values need testing
#define SRF10_DETECTION_CONE_DECIDEGREES 400 // ascertained from beam pattern on spec sheet
#define SRF10_DETECTION_CONE_EXTENDED_DECIDEGREES 450 // ascertained from beam pattern on spec sheet

void sfr10_init(sonarRange_t *sonarRange);
void srf10_start_reading(void);
int32_t srf10_get_distance(void);

