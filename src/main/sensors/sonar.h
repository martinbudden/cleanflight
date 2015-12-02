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

#define SONAR_OUT_OF_RANGE (-1)

typedef void (*sonarUpdateFunctionPtr_t)(void);
typedef int32_t (*sonarReadFunctionPtr_t)(void);

typedef struct sonarRange_s {
    int16_t maxRangeCm;
    // these are full detection cone angles, maximum tilt is half of this
    int16_t detectionConeDeciDegrees; // detection cone angle as in device spec
    int16_t detectionConeExtendedDeciDegrees; // device spec is usually conservative, in practice have slightly larger detection cone
} sonarRange_t;

extern int16_t sonarMaxRangeCm;
extern int16_t sonarCfAltCm;
extern int16_t sonarMaxAltWithTiltCm;

extern sonarUpdateFunctionPtr_t sonarUpdateFunctionPtr;
extern sonarReadFunctionPtr_t sonarReadFunctionPtr;

void sonarUpdate(void);
int32_t sonarRead(void);
int16_t sonarCalculateTiltAngle(int16_t rollDeciDegrees, int16_t pitchDeciDegrees);
int32_t sonarCalculateAltitude(int32_t sonarDistance, int16_t rollDeciDegrees, int16_t pitchDeciDegrees);
int32_t sonarGetLatestAltitude(void);

