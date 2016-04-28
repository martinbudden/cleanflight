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

typedef struct filterStatePt1_s {
    float state;
    float RC;
    float constdT;
} filterStatePt1_t;

// this holds the data required to update samples thru a filter
typedef struct biquad_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquad_t;

float pt1FilterApply(float input, filterStatePt1_t *filter, uint8_t f_cut, float dt);

void BiQuadNewLpf(float filterCutFreq, biquad_t *newState, uint32_t refreshRate);
float applyBiQuadFilter(float sample, biquad_t *state);

int32_t averageFilterInt32Apply(int32_t input, int32_t filterState[], uint8_t filterLength);
float averageFilterApply(float input, float filterState[], uint8_t filterLength);

void firFilterInit(float filterState[], uint8_t filterLength);
float firFilterApply(float input, float filterState[], uint8_t filterLength, const float coeffs[]);

void firFilterInt32Init(int32_t filterState[], uint8_t filterLength);
int32_t firFilterInt32Apply(int32_t input, int32_t filterState[], uint8_t filterLength, const int8_t coeffs[]);
