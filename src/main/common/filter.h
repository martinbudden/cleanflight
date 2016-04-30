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


typedef struct averageFilter_s {
    float *buf;
    uint8_t length;
} averageFilter_t;

typedef struct averageFilterInt32_s {
    int32_t *buf;
    uint8_t length;
} averageFilterInt32_t;

typedef struct firFilter_s {
    float *buf;
    const float *coefficients;
    uint8_t length;
} firFilter_t;

typedef struct firFilterInt32_s {
    int32_t *buf;
    const int8_t *coefficients;
    uint8_t length;
} firFilterInt32_t;

float pt1FilterApply(float input, filterStatePt1_t *filter, uint8_t f_cut, float dt);

void BiQuadNewLpf(float filterCutFreq, biquad_t *newState, uint32_t refreshRate);
float applyBiQuadFilter(float sample, biquad_t *state);

void averageFilterInit(averageFilter_t *state, float *buf, uint8_t length);
void averageFilterUpdate(averageFilter_t *state, float input);
float averageFilterApply(averageFilter_t *state);

void averageFilterInt32Init(averageFilterInt32_t *state, int32_t *buf, uint8_t length);
void averageFilterInt32Update(averageFilterInt32_t *state, int32_t input);
int32_t averageFilterInt32Apply(averageFilterInt32_t *state);

void firFilterInit(firFilter_t *state, float *buf, uint8_t length, const float *coefficients);
void firFilterUpdate(firFilter_t *state, float input);
float firFilterApply(firFilter_t *state);

void firFilterInt32Init(firFilterInt32_t *state, int32_t *buf, uint8_t length, const int8_t *coefficients);
void firFilterInt32Update(firFilterInt32_t *state, float input);
float firFilterInt32Apply(firFilterInt32_t *state);

