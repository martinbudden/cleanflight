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
    const float *coeffs;
    uint8_t bufLength;
    uint8_t coeffsLength;
} firFilter_t;

typedef struct firFilterInt32_s {
    int32_t *buf;
    const int8_t *coeffs;
    uint8_t bufLength;
    uint8_t coeffsLength;
} firFilterInt32_t;

void biQuadFilterInit(biquad_t *filter, float filterCutFreq, uint32_t refreshRate);
float biQuadFilterApply(biquad_t *filter, float input);

void pt1FilterInit(filterStatePt1_t *filter, uint8_t f_cut);
float pt1FilterApply(filterStatePt1_t *filter, float input, uint8_t f_cut, float dT);


void averageFilterInit(averageFilter_t *filter, float *buf, uint8_t length);
void averageFilterUpdate(averageFilter_t *filter, float input);
float averageFilterApply(averageFilter_t *filter);

void averageFilterInt32Init(averageFilterInt32_t *filter, int32_t *buf, uint8_t length);
void averageFilterInt32Update(averageFilterInt32_t *filter, int32_t input);
int32_t averageFilterInt32Apply(averageFilterInt32_t *filter);

void firFilterInit(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs);
void firFilterInit2(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs, uint8_t coeffsLength);
void firFilterUpdate(firFilter_t *filter, float input);
float firFilterApply(firFilter_t *filter);
float firFilterCalcPartialAverage(firFilter_t *filter, uint8_t count);
float firFilterCalcAverage(firFilter_t *filter);
float firFilterLastItem(firFilter_t *filter);

void firFilterInt32Init(firFilterInt32_t *filter, int32_t *buf, uint8_t bufLength, const int8_t *coeffs);
void firFilterInt32Init2(firFilterInt32_t *filter, int32_t *buf, uint8_t bufLength, const int8_t *coeffs, uint8_t coeffsLength);
void firFilterInt32Update(firFilterInt32_t *filter, float input);
int32_t firFilterInt32Apply(firFilterInt32_t *filter);
int32_t firFilterInt32CalcPartialAverage(firFilterInt32_t *filter, uint8_t count);
int32_t firFilterInt32CalcAverage(firFilterInt32_t *filter);
int32_t firFilterInt32LastItem(firFilterInt32_t *filter);
