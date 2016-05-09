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

#define DELTA_MAX_SAMPLES 12

typedef struct filterStatePt1_s {
	float state;
	float RC;
	float constdT;
} filterStatePt1_t;

/* this holds the data required to update samples thru a filter */
typedef struct biquad_s {
    float b0, b1, b2, a1, a2;
    float d1, d2;
} biquad_t;

typedef struct firFilter_s {
    float *buf;
    const float *coeffs;
    uint8_t bufLength;
    uint8_t coeffsLength;
} firFilter_t;


void BiQuadNewLpf(float filterCutFreq, biquad_t *newState, uint32_t refreshRate);
float applyBiQuadFilter(float sample, biquad_t *state);

float filterApplyPt1(float input, filterStatePt1_t *filter, float f_cut, float dt);

int32_t filterApplyAverage(int32_t input, uint8_t averageCount, int32_t averageState[DELTA_MAX_SAMPLES]);
float filterApplyAveragef(float input, uint8_t averageCount, float averageState[DELTA_MAX_SAMPLES]);

void firFilterInit(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs);
void firFilterInit2(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs, uint8_t coeffsLength);
void firFilterUpdate(firFilter_t *filter, float input);
float firFilterApply(firFilter_t *filter);
float firFilterCalcPartialAverage(firFilter_t *filter, uint8_t count);
float firFilterCalcAverage(firFilter_t *filter);
float firFilterLastInput(firFilter_t *filter);
