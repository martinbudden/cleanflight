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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#define M_LN2_FLOAT	0.69314718055994530942f
#define M_PI_FLOAT	3.14159265358979323846f

#define BIQUAD_BANDWIDTH 1.9f     /* bandwidth in octaves */

// PT1 Low Pass filter (when no dT specified it will be calculated from the cycleTime)
float filterApplyPt1(float input, filterStatePt1_t *filter, uint8_t f_cut, float dT) {

	// Pre calculate and store RC
	if (!filter->RC) {
		filter->RC = 1.0f / ( 2.0f * (float)M_PI * f_cut );
	}

    filter->state = filter->state + dT / (filter->RC + dT) * (input - filter->state);

    return filter->state;
}

/* sets up a biquad Filter */
void BiQuadNewLpf(float filterCutFreq, biquad_t *newState, uint32_t refreshRate)
{
    float sampleRate;

    sampleRate = 1 / ((float)refreshRate * 0.000001f);

    float omega, sn, cs, alpha;
    float a0, a1, a2, b0, b1, b2;

    /* setup variables */
    omega = 2 * M_PI_FLOAT * filterCutFreq / sampleRate;
    sn = sinf(omega);
    cs = cosf(omega);
    alpha = sn * sinf(M_LN2_FLOAT /2 * BIQUAD_BANDWIDTH * omega /sn);

    b0 = (1 - cs) /2;
    b1 = 1 - cs;
    b2 = (1 - cs) /2;
    a0 = 1 + alpha;
    a1 = -2 * cs;
    a2 = 1 - alpha;

    /* precompute the coefficients */
    newState->b0 = b0 /a0;
    newState->b1 = b1 /a0;
    newState->b2 = b2 /a0;
    newState->a1 = a1 /a0;
    newState->a2 = a2 /a0;

    /* zero initial samples */
    newState->x1 = newState->x2 = 0;
    newState->y1 = newState->y2 = 0;
}

/* Computes a biquad_t filter on a sample */
float applyBiQuadFilter(float sample, biquad_t *state)
{
    float result;

    /* compute result */
    result = state->b0 * sample + state->b1 * state->x1 + state->b2 * state->x2 -
        state->a1 * state->y1 - state->a2 * state->y2;

    /* shift x1 to x2, sample to x1 */
    state->x2 = state->x1;
    state->x1 = sample;

    /* shift y1 to y2, result to y1 */
    state->y2 = state->y1;
    state->y1 = result;

    return result;
}

int32_t filterApplyAverage(int32_t input, uint8_t count, int32_t averageState[])
{
    int32_t sum = 0;
    for (int ii = count - 1; ii > 0; --ii) {
        averageState[ii] = averageState[ii-1];
        sum += averageState[ii];
    }
    averageState[0] = input;
    sum += input;
    return sum / count;
}

float filterApplyAveragef(float input, uint8_t count, float averageState[])
{
    float sum = 0;
    for (int ii = count - 1; ii > 0; --ii) {
        averageState[ii] = averageState[ii-1];
        sum += averageState[ii];
    }
    averageState[0] = input;
    sum += input;
    return sum / count;
}

void firFilterInit2(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs, uint8_t coeffsLength)
{
    filter->buf = buf;
    filter->bufLength = bufLength;
    filter->coeffs = coeffs;
    filter->coeffsLength = coeffsLength;
    memset(filter->buf, 0, sizeof(float) * filter->bufLength);
}

void firFilterInit(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs)
{
    firFilterInit2(filter, buf, bufLength, coeffs, bufLength);
}

void firFilterUpdate(firFilter_t *filter, float input)
{
    memmove(&filter->buf[1], &filter->buf[0], (filter->bufLength-1) * sizeof(float));
    filter->buf[0] = input;
}

float firFilterApply(firFilter_t *filter)
{
    float ret = 0.0f;
    for (int ii = 0; ii < filter->coeffsLength; ++ii) {
        ret += filter->coeffs[ii] * filter->buf[ii];
    }
    return ret;
}

float firFilterCalcPartialAverage(firFilter_t *filter, uint8_t count)
{
    float ret = 0.0f;
    for (int ii = 0; ii < count; ++ii) {
        ret += filter->buf[ii];
    }
    return ret / count;
}

float firFilterCalcAverage(firFilter_t *filter)
{
    return firFilterCalcPartialAverage(filter, filter->coeffsLength);
}

float firFilterLastInput(firFilter_t *filter)
{
    return filter->buf[0];
}

// integer based FIR filter
// coefficients are multiples of 1/64
void firFilterInt32Init2(firFilterInt32_t *filter, int32_t *buf, uint8_t bufLength, const int8_t *coeffs, uint8_t coeffsLength)
{
    filter->buf = buf;
    filter->bufLength = bufLength;
    filter->coeffs = coeffs;
    filter->coeffsLength = coeffsLength;
    memset(filter->buf, 0, sizeof(int32_t) * filter->bufLength);
}

void firFilterInt32Init(firFilterInt32_t *filter, int32_t *buf, uint8_t bufLength, const int8_t *coeffs)
{
    firFilterInt32Init2(filter, buf, bufLength, coeffs, bufLength);
}

void firFilterInt32Update(firFilterInt32_t *filter, float input)
{
    memmove(&filter->buf[1], &filter->buf[0], (filter->bufLength-1) * sizeof(int32_t));
    filter->buf[0] = input;
}

int32_t firFilterInt32Apply(firFilterInt32_t *filter)
{
    int32_t ret = 0;
    for (int ii = 0; ii < filter->coeffsLength; ++ii) {
        ret += filter->coeffs[ii] * filter->buf[ii];
    }
    return ret / 64;
}

int32_t firFilterInt32CalcPartialAverage(firFilterInt32_t *filter, uint8_t count)
{
    int32_t ret = 0;
    for (int ii = 0; ii < count; ++ii) {
        ret += filter->buf[ii];
    }
    return ret / count;
}

int32_t firFilterInt32CalcAverage(firFilterInt32_t *filter)
{
    return firFilterInt32CalcPartialAverage(filter, filter->coeffsLength);
}

int32_t firFilterInt32LastInput(firFilterInt32_t *filter)
{
    return filter->buf[0];
}
