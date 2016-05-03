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

// constants to scale pidLuxFloat so output is same as pidMultiWiiRewrite
static const float luxPTermScale = 1.0f / 128;
static const float luxITermScale = 1000000.0f / 0x1000000;
static const float luxDTermScale = (0.000001f * (float)0xFFFF) / 512;
static const float luxGyroScale = 4.0f;


typedef struct pidLuxFloatStateAxis_s {
    float kP;
    float kI;
    float kD;
    float kT;

    float desiredRate;

    float PTerm;
    float ITerm;
    float ITermLimit;
    float DTerm;

    firFilter_t         gyroRateFirFilter;
    float               gyroRateBuf[PID_GYRORATE_BUF_LENGTH];

    filterStatePt1_t    DTermPt1Filter;
    firFilter_t         DTermAverageFilter;
    float               DTermAverageFilterBuf[PID_DTERM_AVERAGE_FILTER_BUF_LENGTH];
} pidLuxFloatStateAxis_t;

typedef struct pidLuxFloatState_s {
    pidLuxFloatStateAxis_t stateAxis[FD_INDEX_COUNT];
    float kGyro;
} pidLuxFloatState_t;

void pidLuxFloatInit(const pidProfile_t *pidProfile);
void pidLuxFloatUpdateGyroRate(const pidProfile_t *pidProfile);
void pidLuxFloatUpdateDesiredRate(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig);
void pidLuxFloatCalculate(const pidProfile_t *pidProfile);

// following to be deprecated
void pidLuxFloat(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig);
void pidLuxFloatShim(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig);


