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


typedef struct pidMwrStateAxis_s {
    int32_t kP;
    int32_t kI;
    int32_t kD;
    int32_t kT;

    int32_t desiredRate;

    int32_t PTerm;
    int32_t ITerm;
    int32_t ITermLimit;
    int32_t DTerm;


    firFilterInt32_t    gyroRateFirFilter;
    int32_t             gyroRateFirFilterBuf[PID_GYRORATE_BUF_LENGTH];

    filterStatePt1_t    DTermPt1Filter;
} pidMwrStateAxis_t;

typedef struct pidMwrState_s {
    pidMwrStateAxis_t stateAxis[FD_INDEX_COUNT];
} pidMwrState_t;

void pidMwrInit(const pidProfile_t *pidProfile);
void pidMwrUpdateGyroRate(const pidProfile_t *pidProfile);
void pidMwrUpdateDesiredRate(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig);
void pidMwrCalculate(const pidProfile_t *pidProfile);

// following to be deprecated
void pidMultiWiiRewrite(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig);
void pidMultiWiiRewriteShim(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig);
