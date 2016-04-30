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

#define SRC_MAIN_FLIGHT_PID_LUXFLOAT_C_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>

#include "build_config.h"

#ifndef SKIP_PID_LUXFLOAT

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/parameter_group.h"
#include "config/runtime_config.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "io/rc_controls.h"
#include "io/rate_profile.h"

#include "flight/pid.h"
#include "config/config_unittest.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/gtune.h"
#include "flight/mixer.h"

extern float dT;

extern uint8_t motorCount;

#ifdef BLACKBOX
extern int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif

// constants to scale pidLuxFloat so output is same as pidMultiWiiRewrite
static const float luxPTermScale = 1.0f / 128;
static const float luxITermScale = 1000000.0f / 0x1000000;
static const float luxDTermScale = (0.000001f * (float)0xFFFF) / 512;
static const float luxGyroScale = 16.4f / 4; // the 16.4 is needed because mwrewrite does not scale according to the gyro model gyro.scale


/* Noise-robust differentiator filter coefficients by Pavel Holoborodko, see
http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
N=2: h[0] = 1, h[-1] = -1 - filter length 2, simple differentiation
N=3: h[0] = 1/2, h[-1] = 0, h[-2] = -1/2 - filter length 3, simple differentiation with 2 point moving average
Precise on 1, x:
N=4: 1/4  (h[0] = 1, h[-1] = 1, h[-2] =-1, h[-3] =-1)
N=5: 1/8  (h[0] = 1, h[-1] = 2, h[-2] = 0, h[-3] =-2, h[-4] =-1)
N=6: 1/16 (h[0] = 1, h[-1] = 3, h[-2] = 2, h[-3] =-2, h[-4] =-3, h[-5] =-1)
N=7: 1/32 (h[0] = 1, h[-1] = 4, h[-2] = 5, h[-3] = 0, h[-4] =-5, h[-5] =-4, h[-6] =-1)
N=8: 1/64 (h[0] = 1, h[-1] = 5, h[-2] = 9, h[-3] = 5, h[-4] =-5, h[-5] =-9, h[-6] =-5, h[-7] =-1)
Precise on 1, x, x^2:
N=5: h[0] = 5/8, h[-1] = 1/4, h[-2] = -1, h[-3] = -1/4, h[-4] = 3/8
N=6: h[0] = 3/8, h[-1] = 1/2, h[-2] = -1/2, h[-3] = -3/4, h[-4] = 1/8, h[-5] = 1/4
N=7: h[0] = 7/32, h[-1] = 1/2, h[-2] = -1/32, h[-3] = -3/4, h[-4] = -11/32, h[-5] = 1/4, h[-6] = 5/32
N=8: h[0] = 1/8, h[-1] = 13/32, h[-2] = 1/4, h[-3] = -15/32, h[-4] = -5/8, h[-5] = -1/32, h[-6] = 1/4, h[-7] = 3/32
*/

static const float nrdCoeffs2[] = { 1.0f,   -1.0f };
static const float nrdCoeffs3[] = { 1.0f/2,  0.0f,   -1.0f/2 };
static const float nrdCoeffs4[] = { 1.0f/4,  1.0f/4, -1.0f/4, -1.0f/4 };
#ifdef DTERM_PRECISE_ON_LINEAR
static const float nrdCoeffs5[] = { 1.0f/8,  1.0f/4,  0.0f,   -1.0f/4,  1.0f/8 };
static const float nrdCoeffs6[] = { 1.0f/16, 3.0f/16, 1.0f/8, -1.0f/8, -3.0f/16,-1.0f/16 };
static const float nrdCoeffs7[] = { 1.0f/32, 1.0f/8,  5.0f/32, 0.0f,   -5.0f/32,-1.0f/8, -1.0f/32 };
static const float nrdCoeffs8[] = { 1.0f/64, 5.0f/64, 9.0f/64, 5.0f/64,-5.0f/64,-9.0f/64,-5.0f/64,-1.0/64 };
#else
static const float nrdCoeffs5[] = { 5.0f/8,  1.0f/4, -1.0f,   -1.0f/4,  3.0f/8 };
static const float nrdCoeffs6[] = { 3.0f/8,  1.0f/2, -1.0f/2, -3.0f/4,  1.0f/8,  1.0f/4 };
static const float nrdCoeffs7[] = { 7.0f/32, 1.0f/2, -1.0f/32,-3.0f/4,-11.0f/32, 1.0f/4,  5.0f/32 };
static const float nrdCoeffs8[] = { 1.0f/8, 13.0f/32, 1.0f/4,-15.0f/32,-5.0f/8, -1.0f/32,  1.0f/4, 3.0/32 };
#endif
static const float *nrd[] = {nrdCoeffs2, nrdCoeffs3, nrdCoeffs4, nrdCoeffs5, nrdCoeffs6, nrdCoeffs7, nrdCoeffs8};

typedef struct pidStateAxis_s {
    float kP;
    float kI;
    float kD;
    float kT;

    float gyroRate;
    float desiredRate;


    uint8_t PIDweight;
    float PTerm;
    float ITerm;
    float ITermLimitf;
    float DTerm;

    filterStatePt1_t DTermPt1FilterState;
    firFilter_t DTermFirFilterState;
    float DTermFirFilterBuf[PID_DTERM_FIR_MAX_LENGTH];
    averageFilter_t DTermAverageFilterState;
    float DTermAverageFilterBuf[PID_DTERM_AVERAGE_FILTER_MAX_LENGTH];
} pidStateAxis_t;

typedef struct pidState_s {
    pidStateAxis_t stateAxis[FD_INDEX_COUNT];
} pidState_t;

static pidState_t pidState;

STATIC_UNIT_TESTED void pidUpdateGyroStateAxis(flight_dynamics_index_t axis, const pidProfile_t *pidProfile, pidStateAxis_t* pidStateAxis)
{
    SET_PID_LUX_FLOAT_CORE_LOCALS(axis);

    pidStateAxis->gyroRate = luxGyroScale * gyroADC[axis] * gyro.scale;
    const float rateError = pidStateAxis->desiredRate - pidStateAxis->gyroRate;


    // -----calculate I component
    pidStateAxis->ITerm += rateError * pidStateAxis->kI * dT;

    // -----calculate D component
    if (pidProfile->D8[axis] != 0) {
        // optimisation for when D8 is zero, often used by YAW axis
        // Calculate derivative using FIR filter
        firFilterUpdate(&pidStateAxis->DTermFirFilterState, pidStateAxis->gyroRate);
        pidStateAxis->DTerm = -firFilterApply(&pidStateAxis->DTermFirFilterState) / dT;

        if (pidProfile->dterm_lpf_hz) {
            // DTerm delta low pass filter
            pidStateAxis->DTerm = pt1FilterApply(pidStateAxis->DTerm, &pidStateAxis->DTermPt1FilterState, pidProfile->dterm_lpf_hz, dT);
        }
        if (pidProfile->dterm_average_count) {
            // Apply moving average
            averageFilterUpdate(&pidStateAxis->DTermAverageFilterState, pidStateAxis->DTerm);
        }
    }
}

void pidUpdateGyroState(const pidProfile_t *pidProfile)
{
    for (int axis = 0; axis < 3; axis++) {
        pidUpdateGyroStateAxis(axis, pidProfile, &pidState.stateAxis[axis]);
    }
}

static float calcHorizonLevelStrength(const pidProfile_t *pidProfile, const rxConfig_t *rxConfig)
{
    // Figure out the most deflected stick position
    const int32_t stickPosAil = ABS(getRcStickDeflection(FD_ROLL, rxConfig->midrc));
    const int32_t stickPosEle = ABS(getRcStickDeflection(FD_PITCH, rxConfig->midrc));
    const int32_t mostDeflectedPos = MAX(stickPosAil, stickPosEle);

    // Progressively turn off the horizon self level strength as the stick is banged over
    float horizonLevelStrength = (float)(500 - mostDeflectedPos) / 500;  // 1 at centre stick, 0 = max stick deflection
    if (pidProfile->D8[PIDLEVEL] == 0){
        horizonLevelStrength = 0;
    } else {
        horizonLevelStrength = constrainf(((horizonLevelStrength - 1) * (100.0f / pidProfile->D8[PIDLEVEL])) + 1, 0, 1);
    }
    return horizonLevelStrength;
}

static void pidUpdateRcStateAxis(int axis, const pidProfile_t *pidProfile, pidStateAxis_t* pidStateAxis, float horizonLevelStrength,
        const controlRateConfig_t *controlRateConfig, uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim)
{
    pidStateAxis->kP = luxPTermScale * pidProfile->P8[axis] * pidStateAxis->PIDweight / 100;
    pidStateAxis->kI = luxITermScale * pidProfile->I8[axis];
    pidStateAxis->kD = luxDTermScale * pidProfile->D8[axis] * pidStateAxis->PIDweight / 100;

    const uint8_t rate = controlRateConfig->rates[axis];

    // -----Get the desired angle rate depending on flight mode
    if (axis == FD_YAW) {
        // YAW is always gyro-controlled (MAG correction is applied to rcCommand) 100dps to 1100dps max yaw rate
        pidStateAxis->desiredRate = (float)((rate + 27) * rcCommand[YAW]) / 32.0f;
    } else {
        // control is GYRO based for ACRO and HORIZON - direct sticks control is applied to rate PID
        pidStateAxis->desiredRate = (float)((rate + 27) * rcCommand[axis]) / 16.0f; // 200dps to 1200dps max roll/pitch rate
        if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
            // calculate error angle and limit the angle to the max inclination
            // multiplication of rcCommand corresponds to changing the sticks scaling here
#ifdef GPS
            const float errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -((int)max_angle_inclination), max_angle_inclination)
                    - attitude.raw[axis] + angleTrim->raw[axis];
#else
            const float errorAngle = constrain(2 * rcCommand[axis], -((int)max_angle_inclination), max_angle_inclination)
                    - attitude.raw[axis] + angleTrim->raw[axis];
#endif
            if (FLIGHT_MODE(ANGLE_MODE)) {
                // ANGLE mode
                pidStateAxis->desiredRate = errorAngle * pidProfile->P8[PIDLEVEL] / 16.0f;
            } else {
                // HORIZON mode
                // mix in errorAngle to desiredRate to add a little auto-level feel.
                // horizonLevelStrength has been scaled to the stick input
                pidStateAxis->desiredRate += errorAngle * pidProfile->I8[PIDLEVEL] * horizonLevelStrength / 16.0f;
            }
        }
    }

}

void pidUpdateRcState(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig)
{
    const float horizonLevelStrength = FLIGHT_MODE(HORIZON_MODE) ? calcHorizonLevelStrength(pidProfile, rxConfig) : 1;
    for (int axis = 0; axis < 3; axis++) {
        pidUpdateRcStateAxis(axis, pidProfile, &pidState.stateAxis[axis], horizonLevelStrength,
                controlRateConfig, max_angle_inclination, angleTrim);
    }
}

static int16_t pidCalculateAxis(int axis, const pidProfile_t *pidProfile, pidStateAxis_t* pidStateAxis)
{
    const float rateError = pidStateAxis->desiredRate - pidStateAxis->gyroRate;

    // -----calculate P component
    pidStateAxis->PTerm = rateError * pidStateAxis->kP;
    // Constrain YAW by yaw_p_limit value if not servo driven, in that case servolimits apply
    if (axis == YAW && pidProfile->yaw_p_limit && motorCount >= 4) {
        pidStateAxis->PTerm = constrainf(pidStateAxis->PTerm, -pidProfile->yaw_p_limit, pidProfile->yaw_p_limit);
    }

    // -----calculate I component
    // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
    // I coefficient (I8) moved before integration to make limiting independent from PID settings
    pidStateAxis->ITerm = constrainf(pidStateAxis->ITerm, -PID_MAX_I, PID_MAX_I);
    // Anti windup protection
    if (IS_RC_MODE_ACTIVE(BOXAIRMODE)) {
        if (STATE(ANTI_WINDUP) || motorLimitReached) {
            pidStateAxis->ITerm = constrainf(pidStateAxis->ITerm, -pidStateAxis->ITermLimitf, pidStateAxis->ITermLimitf);
        } else {
            pidStateAxis->ITermLimitf = ABS(pidStateAxis->ITerm);
        }
    }
    // -----calculate D component
    if (pidProfile->dterm_average_count) {
        // Apply moving average
        pidStateAxis->DTerm = averageFilterApply(&pidStateAxis->DTermAverageFilterState);
    }
    pidStateAxis->DTerm *= pidStateAxis->kD;
    pidStateAxis->DTerm = constrainf(pidStateAxis->DTerm, -PID_MAX_D, PID_MAX_D);

#ifdef BLACKBOX
    axisPID_P[axis] = pidStateAxis->PTerm;
    axisPID_I[axis] = pidStateAxis->ITerm;
    axisPID_D[axis] = pidStateAxis->DTerm;
#endif
#ifdef GTUNE
    if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
        calculate_Gtune(axis);
    }
#endif
    GET_PID_LUX_FLOAT_CORE_LOCALS(axis);
    // -----calculate total PID output
    return lrintf(pidStateAxis->PTerm + pidStateAxis->ITerm + pidStateAxis->DTerm);
}

void pidCalculate(const pidProfile_t *pidProfile)
{
    for (int axis = 0; axis < 3; axis++) {
        axisPID[axis] = pidCalculateAxis(axis, pidProfile, &pidState.stateAxis[axis]);
    }
}

static void pidInitAxis(const pidProfile_t *pidProfile, pidStateAxis_t* pidStateAxis)
{
    const float *coeffs = nrd[pidProfile->dterm_differentiator];
    firFilterInit(&pidStateAxis->DTermFirFilterState, pidStateAxis->DTermFirFilterBuf, pidProfile->dterm_differentiator + 2, coeffs);

    averageFilterInit(&pidStateAxis->DTermAverageFilterState, pidStateAxis->DTermAverageFilterBuf, pidProfile->dterm_average_count);
}

void pidInit(const pidProfile_t *pidProfile)
{
    for (int axis = 0; axis < 3; axis++) {
        pidInitAxis(pidProfile, &pidState.stateAxis[axis]);
    }
}

void pidLuxFloat(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig)
{
    pidUpdateRcState(pidProfile, controlRateConfig, max_angle_inclination, angleTrim, rxConfig);
    pidUpdateGyroState(pidProfile);
    pidCalculate(pidProfile);
}

#endif

