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
extern uint8_t PIDweight[3];
extern float lastITermf[3], ITermLimitf[3];

extern biquad_t deltaBiquadFilterState[3];
extern filterStatePt1_t deltaPt1FilterState[3];
extern float DTermFirFilterState[3][PID_DTERM_FIR_MAX_LENGTH];

extern uint8_t motorCount;

#ifdef BLACKBOX
extern int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif

// constants to scale pidLuxFloat so output is same as pidMultiWiiRewrite
static const float luxPTermScale = 1.0f / 128;
static const float luxITermScale = 1000000.0f / 0x1000000;
static const float luxDTermScale = (0.000001f * (float)0xFFFF) / 512;
static const float luxGyroScale = 16.4f / 4; // the 16.4 is needed because mwrewrite does not scale according to the gyro model gyro.scale


// Filter coefficients, see http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
// N=2: h[0] = 1, h[-1] = -1
// N=3: h[0] = 1/2, h[-1] = 0, h[-2] = -1/2
// N=4: h[0] = 1/4, h[-1] = 1/4, h[-2] = -1/4, h[-3] = -1/4
// N=5: h[0] = 5/8, h[-1] = 1/4, h[-2] = -1, h[-3] = -1/4, h[-4] = 3/8
// N=6: h[0] = 3/8, h[-1] = 1/2, h[-2] = -1/2, h[-3] = -3/4, h[-4] = 1/8, h[-5] = 1/4
// N=7: h[0] = 7/32, h[-1] = 1/2, h[-2] = -1/32, h[-3] = -3/4, h[-4] = -11/32, h[-5] = 1/4, h[-6] = 5/32
// first coefficient is the divisor
static const int nrdCoefficents2[] = { 1, 1, -1};
static const int nrdCoefficents3[] = { 2, 1,  0, -1};
static const int nrdCoefficents4[] = { 4, 1,  1, -1, -1};
static const int nrdCoefficents5[] = { 8, 5,  2, -8, -2,  3};
static const int nrdCoefficents6[] = { 8, 3,  4, -4, -6,  1,  2};
static const int nrdCoefficents7[] = {32, 7, 16, -1,-24,-11,  8,  5};

static const int* nrd[] = {
        nrdCoefficents2,
        nrdCoefficents3,
        nrdCoefficents4,
        nrdCoefficents5,
        nrdCoefficents6,
        nrdCoefficents7,
};

float applyFirFilterIntCoeffs(float input, float firState[], uint8_t filterLength, const int coeffs[])
{
    memmove(&firState[1], &firState[0], (filterLength-1) * sizeof(float));
    firState[0] = input;
    float ret = 0.0f;
    for (int ii = 0; ii < filterLength; ++ii) {
        ret += coeffs[ii] * firState[ii];
    }
    return ret;
}

STATIC_UNIT_TESTED int16_t pidLuxFloatCore(int axis, const pidProfile_t *pidProfile, float gyroRate, float angleRate)
{
    static float lastRate[3][PID_LAST_RATE_COUNT];
    static float deltaState[3][PID_DELTA_MAX_SAMPLES];

    SET_PID_LUX_FLOAT_CORE_LOCALS(axis);

    const float rateError = angleRate - gyroRate;

    // -----calculate P component
    float PTerm = luxPTermScale * rateError * pidProfile->P8[axis] * PIDweight[axis] / 100;
    // Constrain YAW by yaw_p_limit value if not servo driven, in that case servolimits apply
    if (axis == YAW && pidProfile->yaw_p_limit && motorCount >= 4) {
        PTerm = constrainf(PTerm, -pidProfile->yaw_p_limit, pidProfile->yaw_p_limit);
    }

    // -----calculate I component
    float ITerm = lastITermf[axis] + luxITermScale * rateError * dT * pidProfile->I8[axis];
    // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
    // I coefficient (I8) moved before integration to make limiting independent from PID settings
    ITerm = constrainf(ITerm, -PID_MAX_I, PID_MAX_I);
    // Anti windup protection
    if (IS_RC_MODE_ACTIVE(BOXAIRMODE)) {
        if (STATE(ANTI_WINDUP) || motorLimitReached) {
            ITerm = constrainf(ITerm, -ITermLimitf[axis], ITermLimitf[axis]);
        } else {
            ITermLimitf[axis] = ABS(ITerm);
        }
    }
    lastITermf[axis] = ITerm;

    // -----calculate D component
    float DTerm;
    if (pidProfile->D8[axis] == 0) {
        // optimisation for when D8 is zero, often used by YAW axis
        DTerm = 0;
    } else {
        // Calculate derivative using FIR filter
        // FIR filter is noise-robust differentiator without time delay (one-sided or forward filters) by Pavel Holoborodko,
        // see http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
        const int* coeffs = nrd[pidProfile->dterm_noise_robust_differentiator];
        DTerm = applyFirFilterIntCoeffs(gyroRate, DTermFirFilterState[axis], pidProfile->dterm_noise_robust_differentiator + 2, coeffs + 1);
        DTerm = -DTerm / (coeffs[0] * dT);
        if (pidProfile->dterm_lpf_hz) {
            // DTerm delta low pass filter
#ifdef USE_PID_BIQUAD_FILTER
            DTerm = applyBiQuadFilter(DTerm, &deltaBiquadFilterState[axis]);
#else
            DTerm = filterApplyPt1(DTerm, &deltaPt1FilterState[axis], pidProfile->dterm_lpf_hz, dT);
#endif
        }
        if (pidProfile->dterm_average_count) {
            // Apply moving average
            DTerm = filterApplyAveragef(DTerm, pidProfile->dterm_average_count, deltaState[axis]);
        }
        DTerm = DTerm * luxDTermScale * pidProfile->D8[axis] * PIDweight[axis] / 100;
        DTerm = constrainf(DTerm, -PID_MAX_D, PID_MAX_D);
    }

#ifdef BLACKBOX
    axisPID_P[axis] = PTerm;
    axisPID_I[axis] = ITerm;
    axisPID_D[axis] = DTerm;
#endif
    GET_PID_LUX_FLOAT_CORE_LOCALS(axis);
    // -----calculate total PID output
    return lrintf(PTerm + ITerm + DTerm);
}

void pidLuxFloat(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig)
{
#ifdef USE_PID_BIQUAD_FILTER
    pidFilterIsSetCheck(pidProfile);
#endif

    float horizonLevelStrength;
    if (FLIGHT_MODE(HORIZON_MODE)) {
        // Figure out the most deflected stick position
        const int32_t stickPosAil = ABS(getRcStickDeflection(ROLL, rxConfig->midrc));
        const int32_t stickPosEle = ABS(getRcStickDeflection(PITCH, rxConfig->midrc));
        const int32_t mostDeflectedPos =  MAX(stickPosAil, stickPosEle);

        // Progressively turn off the horizon self level strength as the stick is banged over
        horizonLevelStrength = (float)(500 - mostDeflectedPos) / 500;  // 1 at centre stick, 0 = max stick deflection
        if(pidProfile->D8[PIDLEVEL] == 0){
            horizonLevelStrength = 0;
        } else {
            horizonLevelStrength = constrainf(((horizonLevelStrength - 1) * (100 / pidProfile->D8[PIDLEVEL])) + 1, 0, 1);
        }
    }

    // ----------PID controller----------
    for (int axis = 0; axis < 3; axis++) {
        const uint8_t rate = controlRateConfig->rates[axis];

        // -----Get the desired angle rate depending on flight mode
        float angleRate;
        if (axis == FD_YAW) {
            // YAW is always gyro-controlled (MAG correction is applied to rcCommand) 100dps to 1100dps max yaw rate
            angleRate = (float)((rate + 27) * rcCommand[YAW]) / 32.0f;
        } else {
            // control is GYRO based for ACRO and HORIZON - direct sticks control is applied to rate PID
            angleRate = (float)((rate + 27) * rcCommand[axis]) / 16.0f; // 200dps to 1200dps max roll/pitch rate
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
                    angleRate = errorAngle * pidProfile->P8[PIDLEVEL] / 16.0f;
                } else {
                    // HORIZON mode
                    // mix in errorAngle to desired angleRate to add a little auto-level feel.
                    // horizonLevelStrength has been scaled to the stick input
                    angleRate += errorAngle * pidProfile->I8[PIDLEVEL] * horizonLevelStrength / 16.0f;
                }
            }
        }

        // --------low-level gyro-based PID. ----------
        const float gyroRate = luxGyroScale * gyroADC[axis] * gyro.scale;
        axisPID[axis] = pidLuxFloatCore(axis, pidProfile, gyroRate, angleRate);
        //axisPID[axis] = constrain(axisPID[axis], -PID_LUX_FLOAT_MAX_PID, PID_LUX_FLOAT_MAX_PID);
#ifdef GTUNE
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            calculate_Gtune(axis);
        }
#endif
    }
}

#endif

