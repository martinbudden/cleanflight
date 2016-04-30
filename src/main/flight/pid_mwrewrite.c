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

#define SRC_MAIN_FLIGHT_PID_MWREWRITE_C_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/parameter_group.h"
#include "config/runtime_config.h"
#include "config/config_unittest.h"

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
#include "flight/pid_mwrewrite.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/gtune.h"
#include "flight/mixer.h"


extern float dT;
extern uint8_t PIDweight[3];
extern int32_t lastITerm[3], ITermLimit[3];

extern filterStatePt1_t DTermPt1FilterState[3];

extern uint8_t motorCount;

#ifdef BLACKBOX
extern int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif

STATIC_UNIT_TESTED pidMwrState_t pidMwrState;


/* Noise-robust differentiator filter coefficients by Pavel Holoborodko, see
http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
N=2: h[0] = 1, h[-1] = -1
N=3: h[0] = 1/2, h[-1] = 0, h[-2] = -1/2
Precise on 1, x:
N=4: 1/4  (h[0] = 1, h[-1] = 1, h[-2] =-1, h[-3] =-1)
N=5: 1/8  (h[0] = 1, h[-1] = 2, h[-2] = 0, h[-3] =-2, h[-4] =-1)
N=5: 1/16 (h[0] = 1, h[-1] = 3, h[-2] = 2, h[-3] =-2, h[-4] =-3, h[-5] =-1)
N=7: 1/32 (h[0] = 1, h[-1] = 4, h[-2] = 5, h[-3] = 0, h[-4] =-5, h[-5] =-4, h[-6] =-1)
N=8: 1/64 (h[0] = 1, h[-1] = 5, h[-2] = 9, h[-3] = 5, h[-4] =-5, h[-5] =-9, h[-6] =-5, h[-7] =-1)
Precise on 1, x, x^2:
N=5: h[0] = 5/8, h[-1] = 1/4, h[-2] = -1, h[-3] = -1/4, h[-4] = 3/8
N=6: h[0] = 3/8, h[-1] = 1/2, h[-2] = -1/2, h[-3] = -3/4, h[-4] = 1/8, h[-5] = 1/4
N=7: h[0] = 7/32, h[-1] = 1/2, h[-2] = -1/32, h[-3] = -3/4, h[-4] = -11/32, h[-5] = 1/4, h[-6] = 5/32
N=8: h[0] = 1/8, h[-1] = 13/32, h[-2] = 1/4, h[-3] = -15/32, h[-4] = -5/8, h[-5] = -1/32, h[-6] = 1/4, h[-7] = 3/32
*/

static const int8_t nrdCoeffs2[] = { 64,-64 }; // filter length 2, simple differentiation
static const int8_t nrdCoeffs3[] = { 32,  0,-32 };
static const int8_t nrdCoeffs4[] = { 16, 16,-16,-16 };
#ifdef DTERM_PRECISE_ON_LINEAR
static const int8_t nrdCoeffs5[] = {  8, 16,  0,-16, -8 };
static const int8_t nrdCoeffs6[] = {  4, 12,  8, -8,-12, -4 };
static const int8_t nrdCoeffs7[] = {  2,  8, 10,  0,-10, -8, -2 };
static const int8_t nrdCoeffs8[] = {  1,  5,  9,  5, -5, -9, -5, -1 };
#else
static const int8_t nrdCoeffs5[] = { 40,  8,-64,-16, 24 };
static const int8_t nrdCoeffs6[] = { 24, 32,-32,-48,  8, 16 };
static const int8_t nrdCoeffs7[] = { 14, 32, -2,-48,-22, 16, 10 };
static const int8_t nrdCoeffs8[] = {  8, 26, 16,-30,-40, -2, 16,  6 };
#endif
static const int8_t *nrd[] = {nrdCoeffs2, nrdCoeffs3, nrdCoeffs4, nrdCoeffs5, nrdCoeffs6, nrdCoeffs7, nrdCoeffs8};


STATIC_UNIT_TESTED void pidUpdateGyroStateAxis(flight_dynamics_index_t axis, const pidProfile_t *pidProfile, pidMwrStateAxis_t* pidStateAxis)
{
    pidStateAxis->gyroRate = gyroADC[axis] / 4;
    const int32_t rateError = pidStateAxis->desiredRate - pidStateAxis->gyroRate;

    // -----calculate I component
    // There should be no division before accumulating the error to integrator, because the precision would be reduced.
    // Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator (Q19.13 format) is used.
    // Time correction (to avoid different I scaling for different builds based on average cycle time)
    // is normalized to cycle time = 2048 (2^11).
    pidStateAxis->ITerm += ((rateError * (uint16_t)targetLooptime) >> 11) * pidProfile->I8[axis];
    // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
    // I coefficient (I8) moved before integration to make limiting independent from PID settings
    pidStateAxis->ITerm = constrain(pidStateAxis->ITerm, (int32_t)(-PID_MAX_I << 13), (int32_t)(PID_MAX_I << 13));
    // Anti windup protection
    if (pidMwrState.antiWindupProtection) {
        pidStateAxis->ITerm = constrain(pidStateAxis->ITerm, -pidStateAxis->ITermLimit, pidStateAxis->ITermLimit);
    } else {
        pidStateAxis->ITermLimit = ABS(pidStateAxis->ITerm);
    }

    // -----calculate D component
    if (pidProfile->D8[axis] != 0) {
        // optimisation for when D8 is zero, often used by YAW axis
        // delta calculated from measurement
        // Calculate derivative using FIR filter
        firFilterInt32Update(&pidStateAxis->DTermFirFilterState, pidStateAxis->gyroRate);
        int32_t delta = -firFilterInt32Apply(&pidStateAxis->DTermFirFilterState);

        // Divide delta by targetLooptime to get differential (ie dr/dt)
        pidStateAxis->DTerm = (delta * ((uint16_t)0xFFFF / ((uint16_t)targetLooptime >> 4))) >> 5;
        if (pidProfile->dterm_lpf_hz) {
            // DTerm low pass filter
            pidStateAxis->DTerm = pt1FilterApply((float)pidStateAxis->DTerm, &DTermPt1FilterState[axis], pidProfile->dterm_lpf_hz, dT);
        }
        if (pidProfile->dterm_average_count) {
            // Apply moving average
            averageFilterInt32Update(&pidStateAxis->DTermAverageFilterState, pidStateAxis->DTerm);
        }
    }

}

void pidMwrUpdateGyroState(const pidProfile_t *pidProfile)
{
    for (int axis = 0; axis < 3; axis++) {
        pidUpdateGyroStateAxis(axis, pidProfile, &pidMwrState.stateAxis[axis]);
    }
}

static int8_t calcHorizonLevelStrength(const pidProfile_t *pidProfile)
{
    int horizonLevelStrength;
    // Figure out the most deflected stick position
    const int32_t stickPosAil = ABS(getRcStickDeflection(ROLL, rxConfig()->midrc));
    const int32_t stickPosEle = ABS(getRcStickDeflection(PITCH, rxConfig()->midrc));
    const int32_t mostDeflectedPos =  MAX(stickPosAil, stickPosEle);

    // Progressively turn off the horizon self level strength as the stick is banged over
    horizonLevelStrength = (500 - mostDeflectedPos) / 5;  // 100 at centre stick, 0 = max stick deflection

    // Using D8[PIDLEVEL] as a Sensitivity for Horizon.
    // 0 more level to 255 more rate. Default value of 100 seems to work fine.
    // For more rate mode increase D and slower flips and rolls will be possible
    horizonLevelStrength = constrain((10 * (horizonLevelStrength - 100) * (10 * pidProfile->D8[PIDLEVEL] / 80) / 100) + 100, 0, 100);
    return horizonLevelStrength;
}

static void pidUpdateRcStateAxis(int axis, const pidProfile_t *pidProfile, pidMwrStateAxis_t* pidStateAxis, const controlRateConfig_t *controlRateConfig)
{
    const uint8_t rate = controlRateConfig->rates[axis];

    // -----Get the desired angle rate depending on flight mode
    if (axis == FD_YAW) {
        // YAW is always gyro-controlled (MAG correction is applied to rcCommand)
        pidStateAxis->desiredRate = (((int32_t)(rate + 27) * rcCommand[YAW]) >> 5);
    } else {
        // control is GYRO based for ACRO and HORIZON - direct sticks control is applied to rate PID
        pidStateAxis->desiredRate = ((int32_t)(rate + 27) * rcCommand[axis]) >> 4;
        if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
            // calculate error angle and limit the angle to the max inclination
            // multiplication of rcCommand corresponds to changing the sticks scaling here
            const uint16_t max_angle_inclination = imuConfig()->max_angle_inclination;
            const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
#ifdef GPS
            const int32_t errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -((int)max_angle_inclination), max_angle_inclination)
                    - attitude.raw[axis] + angleTrim->raw[axis];
#else
            const int32_t errorAngle = constrain(2 * rcCommand[axis], -((int)max_angle_inclination), max_angle_inclination)
                    - attitude.raw[axis] + angleTrim->raw[axis];
#endif
            if (FLIGHT_MODE(ANGLE_MODE)) {
                // ANGLE mode
                pidStateAxis->desiredRate = (errorAngle * pidProfile->P8[PIDLEVEL]) >> 4;
            } else {
                // HORIZON mode
                // mix in errorAngle to desiredRate to add a little auto-level feel.
                // horizonLevelStrength has been scaled to the stick input
                const int horizonLevelStrength = calcHorizonLevelStrength(pidProfile);
                pidStateAxis->desiredRate += (errorAngle * pidProfile->I8[PIDLEVEL] * horizonLevelStrength / 100) >> 4;
            }
        }
    }
}

void pidMwrUpdateRcState(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig)
{
    pidMwrState.antiWindupProtection = false;
    if (IS_RC_MODE_ACTIVE(BOXAIRMODE)) {
        if (STATE(ANTI_WINDUP) || motorLimitReached) {
           pidMwrState.antiWindupProtection = true;
        }
    }
    for (int axis = 0; axis < 3; axis++) {
        pidUpdateRcStateAxis(axis, pidProfile, &pidMwrState.stateAxis[axis], controlRateConfig);
    }
}

static int16_t pidCalculateAxis(int axis, const pidProfile_t *pidProfile, pidMwrStateAxis_t* pidStateAxis)
{
    const int32_t rateError = pidStateAxis->desiredRate - pidStateAxis->gyroRate;

    // -----calculate P component
    int32_t PTerm = (rateError * pidProfile->P8[axis] * PIDweight[axis] / 100) >> 7;
    // Constrain YAW by yaw_p_limit value if not servo driven, in that case servolimits apply
    if (axis == YAW && pidProfile->yaw_p_limit && motorCount >= 4) {
        PTerm = constrain(PTerm, -pidProfile->yaw_p_limit, pidProfile->yaw_p_limit);
    }

    int ITerm = pidStateAxis->ITerm >> 13; // take integer part of Q19.13 value

    pidStateAxis->DTerm = (pidStateAxis->DTerm * pidProfile->D8[axis] * PIDweight[axis] / 100) >> 8;
    pidStateAxis->DTerm = constrain(pidStateAxis->DTerm, -PID_MAX_D, PID_MAX_D);

#ifdef BLACKBOX
    axisPID_P[axis] = PTerm;
    axisPID_I[axis] = ITerm;
    axisPID_D[axis] = pidStateAxis->DTerm;
#endif
#ifdef GTUNE
    if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
        calculate_Gtune(axis);
    }
#endif
    // -----calculate total PID output
    return PTerm + ITerm + pidStateAxis->DTerm;
}
void pidMwrCalculate(const pidProfile_t *pidProfile)
{
    for (int axis = 0; axis < 3; axis++) {
        axisPID[axis] = pidCalculateAxis(axis, pidProfile, &pidMwrState.stateAxis[axis]);
    }
}

void pidMultiWiiRewriteInit(const pidProfile_t *pidProfile)
{
    memset(&pidMwrState, 0, sizeof(pidMwrState));
    pidMwrState.kGyro = luxGyroScale * gyro.scale;
    const int8_t *coeffs = nrd[pidProfile->dterm_differentiator];
    for (int axis = 0; axis < 3; ++ axis) {
        pidMwrStateAxis_t *pidStateAxis = &pidMwrState.stateAxis[axis];
        pidStateAxis->ITerm = 0;
        pidStateAxis->ITermLimit = 0;
        firFilterInt32Init(&pidStateAxis->DTermFirFilterState, pidStateAxis->DTermFirFilterBuf, pidProfile->dterm_differentiator + 2, coeffs);
        averageFilterInt32Init(&pidStateAxis->DTermAverageFilterState, pidStateAxis->DTermAverageFilterBuf, pidProfile->dterm_average_count);
    }
}

void pidMultiWiiRewriteShim(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig)
{
    // note, for test code must update RC state before updating gyro state
    pidMwrUpdateRcState(pidProfile, controlRateConfig);
    pidMwrUpdateGyroState(pidProfile);
    pidMwrCalculate(pidProfile);
}

void pidMultiWiiRewrite(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig)
{
    UNUSED(max_angle_inclination);
    UNUSED(angleTrim);
    UNUSED(rxConfig);
    pidMultiWiiRewriteShim(pidProfile, controlRateConfig);
}

