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

#include "platform.h"
#include "build_config.h"

#include "common/maths.h"

#include "config/runtime_config.h"

#include "drivers/gpio.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/sonar_srf10.h"

#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/sonar.h"

// Sonar measurements are in cm, a value of SONAR_OUT_OF_RANGE indicates sonar is not in range.
// Inclination is adjusted by imu

#ifdef SONAR

int16_t sonarMaxRangeCm;
int16_t sonarMaxAltWithTiltCm;
int16_t sonarCfAltCm; // Complimentary Filter altitude
STATIC_UNIT_TESTED int16_t sonarMaxTiltDeciDegrees;
static sonarHardwareType_e sonarHardwareType;
static sonarFunctionPointers_t sonarFunctionPointers;


static int32_t calculatedAltitude;

/*
 * Setup the hardware configuration for the designated hardware type.
 * NOTE: sonarInit() must be subsequently called before using any of the sonar functions.
 */
const sonarGPIOConfig_t *sonarGetHardwareConfiguration(sonarHardwareType_e sonarHardware, currentSensor_e currentSensor)
{
    const sonarGPIOConfig_t *GPIOConfig;

    sonarHardwareType = sonarHardware;
    switch (sonarHardwareType) {
    case SONAR_HCSR04:
        GPIOConfig = hcsr04_get_hardware_configuration(currentSensor);
        break;
    case SONAR_SRF10:
        GPIOConfig = srf10_get_hardware_configuration();
        break;
    }
    return GPIOConfig;
}


#define coefX2 1.52309E-06f // (PI/1800)^2/2, coefficient of x^2 in Taylor expansion of cosDeciDegrees(x)
#define cosDeciDegrees(x) (1.0f - x * x * coefX2)


void sonarInit()
{
    sonarRange_t sonarRange;

    switch (sonarHardwareType) {
    case SONAR_HCSR04:
        hcsr04_init(&sonarRange, &sonarFunctionPointers);
        break;
    case SONAR_SRF10:
        srf10_init(&sonarRange, &sonarFunctionPointers);
        break;
    }
    sensorsSet(SENSOR_SONAR);
    sonarMaxRangeCm = sonarRange.maxRangeCm;
    sonarCfAltCm = sonarMaxRangeCm / 2;
    sonarMaxTiltDeciDegrees =  sonarRange.detectionConeExtendedDeciDegrees / 2;
    sonarMaxAltWithTiltCm = sonarMaxRangeCm * cosDeciDegrees(sonarMaxTiltDeciDegrees);
    calculatedAltitude = SONAR_OUT_OF_RANGE;
}

/*
 * This is called periodically from within main process loop
 */
void sonarUpdate(void)
{
    sonarFunctionPointers.startReading();
}

/**
 * Get the last distance measured by the sonar in centimeters. When the ground is too far away, SONAR_OUT_OF_RANGE is returned.
 */
int32_t sonarRead(void)
{
    return sonarFunctionPointers.getDistance();
}

/*
* This (poorly named) function merely returns whichever is higher, roll inclination or pitch inclination.
* //TODO: Fix this up. We could either actually return the angle between 'down' and the normal of the craft
* (my best interpretation of scalar 'tiltAngle') or rename the function.
*/
int16_t sonarCalculateTiltAngle(int16_t rollDeciDegrees, int16_t pitchDeciDegrees)
{
    return MAX(ABS(rollDeciDegrees), ABS(pitchDeciDegrees));
}

/**
 * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
 * the altitude. Returns the computed altitude in centimeters.
 *
 * When the ground is too far away or the tilt is too large, SONAR_OUT_OF_RANGE is returned.
 */
int32_t sonarCalculateAltitude(int32_t sonarDistance, int16_t rollDeciDegrees, int16_t pitchDeciDegrees)
{
    int16_t tiltAngle = sonarCalculateTiltAngle(rollDeciDegrees, pitchDeciDegrees);
    // calculate sonar altitude only if the ground is in the sonar cone
    if (tiltAngle > sonarMaxTiltDeciDegrees)
        calculatedAltitude = SONAR_OUT_OF_RANGE;
    else if (sonarDistance == SONAR_OUT_OF_RANGE)
        calculatedAltitude = SONAR_OUT_OF_RANGE;
    else
        // altitude = distance * cos(tiltAngle), use approximation
        calculatedAltitude = sonarDistance * cosDeciDegrees(tiltAngle);
    return calculatedAltitude;
}

/**
 * Get the latest altitude that was computed by a call to sonarCalculateAltitude(), or SONAR_OUT_OF_RANGE if sonarCalculateAltitude
 * has never been called.
 */
int32_t sonarGetLatestAltitude(void)
{
    return calculatedAltitude;
}

#endif
