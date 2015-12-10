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

#include "drivers/system.h"
#include "drivers/bus_i2c.h"

#include "drivers/sonar.h"
#include "drivers/lidar_lite.h"

#if defined(SONAR)

#define LidarLite_MinimumFiringIntervalMs 40

#define LIDAR_LITE_MAX_RANGE_CM 4000 // 40 meters
#define LIDAR_LITE_DETECTION_CONE_DECIDEGREES 500
#define LIDAR_LITE_DETECTION_CONE_EXTENDED_DECIDEGREES 500

// Lidar Lite hardware constants
#define LIDAR_LITE_Address 0xC4
#define LIDAR_LITE_AddressI2C (LIDAR_LITE_Address>>1) // the I2C 7 bit address

// http://lidarlite.com/docs/v2/registers/
#define LIDAR_LITE_CommandRegister 0x00
#define LIDAR_LITE_COMMAND_Reset 0x00
#define LIDAR_LITE_COMMAND_InitiateRangingWithoutDcCorrection 0x03
#define LIDAR_LITE_COMMAND_InitiateRangingWithDcCorrection 0x04

#define LIDAR_LITE_HardwareVersionRegister 0x41

#define LIDAR_LITE_READ_RangeHighByte 0x0f
#define LIDAR_LITE_READ_RangeLowByte 0x10
#define LIDAR_LITE_READ_Range2Bytes 0x8f

STATIC_UNIT_TESTED volatile int32_t lidarLiteMeasurementCm = SONAR_OUT_OF_RANGE;

static bool i2c_lidar_lite_send_command(uint8_t command)
{
    return i2cWrite(LIDAR_LITE_AddressI2C, LIDAR_LITE_CommandRegister, command);
}

bool lidar_lite_detect()
{
    uint8_t hardwareVersion = 0;
    // read the hardware version register to detect if a Lidar Lite is present
    const bool ack = i2cRead(LIDAR_LITE_AddressI2C, LIDAR_LITE_HardwareVersionRegister, 1, &hardwareVersion);
    // hardware versions start at 0x01
    if (!ack || hardwareVersion == 0x00) {
        return false;
    }
    return true;
}

void lidar_lite_init(sonarRange_t *sonarRange)
{
    i2c_lidar_lite_send_command(LIDAR_LITE_COMMAND_Reset);
    sonarRange->maxRangeCm = LIDAR_LITE_MAX_RANGE_CM;
    sonarRange->detectionConeDeciDegrees = LIDAR_LITE_DETECTION_CONE_DECIDEGREES;
    sonarRange->detectionConeExtendedDeciDegrees = LIDAR_LITE_DETECTION_CONE_EXTENDED_DECIDEGREES;
}

/*
 * Start a range reading
 * Called periodically from within main process loop
 */
void lidar_lite_start_reading(void)
{
    static uint32_t timeOfLastMeasurementMs = 0;
    static int firingCount = 0;
    static bool fired = false;

    if (fired) {
        uint8_t distanceArray[2];
        const bool ack = i2cRead(LIDAR_LITE_AddressI2C, LIDAR_LITE_READ_Range2Bytes, 2, &distanceArray[0]);
        if (ack == true) {
            // there is a measurement, ranging has completed
            lidarLiteMeasurementCm =  (distanceArray[0] << 8) | distanceArray[1];
            if (lidarLiteMeasurementCm > LIDAR_LITE_MAX_RANGE_CM)
                lidarLiteMeasurementCm = SONAR_OUT_OF_RANGE;
        }
        fired = false;
    }

    const uint32_t timeNowMs = millis();
    if (timeNowMs > timeOfLastMeasurementMs + LidarLite_MinimumFiringIntervalMs) {
        // measurement repeat interval should be greater than SRF10_MinimumFiringIntervalFor600cmRangeMs
        // to avoid interference between connective measurements.
        timeOfLastMeasurementMs = timeNowMs;
        if (firingCount >= 100) {
            // PulsedLight recommend DC correction every 100 rangings
            i2c_lidar_lite_send_command(LIDAR_LITE_COMMAND_InitiateRangingWithDcCorrection);
            firingCount = 0;
        } else {
            i2c_lidar_lite_send_command(LIDAR_LITE_COMMAND_InitiateRangingWithoutDcCorrection);
        }
        ++firingCount;
        fired = true;
    }
}


/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t lidar_lite_get_distance(void)
{
    return lidarLiteMeasurementCm;
}
#endif
