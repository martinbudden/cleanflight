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

#include "bus_i2c.h"

#include "sensors/sonar.h"
#include "sonar_srf10.h"

// Technical specification is at: http://robot-electronics.co.uk/htm/srf10tech.htm

#define SRF10_MinimumFiringIntervalDefaultmS 65 // from spec sheet

#define SRF10_Address 0xE0
#define SRF10_AddressI2C (SRF10_Address>>1)

#define SRF10_READ_SoftwareRevision 0x00
#define SRF10_READ_Unused 0x01 // (reads 0x80)
#define SRF10_READ_RangeHighByte 0x02
#define SRF10_READ_RangeLowByte 0x03

#define SRF10_WRITE_CommandRegister 0x00
#define SRF10_WRITE_MaxGainRegister 0x01 //(default gain value 16)
#define SRF10_WRITE_RangeRegister 0x02 //(default range value 255)

#define SRF10_COMMAND_InitiateRangingInches 0x50
#define SRF10_COMMAND_InitiateRangingCm 0x51
#define SRF10_COMMAND_InitiateRangingMicroSeconds 0x53

#define SRF10_COMMAND_SetGain_40 0x00
#define SRF10_COMMAND_SetGain_100 0x06
#define SRF10_COMMAND_SetGain_200 0x09
#define SRF10_COMMAND_SetGain_300 0x0B
#define SRF10_COMMAND_SetGain_400 0x0D
#define SRF10_COMMAND_SetGain_500 0x0E
#define SRF10_COMMAND_SetGain_600 0x0F
#define SRF10_COMMAND_SetGain_700 0x10 // default and maximum
#define SRF10_COMMAND_SetGain_Max 0x10 // default

#define SRF10_COMMAND_ChangeAddress1 0xA0
#define SRF10_COMMAND_ChangeAddress2 0xAA
#define SRF10_COMMAND_ChangeAddress3 0xA5

// The range is ((Range Register x 43mm) + 43mm)
// so setting the Range Register to 0 (0x00) gives a maximum range of 43mm.
// Setting the Range Register to 1 (0x01) gives a maximum r
#define SFR10_RangeValue43mm 0
#define SFR10_RangeValue86mm 0
#define SFR10_RangeValue1m 24
#define SFR10_RangeValue4m 93
#define SFR10_RangeValue6m 139 // maximum range
#define SFR10_RangeValue11m 0xFF // exceeds actual maximum range

STATIC_UNIT_TESTED volatile int32_t srf10measurement = -1;
static uint32_t lastMeasurementAt = 0;

static bool i2c_srf10_send_command(uint8_t command)
{
#if defined(UNIT_TEST)
    UNUSED(command);
    return 0;
#else
    return i2cWrite(SRF10_AddressI2C, SRF10_WRITE_CommandRegister, command);
#endif
}

static bool i2c_srf10_send_byte(uint8_t i2cRegister, uint8_t val)
{
#if defined(UNIT_TEST)
    UNUSED(i2cRegister);
    UNUSED(val);
    return 0;
#else
    return i2cWrite(SRF10_AddressI2C, i2cRegister, val);
#endif
}

static uint8_t i2c_srf10_read_byte(uint8_t i2cRegister)
{
#if defined(UNIT_TEST)
    UNUSED(i2cRegister);
    return 0;
#else
    uint8_t byte;
    i2cRead(SRF10_AddressI2C, i2cRegister, 1, &byte);
    return byte;
#endif
}
void srf10_init(sonarRange_t *sonarRange)
{
    sonarRange->maxRangeCm = SRF10_MAX_RANGE_CM;
    sonarRange->detectionConeDeciDegrees = SRF10_DETECTION_CONE_DECIDEGREES;
    sonarRange->detectionConeExtendedDeciDegrees = SRF10_DETECTION_CONE_EXTENDED_DECIDEGREES;
    i2c_srf10_send_byte(SRF10_WRITE_MaxGainRegister, SRF10_COMMAND_SetGain_500);
    i2c_srf10_send_byte(SRF10_WRITE_RangeRegister, SFR10_RangeValue6m);
}

// This is called periodically from within mw.c executePeriodicTasks()
void srf10_start_reading(void)
{
    uint32_t now = millis();
    // see if there is a measurement outstanding, 0xFF is returned if no measurement
    uint8_t revision = i2c_srf10_read_byte(SRF10_READ_SoftwareRevision);
    if (revision != 0xFF) {
        // there is a measurement
        uint8_t lowByte = i2c_srf10_read_byte(SRF10_READ_RangeLowByte);
        uint8_t highByte = i2c_srf10_read_byte(SRF10_READ_RangeHighByte);
        srf10measurement =  highByte << 8 | lowByte;
        if (srf10measurement > SRF10_MAX_RANGE_CM)
            srf10measurement = SONAR_OUT_OF_RANGE;
    }

    if (now > lastMeasurementAt + SRF10_MinimumFiringIntervalDefaultmS) {
        // measurement repeat interval should be greater than SRF10_MinimumFiringIntervalDefaultmS
        // to avoid interference between connective measurements.
        lastMeasurementAt = now;
        i2c_srf10_send_command(SRF10_COMMAND_InitiateRangingCm);
    }
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t srf10_get_distance(void)
{
    return srf10measurement;
}

