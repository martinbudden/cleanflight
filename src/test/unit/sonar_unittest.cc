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

#include <stdint.h>

extern "C" {
    #include "build_config.h"
    #include "drivers/sonar_hcsr04.h"
    #include "drivers/sonar_srf10.h"
    #include "sensors/sonar.h"
    #include "sensors/sonar_init.h"
    extern int32_t hcsr04SonarPulseTravelTime;
    extern int32_t srf10measurement;
    extern int16_t sonarMaxTiltDeciDegrees;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(SonarUnittest, TestConstants)
{
    // SONAR_OUT_OF_RANGE must be negative
    EXPECT_LE(SONAR_OUT_OF_RANGE, 0);
    // Check against gross errors in max range constants
    EXPECT_GT(HCSR04_MAX_RANGE_CM, 100);
    EXPECT_LE(HCSR04_MAX_RANGE_CM, 1000);
    EXPECT_GT(SRF10_MAX_RANGE_CM, 100);
    EXPECT_LE(SRF10_MAX_RANGE_CM, 1000);
}

TEST(SonarUnittest, TestSonarInit_HCSR04)
{
    sonarGetHardwareConfiguration(SONAR_HCSR04, CURRENT_SENSOR_NONE);
    sonarInit();
    EXPECT_EQ(sonarMaxRangeCm, HCSR04_MAX_RANGE_CM);
    // Check against gross errors in max range values
    EXPECT_GE(sonarMaxAltWithTiltCm, 100);
    EXPECT_LE(sonarMaxAltWithTiltCm, sonarMaxRangeCm);
    EXPECT_GE(sonarCfAltCm, 100);
    EXPECT_LE(sonarCfAltCm, sonarMaxRangeCm);
    EXPECT_LE(sonarCfAltCm, sonarMaxAltWithTiltCm);
    // Check reasonable values for maximum tilt
    EXPECT_GE(sonarMaxTiltDeciDegrees, 0);
    EXPECT_LE(sonarMaxTiltDeciDegrees, 450);
}

TEST(SonarUnittest, TestSonarInit_SRF10)
{
    sonarGetHardwareConfiguration(SONAR_SRF10, CURRENT_SENSOR_NONE);
    sonarInit();
    EXPECT_EQ(sonarMaxRangeCm, SRF10_MAX_RANGE_CM);
    // Check against gross errors in max range values
    EXPECT_GE(sonarMaxAltWithTiltCm, 100);
    EXPECT_LE(sonarMaxAltWithTiltCm, sonarMaxRangeCm);
    EXPECT_GE(sonarCfAltCm, 100);
    EXPECT_LE(sonarCfAltCm, sonarMaxRangeCm);
    EXPECT_LE(sonarCfAltCm, sonarMaxAltWithTiltCm);
    // Check reasonable values for maximum tilt
    EXPECT_GE(sonarMaxTiltDeciDegrees, 0);
    EXPECT_LE(sonarMaxTiltDeciDegrees, 450);
}

TEST(SonarUnittest, TestDistance_HCSR04)
{
    // Check sonar pulse time converted correctly to cm
    const int echoMicroSecondsPerCm = 59;
    hcsr04SonarPulseTravelTime =  0;
    EXPECT_EQ(hcsr04_get_distance(), 0);

    hcsr04SonarPulseTravelTime =  echoMicroSecondsPerCm;
    EXPECT_EQ(hcsr04_get_distance(), 1);

    hcsr04SonarPulseTravelTime =  10 * echoMicroSecondsPerCm;
    EXPECT_EQ(hcsr04_get_distance(), 10);

    hcsr04SonarPulseTravelTime =  HCSR04_MAX_RANGE_CM * echoMicroSecondsPerCm;
    EXPECT_EQ(hcsr04_get_distance(), HCSR04_MAX_RANGE_CM);
}

TEST(SonarUnittest, TestDistance_SRF10)
{
    // initial distance should be out of range
    EXPECT_EQ(srf10_get_distance(), SONAR_OUT_OF_RANGE);

    // just check that get distance returns the measurement, no conversion
    srf10measurement =  0;
    EXPECT_EQ(srf10_get_distance(), 0);
    srf10measurement =  100;
    EXPECT_EQ(srf10_get_distance(), 100);
}

TEST(SonarUnittest, TestAltitude)
{
    for (int ii = SONAR_HCSR04; ii <= SONAR_SRF10; ++ii) {
        sonarGetHardwareConfiguration((sonarHardwareType_e)ii, CURRENT_SENSOR_NONE);
        sonarInit();
        // Check distance not modified if no tilt
        EXPECT_EQ(sonarCalculateAltitude(0, 0, 0), 0);
        EXPECT_EQ(sonarGetLatestAltitude(), 0);
        EXPECT_EQ(sonarCalculateAltitude(100, 0, 0), 100);
        EXPECT_EQ(sonarGetLatestAltitude(), 100);

        // Check that out of range is returned if tilt is too large
        EXPECT_EQ(sonarCalculateAltitude(0, sonarMaxTiltDeciDegrees+1, 0), SONAR_OUT_OF_RANGE);
        EXPECT_EQ(sonarGetLatestAltitude(), SONAR_OUT_OF_RANGE);

        // Check distance at various roll angles
        // distance 400, 5 degrees of roll
        EXPECT_EQ(sonarCalculateAltitude(400, 50, 0), 398);
        EXPECT_EQ(sonarGetLatestAltitude(), 398);
        // distance 400, 10 degrees of roll
        EXPECT_EQ(sonarCalculateAltitude(400, 100, 0), 393);
        EXPECT_EQ(sonarGetLatestAltitude(), 393);
        // distance 400, 15 degrees of roll, this corresponds to HC-SR04 specified max detection angle
        EXPECT_EQ(sonarCalculateAltitude(400, 150, 0), 386);
        EXPECT_EQ(sonarGetLatestAltitude(), 386);
        // distance 400, 20 degrees of roll
        EXPECT_EQ(sonarCalculateAltitude(400, 200, 0), 375);
        EXPECT_EQ(sonarGetLatestAltitude(), 375);
        // distance 400, 22.5 degrees of roll, this corresponds to HC-SR04 effective max detection angle
        EXPECT_EQ(sonarCalculateAltitude(400, 225, 0), 369);
        EXPECT_EQ(sonarGetLatestAltitude(), 369);
        // max range, max tilt
        EXPECT_EQ(sonarCalculateAltitude(sonarMaxRangeCm, sonarMaxTiltDeciDegrees, 0), sonarMaxAltWithTiltCm);
        EXPECT_EQ(sonarGetLatestAltitude(), sonarMaxAltWithTiltCm);
    }
}

typedef struct rollAndPitch_s {
    // absolute angle inclination in multiple of 0.1 degree (deci degrees): 180 degrees = 1800
    int16_t rollDeciDegrees;
    int16_t pitchDeciDegrees;
} rollAndPitch_t;

typedef struct inclinationAngleExpectations_s {
    rollAndPitch_t inclination;
    int16_t expected_angle;
} inclinationAngleExpectations_t;

TEST(SonarUnittest, TestCalculateTiltAngle)
{
    const int testCount = 9;
    inclinationAngleExpectations_t inclinationAngleExpectations[testCount] = {
        { { 0,  0}, 0},
        { { 1,  0}, 1},
        { { 0,  1}, 1},
        { { 0, -1}, 1},
        { {-1,  0}, 1},
        { {-1, -2}, 2},
        { {-2, -1}, 2},
        { { 1,  2}, 2},
        { { 2,  1}, 2}
    };

    rollAndPitch_t inclination = {0, 0};
    int tilt_angle = sonarCalculateTiltAngle(inclination.rollDeciDegrees, inclination.pitchDeciDegrees);
    EXPECT_EQ(tilt_angle, 0);

    for (int i = 0; i < testCount; i++) {
        inclinationAngleExpectations_t *expectation = &inclinationAngleExpectations[i];
        int result = sonarCalculateTiltAngle(expectation->inclination.rollDeciDegrees, expectation->inclination.pitchDeciDegrees);
        EXPECT_EQ(expectation->expected_angle, result);
    }
}


// STUBS
extern "C" {
void sensorsSet(uint32_t mask) {UNUSED(mask);}
uint32_t millis(void) {return 0;}
bool i2cWrite(uint8_t addr_, uint8_t reg, uint8_t data) {UNUSED(addr_); UNUSED(reg); UNUSED(data); return false;}
bool i2cRead(uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf) {UNUSED(addr_); UNUSED(reg);UNUSED(len); UNUSED(buf); return false;}
}

