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

#pragma once

typedef enum {
    GYRO_NONE = 0,
    GYRO_DEFAULT,
    GYRO_MPU6050,
    GYRO_L3G4200D,
    GYRO_MPU3050,
    GYRO_L3GD20,
    GYRO_MPU6000,
    GYRO_MPU6500,
    GYRO_FAKE
} gyroSensor_e;

extern gyro_t gyro;
extern sensor_align_e gyroAlign;

extern int32_t gyroADC[XYZ_AXIS_COUNT];

extern uint32_t targetLooptime;

typedef struct gyroConfig_s {
    uint8_t  gyroMovementCalibrationThreshold;  // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint8_t  gyro_lpf;                          // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
    uint8_t  gyroSync;                          // enable interrupt based loop
    uint8_t  gyroSyncDenominator;               // gyro sync Denominator
    uint16_t looptime;                          // loop time in microseconds
    uint16_t soft_gyro_lpf_hz;                  // software based gyro filter in hz
} gyroConfig_t;

PG_DECLARE(gyroConfig_t, gyroConfig);

void gyroInit(void);
void gyroSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void gyroUpdate(void);
bool isGyroCalibrationComplete(void);

