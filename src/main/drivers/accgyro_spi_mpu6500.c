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

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "exti.h"
#include "io.h"
#include "bus_spi.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_mpu6500.h"
#include "accgyro_spi_mpu6500.h"

#define DISABLE_MPU6500       IOHi(spiCsnPin)
#define ENABLE_MPU6500        IOLo(spiCsnPin)

#define BIT_SLEEP                   0x40

bool mpu6500SpiWriteRegister(IO_t spiCsnPin, uint8_t reg, uint8_t data)
{
    ENABLE_MPU6500;
    spiTransferByte(MPU6500_SPI_INSTANCE, reg);
    spiTransferByte(MPU6500_SPI_INSTANCE, data);
    DISABLE_MPU6500;

    return true;
}

bool mpu6500SpiReadRegister(IO_t spiCsnPin, uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_MPU6500;
    spiTransferByte(MPU6500_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(MPU6500_SPI_INSTANCE, data, NULL, length);
    DISABLE_MPU6500;

    return true;
}

static void mpu6500SpiInit(IO_t spiCsnPin)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }

    IOInit(spiCsnPin, OWNER_MPU_CS, 0);
    IOConfigGPIO(spiCsnPin, SPI_IO_CS_CFG);

    spiSetDivisor(MPU6500_SPI_INSTANCE, SPI_CLOCK_FAST);

    hardwareInitialised = true;
}

static uint8_t mpuDetected = MPU_NONE;
uint8_t mpu6500SpiDetect(IO_t mpuCsPin)
{
<<<<<<< HEAD
    uint8_t tmp;

    mpu6500SpiInit();

    mpu6500ReadRegister(MPU_RA_WHO_AM_I, 1, &tmp);

    switch (tmp) {
    case MPU6500_WHO_AM_I_CONST:
        mpuDetected = MPU_65xx_SPI;
        break;
    case MPU9250_WHO_AM_I_CONST:
    case MPU9255_WHO_AM_I_CONST:
        mpuDetected = MPU_9250_SPI;
        break;
    case ICM20608G_WHO_AM_I_CONST:
        mpuDetected = ICM_20608_SPI;
        break;
    case ICM20602_WHO_AM_I_CONST:
        mpuDetected = ICM_20602_SPI;
        break;
    default:
        mpuDetected = MPU_NONE;
=======
    mpu6500SpiInit(mpuCsPin);

    uint8_t tmp;
    uint8_t detectRetries = 0;
    delayMicroseconds(15);
    do {
        mpu6500SpiReadRegister(mpuCsPin, MPU_RA_PWR_MGMT_1, 1, &tmp);
        detectRetries++;
    } while (tmp != BIT_SLEEP && detectRetries < 30);

    if (tmp == BIT_SLEEP) {
        mpu6500SpiReadRegister(mpuCsPin, MPU_RA_WHO_AM_I, 1, &tmp);
        delayMicroseconds(15);
        switch (tmp) {
        case MPU6500_WHO_AM_I_CONST:
            mpuDetected = MPU_65xx_SPI;
            break;
        case MPU9250_WHO_AM_I_CONST:
        case MPU9255_WHO_AM_I_CONST:
            mpuDetected = MPU_9250_SPI;
            break;
        case ICM20608G_WHO_AM_I_CONST:
            mpuDetected = ICM_20608_SPI;
            break;
        case ICM20602_WHO_AM_I_CONST:
            mpuDetected = ICM_20602_SPI;
            break;
        default:
            mpuDetected = MPU_NONE;
        }
>>>>>>> Added runtime setting of gyro SPI pin
    }
    return mpuDetected;
}

void mpu6500SpiAccInit(accDev_t *acc)
{
    mpu6500AccInit(acc);
}

void mpu6500SpiGyroInit(gyroDev_t *gyro)
{
    spiSetDivisor(MPU6500_SPI_INSTANCE, SPI_CLOCK_SLOW);
    delayMicroseconds(1);

    mpu6500GyroInit(gyro);

    // Disable Primary I2C Interface
    mpu6500SpiWriteRegister(gyro->spiCsnPin, MPU_RA_USER_CTRL, MPU6500_BIT_I2C_IF_DIS);
    delay(100);

    spiSetDivisor(MPU6500_SPI_INSTANCE, SPI_CLOCK_FAST);
    delayMicroseconds(1);
}

bool mpu6500SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != mpuDetected || !mpuDetected) {
        return false;
    }

    acc->init = mpu6500SpiAccInit;
    acc->read = mpuAccRead;

    return true;
}

bool mpu6500SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != mpuDetected || !mpuDetected) {
        return false;
    }

    gyro->init = mpu6500SpiGyroInit;
    gyro->read = mpuGyroRead;
    gyro->intStatus = mpuCheckDataReady;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}
