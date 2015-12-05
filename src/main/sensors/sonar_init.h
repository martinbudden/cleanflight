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

#include "sensors/battery.h"

typedef struct sonarGPIOConfig_s {
    GPIO_TypeDef *gpio;
    const uint16_t trigger_pin;
    const uint16_t echo_pin;
} sonarGPIOConfig_t;

const sonarGPIOConfig_t *sonarGetHardwareConfiguration(currentSensor_e currentSensor);
void sonarInit();
void sonarUpdate(void);
int32_t sonarRead(void);
