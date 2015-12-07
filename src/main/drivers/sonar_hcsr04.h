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

#include "platform.h"
#include "drivers/sonar.h"

#define HCSR04_MAX_RANGE_CM 400 // 4m, from HC-SR04 spec sheet
#define HCSR04_DETECTION_CONE_DECIDEGREES 300 // recommended cone angle 30 degrees, from HC-SR04 spec sheet
#define HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES 450 // in practice 45 degrees seems to work well

typedef struct sonarGPIOConfig_s {
    GPIO_TypeDef *gpio;
    const uint16_t trigger_pin;
    const uint16_t echo_pin;
} sonarGPIOConfig_t;

typedef struct sonarHardware_s {
    sonarGPIOConfig_t GPIOConfig;
    uint32_t exti_line;
    uint8_t exti_pin_source;
    IRQn_Type exti_irqn;
} sonarHardware_t;

void hcsr04_set_sonar_hardware(const sonarHardware_t *initialSonarHardware);
void hcsr04_init(sonarRange_t *sonarRange);
void hcsr04_start_reading(void);
int32_t hcsr04_get_distance(void);
