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
#include "config/config.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/nvic.h"

#include "sensors/sonar.h"
#include "drivers/sonar_hcsr04.h"

/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When triggered it sends out a series of 40KHz ultrasonic pulses and receives
 * echo from an object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */

#if defined(SONAR)

#define SONAR_GPIO GPIOB

typedef struct sonarHardware_s {
    sonarGPIOConfig_t GPIOConfig;
    uint32_t exti_line;
    uint8_t exti_pin_source;
    IRQn_Type exti_irqn;
} sonarHardware_t;

STATIC_UNIT_TESTED volatile int32_t hcsr04SonarPulseTravelTime = -1;
static uint32_t lastMeasurementAt;
static sonarHardware_t const *sonarHardware;

#if !defined(UNIT_TEST)
static void ECHO_EXTI_IRQHandler(void)
{
    static uint32_t timing_start;
    uint32_t timing_stop;

    if (digitalIn(sonarHardware->GPIOConfig.gpio, sonarHardware->GPIOConfig.echo_pin) != 0) {
        timing_start = micros();
    } else {
        timing_stop = micros();
        if (timing_stop > timing_start) {
            hcsr04SonarPulseTravelTime = timing_stop - timing_start;
        }
    }

    EXTI_ClearITPendingBit(sonarHardware->exti_line);
}

void EXTI0_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void EXTI1_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}
#endif

const sonarGPIOConfig_t *hcsr04_get_hardware_configuration(currentSensor_e currentSensor)
{
#if defined(NAZE) || defined(EUSTM32F103RC) || defined(PORT103R)
    static const sonarHardware_t const sonarPWM56 = {
        .GPIOConfig = {
            .gpio = SONAR_GPIO,
            .trigger_pin = Pin_8,   // PWM5 (PB8) - 5v tolerant
            .echo_pin = Pin_9       // PWM6 (PB9) - 5v tolerant
        },
        .exti_line = EXTI_Line9,
        .exti_pin_source = GPIO_PinSource9,
        .exti_irqn = EXTI9_5_IRQn
    };
    static const sonarHardware_t const sonarRC78 = {
        .GPIOConfig = {
            .gpio = SONAR_GPIO,
            .trigger_pin = Pin_0,   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
            .echo_pin = Pin_1       // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
        },
        .exti_line = EXTI_Line1,
        .exti_pin_source = GPIO_PinSource1,
        .exti_irqn = EXTI1_IRQn
    };
    // If we are using parallel PWM for our receiver or ADC current sensor, then use motor pins 5 and 6 for sonar, otherwise use rc pins 7 and 8
    if (feature(FEATURE_RX_PARALLEL_PWM ) || (feature(FEATURE_CURRENT_METER) && currentSensor == CURRENT_SENSOR_ADC) ) {
        sonarHardware = &sonarPWM56;
        return &sonarHardware->GPIOConfig;
    } else {
        sonarHardware = &sonarRC78;
        return &sonarHardware->GPIOConfig;
    }
#elif defined(OLIMEXINO)
    UNUSED(currentSensor);
    static const sonarHardware_t const sonarHardwareOLIMEXINO = {
        .GPIOConfig = {
            .gpio = SONAR_GPIO,
            .trigger_pin = Pin_0,   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
            .echo_pin = Pin_1       // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
        },
        .exti_line = EXTI_Line1,
        .exti_pin_source = GPIO_PinSource1,
        .exti_irqn = EXTI1_IRQn
    };
    sonarHardware = &sonarHardwareOLIMEXINO;
    return &sonarHardware->GPIOConfig;
#elif defined(CC3D)
    UNUSED(currentSensor);
    static const sonarHardware_t const sonarHardwareCC3D = {
        .GPIOConfig = {
           .gpio = SONAR_GPIO,
           .trigger_pin = Pin_5,   // (PB5)
           .echo_pin = Pin_0       // (PB0) - only 3.3v ( add a 1K Ohms resistor )
        },
        .exti_line = EXTI_Line0,
        .exti_pin_source = GPIO_PinSource0,
        .exti_irqn = EXTI0_IRQn
    };
    sonarHardware = &sonarHardwareCC3D;
    return &sonarHardware->GPIOConfig;
#elif defined(SPRACINGF3)
    UNUSED(currentSensor);
    static const sonarHardware_t const sonarHardwareSPRACINGF3 = {
        .GPIOConfig = {
            .gpio = SONAR_GPIO,
            .trigger_pin = Pin_0,   // RC_CH7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
            .echo_pin = Pin_1       // RC_CH8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
        },
        .exti_line = EXTI_Line1,
        .exti_pin_source = EXTI_PinSource1,
        .exti_irqn = EXTI1_IRQn
    };
    sonarHardware = &sonarHardwareSPRACINGF3;
    return &sonarHardware->GPIOConfig;
#elif defined(UNIT_TEST)
    UNUSED(currentSensor);
    sonarHardware = 0;
    return 0;
#else
#error Sonar not defined for target
#endif
}

void hcsr04_init(sonarRange_t *sonarRange, sonarFunctionPointers_t *sonarFunctionPointers)
{
    sonarRange->maxRangeCm = HCSR04_MAX_RANGE_CM;
    sonarRange->detectionConeDeciDegrees = HCSR04_DETECTION_CONE_DECIDEGREES;
    sonarRange->detectionConeExtendedDeciDegrees = HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES;
    sonarFunctionPointers->startReading = hcsr04_start_reading;
    sonarFunctionPointers->getDistance = hcsr04_get_distance;

#if !defined(UNIT_TEST)
    gpio_config_t gpio;
    EXTI_InitTypeDef EXTIInit;

#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef STM32F303xC
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

    // trigger pin
    gpio.pin = sonarHardware->GPIOConfig.trigger_pin;
    gpio.mode = Mode_Out_PP;
    gpio.speed = Speed_2MHz;
    gpioInit(sonarHardware->GPIOConfig.gpio, &gpio);

    // echo pin
    gpio.pin = sonarHardware->GPIOConfig.echo_pin;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(sonarHardware->GPIOConfig.gpio, &gpio);

#ifdef STM32F10X
    // setup external interrupt on echo pin
    gpioExtiLineConfig(GPIO_PortSourceGPIOB, sonarHardware->exti_pin_source);
#endif

#ifdef STM32F303xC
    gpioExtiLineConfig(EXTI_PortSourceGPIOB, sonarHardware->exti_pin_source);
#endif

    EXTI_ClearITPendingBit(sonarHardware->exti_line);

    EXTIInit.EXTI_Line = sonarHardware->exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = sonarHardware->exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SONAR_ECHO);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SONAR_ECHO);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    lastMeasurementAt = millis() - 60; // force 1st measurement in hcsr04_get_distance()
#else
    UNUSED(lastMeasurementAt);
#endif
}

/*
 * Start a range reading
 * Called periodically from within main process loop
 * Measurement reading is done asynchronously, using interrupt
 */
void hcsr04_start_reading(void)
{
#if !defined(UNIT_TEST)
    uint32_t now = millis();

    if (now < (lastMeasurementAt + 60)) {
        // the repeat interval of trig signal should be greater than 60ms
        // to avoid interference between consecutive measurements.
        return;
    }

    lastMeasurementAt = now;

    digitalHi(sonarHardware->GPIOConfig.gpio, sonarHardware->GPIOConfig.trigger_pin);
    //  The width of trig signal must be greater than 10us, according to device spec
    delayMicroseconds(11);
    digitalLo(sonarHardware->GPIOConfig.gpio, sonarHardware->GPIOConfig.trigger_pin);
#endif
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t hcsr04_get_distance(void)
{
    // The speed of sound is 340 m/s or approx. 29 microseconds per centimeter.
    // The ping travels out and back, so to find the distance of the
    // object we take half of the distance traveled.
    //
    // 340 m/s = 0.034 cm/microsecond = 29.41176471 *2 = 58.82352941 rounded to 59
    int32_t distance = hcsr04SonarPulseTravelTime / 59;
    if (distance > HCSR04_MAX_RANGE_CM)
        distance = SONAR_OUT_OF_RANGE;
    return distance;
}
#endif
