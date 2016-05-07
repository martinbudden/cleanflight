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
/*struct acc_s;
typedef void (*sensorAccInitFuncPtr)(struct acc_s *acc);                    // sensor init prototype
typedef struct acc_s {
    sensorAccInitFuncPtr init;                              // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
    uint16_t acc_1G;
    char revisionCode;                                      // a revision code for the sensor, if known
} acc_t;*/

struct pidProfile_s;
struct controlRateConfig_s;
typedef void (*pidUpdateGyroRateFuncPtr)(const struct pidProfile_s *pidProfile);
typedef void (*pidUpdateDesiredRateFuncPtr)(const struct pidProfile_s *pidProfile, const struct controlRateConfig_s *controlRateConfig);
typedef void (*pidCalculateFuncPtr)(const struct pidProfile_s *pidProfile);

void pidMarsInit(const struct pidProfile_s *pidProfile);
void pidMarsResetITerm(void);
void pidMarsUpdateGyroRate(const struct pidProfile_s *pidProfile);
void pidMarsUpdateDesiredRate(const struct pidProfile_s *pidProfile, const struct controlRateConfig_s *controlRateConfig);
void pidMarsCalculate(const struct pidProfile_s *pidProfile);

