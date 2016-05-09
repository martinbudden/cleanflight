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

struct pidProfile_s;
struct controlRateConfig_s;
typedef void (*pidUpdateGyroRateFuncPtr)(const struct pidProfile_s *pidProfile);
typedef void (*pidUpdateDesiredRateFuncPtr)(const struct pidProfile_s *pidProfile, const struct controlRateConfig_s *controlRateConfig,
        uint16_t max_angle_inclination, const union rollAndPitchTrims_u *angleTrim, const struct rxConfig_s *rxConfig);
typedef void (*pidCalculateFuncPtr)(const struct pidProfile_s *pidProfile);

void pidMarsInit(const struct pidProfile_s *pidProfile);
void pidMarsResetITerm(void);
void pidMarsUpdateGyroRate(const struct pidProfile_s *pidProfile);
void pidMarsUpdateDesiredRate(const struct pidProfile_s *pidProfile, const struct controlRateConfig_s *controlRateConfig,
        uint16_t max_angle_inclination, const union rollAndPitchTrims_u *angleTrim, const struct rxConfig_s *rxConfig);
void pidMarsCalculate(const struct pidProfile_s *pidProfile);

