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
#include <stdbool.h>

#include <limits.h>

#include <math.h>

extern "C" {
    #include "common/filter.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(FilterUnittest, TestFilterApplyAverageInt)
{
#define VALUE_COUNT 4
    averageFilterInt32_t filterState;
    int32_t valueState[VALUE_COUNT];

    averageFilterInt32Init(&filterState, valueState, VALUE_COUNT);

    averageFilterInt32Update(&filterState, 8);
    int average = averageFilterInt32Apply(&filterState);
    EXPECT_EQ(2, average); // 8/4
    EXPECT_EQ(8, valueState[0]);
    EXPECT_EQ(0, valueState[1]);

    averageFilterInt32Update(&filterState, 16);
    average = averageFilterInt32Apply(&filterState);
    EXPECT_EQ(6, average); // (8+16)/4
    EXPECT_EQ(16, valueState[0]);
    EXPECT_EQ(8, valueState[1]);


    averageFilterInt32Update(&filterState, 4);
    average = averageFilterInt32Apply(&filterState);
    EXPECT_EQ(7, average); // (8+16+4)/4
    EXPECT_EQ(4, valueState[0]);
    EXPECT_EQ(16, valueState[1]);
    EXPECT_EQ(8, valueState[2]);

    averageFilterInt32Update(&filterState, -12);
    average = averageFilterInt32Apply(&filterState);
    EXPECT_EQ(4, average); // (8+16+4-12)/4
    EXPECT_EQ(-12, valueState[0]);
    EXPECT_EQ(4, valueState[1]);
    EXPECT_EQ(16, valueState[2]);
    EXPECT_EQ(8, valueState[3]);

    averageFilterInt32Update(&filterState, 48);
    average = averageFilterInt32Apply(&filterState);
    EXPECT_EQ(14, average); // (16+4-12+48)/4
    EXPECT_EQ(48, valueState[0]);
    EXPECT_EQ(-12, valueState[1]);
    EXPECT_EQ(4, valueState[2]);
    EXPECT_EQ(16, valueState[3]);

    averageFilterInt32Update(&filterState, 4);
    average = averageFilterInt32Apply(&filterState);
    EXPECT_EQ(11, average); // (4-12+48+4)/4
    EXPECT_EQ(4, valueState[0]);
    EXPECT_EQ(48, valueState[1]);
    EXPECT_EQ(-12, valueState[2]);
    EXPECT_EQ(4, valueState[3]);
}

TEST(FilterUnittest, TestFilterApplyAverageInt2)
{
#define VALUE_COUNT 4
    averageFilterInt32_t filterState;
    int32_t valueState[3][VALUE_COUNT];

    averageFilterInt32Init(&filterState, valueState[0], VALUE_COUNT);

    averageFilterInt32Update(&filterState, 8);
    int average = averageFilterInt32Apply(&filterState);
    EXPECT_EQ(2, average); // 8/4
    EXPECT_EQ(8, valueState[0][0]);
    EXPECT_EQ(0, valueState[0][1]);

    averageFilterInt32Update(&filterState, 16);
    average = averageFilterInt32Apply(&filterState);
    EXPECT_EQ(6, average); // (8+16)/4
    EXPECT_EQ(16, valueState[0][0]);
    EXPECT_EQ(8, valueState[0][1]);

}

TEST(FilterUnittest, TestFilterApplyAverage)
{
#define VALUE_COUNT 4
    averageFilter_t filterState;
    float valueState[VALUE_COUNT];

    averageFilterInit(&filterState, valueState, VALUE_COUNT);

    averageFilterUpdate(&filterState, 8);
    float average = averageFilterApply(&filterState);
    EXPECT_EQ(2, average); // 8/4
    EXPECT_EQ(8, valueState[0]);
    EXPECT_EQ(0, valueState[1]);

    averageFilterUpdate(&filterState, 16);
    average = averageFilterApply(&filterState);
    EXPECT_EQ(6, average); // (8+16)/4
    EXPECT_EQ(16, valueState[0]);
    EXPECT_EQ(8, valueState[1]);


    averageFilterUpdate(&filterState, 4);
    average = averageFilterApply(&filterState);
    EXPECT_EQ(7, average); // (8+16+4)/4
    EXPECT_EQ(4, valueState[0]);
    EXPECT_EQ(16, valueState[1]);
    EXPECT_EQ(8, valueState[2]);

    averageFilterUpdate(&filterState, -12);
    average = averageFilterApply(&filterState);
    EXPECT_EQ(4, average); // (8+16+4-12)/4
    EXPECT_EQ(-12, valueState[0]);
    EXPECT_EQ(4, valueState[1]);
    EXPECT_EQ(16, valueState[2]);
    EXPECT_EQ(8, valueState[3]);

    averageFilterUpdate(&filterState, 48);
    average = averageFilterApply(&filterState);
    EXPECT_EQ(14, average); // (16+4-12+48)/4
    EXPECT_EQ(48, valueState[0]);
    EXPECT_EQ(-12, valueState[1]);
    EXPECT_EQ(4, valueState[2]);
    EXPECT_EQ(16, valueState[3]);

    averageFilterUpdate(&filterState, 4);
    average = averageFilterApply(&filterState);
    EXPECT_EQ(11, average); // (4-12+48+4)/4
    EXPECT_EQ(4, valueState[0]);
    EXPECT_EQ(48, valueState[1]);
    EXPECT_EQ(-12, valueState[2]);
    EXPECT_EQ(4, valueState[3]);
}

