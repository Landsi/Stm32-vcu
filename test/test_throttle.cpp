/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "my_fp.h"
#include "my_math.h"
#include "test_list.h"
#include "throttle.h"

using namespace std;

static void TestSetup()
{
   //percentage of deadzone
   Throttle::throtdead = 5;
   Throttle::potmin[0] = 100;
   Throttle::potmax[0] = 4000;
}

// TEMPERATURE DERATING
static void TestThrottleTemperateOverMaxThrottleTo0() {
   float temp = 60;
   float tempMax = 50;
   float finalSpnt = 100;
   ASSERT(Throttle::TemperatureDerate(temp, tempMax, finalSpnt) && finalSpnt == 0);
}

static void TestThrottleTemperateInDerateZoneThrottleTo50Percent() {
   float temp = 61;
   float tempMax = 60;
   float finalSpnt = 100;
   ASSERT(Throttle::TemperatureDerate(temp, tempMax, finalSpnt) && finalSpnt == 50);
}

static void TestThrottleUnderTemperateNoDeRate() {
   float temp = 30;
   float tempMax = 60;
   float finalSpnt = 100;
   ASSERT(Throttle::TemperatureDerate(temp, tempMax, finalSpnt) == false && finalSpnt == 100);
}

static void TestThrottleTemperateInDerateZoneThrottleButThrottleUnderLimit() {
   float temp = 61;
   float tempMax = 60;
   float finalSpnt = 49;
   ASSERT(Throttle::TemperatureDerate(temp, tempMax, finalSpnt) && finalSpnt == 49);
}

// CALC THROTTLE
static void TestCalcThrottleIs0WhenThrottleAndBrakePressed() {


   ASSERT(Throttle::CalcThrottle(3000, 0, true) == 0);
}

static void TestCalcThrottleIs0WhenNoThrottleAndBrakePressed() {

   ASSERT(Throttle::CalcThrottle(0, 0, true) == 0);
}

static void TestCalcThrottleIs0WhenInDeadZone() {
   //deadzone is first 5% of travel between 100 and 4000
   ASSERT(Throttle::CalcThrottle(295, 0, false) == 0);
}

static void TestCalcThrottleIsAbove0WhenJustOutOfDeadZone() {
   //deadzone is first 5% of travel between 100 and 4000
   ASSERT(Throttle::CalcThrottle(296, 0, false) > 0);
}

static void TestCalcThrottleIs100WhenMax() {
   //deadzone is first 5% of travel between 100 and 4000
   float throtVal = Throttle::CalcThrottle(4000, 0, false);
   ASSERT(throtVal ==  100);
}

// CHECK AND LIMIT RANGE
static void TestLimitAndCalcThrottleIs100WhenOverMax() {
   //deadzone is first 5% of travel between 100 and 4000
   int potVal = 4001;
   Throttle::CheckAndLimitRange(&potVal, 0);
   ASSERT(Throttle::CalcThrottle(potVal, 0, false) ==  100);
}

static void TestLimitThrottleRangeReturnsTrueInRangeWithSlack() {
   // POT_SLACK is defined in throttle.cpp - matching value of +/- 200 here. TODO: shared constant?
   int potVal = 4000 + 200;
   bool inRange = Throttle::CheckAndLimitRange(&potVal, 0);
   ASSERT(inRange);

   potVal = 100 - 200;
   inRange = Throttle::CheckAndLimitRange(&potVal, 0);
   ASSERT(inRange);

   potVal = 2386;
   inRange = Throttle::CheckAndLimitRange(&potVal, 0);
   ASSERT(inRange);
}

static void TestLimitThrottleRangeReturnsFalseOutsideRangeWithSlack() {
   // POT_SLACK is defined in throttle.cpp - matching value of +/- 200 here. TODO: shared constant?
   int potVal = 6000;
   bool inRange = Throttle::CheckAndLimitRange(&potVal, 0);
   ASSERT(!inRange);

   potVal = 4001 + 200;
   inRange = Throttle::CheckAndLimitRange(&potVal, 0);
   ASSERT(!inRange);

   potVal = 99 - 200;
   inRange = Throttle::CheckAndLimitRange(&potVal, 0);
   ASSERT(!inRange);

   potVal = -1234;
   inRange = Throttle::CheckAndLimitRange(&potVal, 0);
   ASSERT(!inRange);
}

static void TestLimitThrottleRangeLimitsToPotMaxWhenOver() {
   int potVal = 4001;
   Throttle::CheckAndLimitRange(&potVal, 0);
   ASSERT(potVal == 4000);
}

static void TestLimitThrottleRangeLimitsToPotMinWhenOverSlack() {
   // POT_SLACK is defined in throttle.cpp - matching value of +/- 200 here. TODO: shared constant?
   int potVal = 4001 + 200;
   Throttle::CheckAndLimitRange(&potVal, 0);
   ASSERT(potVal == 100);
}

static void TestLimitThrottleRangeLimitsToPotMinWhenUnder() {
   int potVal = 99;
   Throttle::CheckAndLimitRange(&potVal, 0);
   ASSERT(potVal == 100);
}

static void TestLimitThrottleRangeUnchangedWhenInRange() {
   int potVal = 1234;
   Throttle::CheckAndLimitRange(&potVal, 0);
   ASSERT(potVal == 1234);
}

// NORMALIZE THROTTLE
static void TestNormalizeThrottle0WhenIdxNotInBounds() {
   ASSERT(Throttle::NormalizeThrottle(1234, -1) == 0);
   ASSERT(Throttle::NormalizeThrottle(1234, -5) == 0);
   ASSERT(Throttle::NormalizeThrottle(1234, 2) == 0);
   ASSERT(Throttle::NormalizeThrottle(1234, 6) == 0);
}

static void TestNormalizeThrottle0WhenMinEqualsMax() {
   Throttle::potmin[0] = 1234;
   Throttle::potmax[0] = 1234;

   ASSERT(Throttle::NormalizeThrottle(1234, 0) == 0);
   
   Throttle::potmin[0] = 100;
   Throttle::potmax[0] = 4000;
}

static void TestNormalizeThrottle() {
   ASSERT(Throttle::NormalizeThrottle(100, 0) == 0);
   ASSERT(Throttle::NormalizeThrottle(1234, 0) - 31.64102564f < 0.00001f); // float imprecision workaround
   ASSERT(Throttle::NormalizeThrottle(4000, 0) == 100);
}

void ThrottleTest::RunTest()
{
   TestSetup();
   TestThrottleTemperateOverMaxThrottleTo0();
   TestThrottleTemperateInDerateZoneThrottleTo50Percent();
   TestThrottleUnderTemperateNoDeRate();
   TestThrottleTemperateInDerateZoneThrottleButThrottleUnderLimit();
   TestCalcThrottleIs0WhenThrottleAndBrakePressed();
   TestCalcThrottleIs0WhenNoThrottleAndBrakePressed();
   TestCalcThrottleIs0WhenInDeadZone();
   TestCalcThrottleIsAbove0WhenJustOutOfDeadZone();
   TestCalcThrottleIs100WhenMax();
   TestLimitAndCalcThrottleIs100WhenOverMax();
   TestLimitThrottleRangeReturnsTrueInRangeWithSlack();
   TestLimitThrottleRangeReturnsFalseOutsideRangeWithSlack();
   TestLimitThrottleRangeLimitsToPotMaxWhenOver();
   TestLimitThrottleRangeLimitsToPotMinWhenOverSlack();
   TestLimitThrottleRangeLimitsToPotMinWhenUnder();
   TestLimitThrottleRangeUnchangedWhenInRange();
   TestNormalizeThrottle0WhenIdxNotInBounds();
   TestNormalizeThrottle0WhenMinEqualsMax();
   TestNormalizeThrottle();
}
