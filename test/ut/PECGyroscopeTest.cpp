/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2020 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */


/**
 * Unit test of the PE::CGyroscope class.
 *
 * Code under test:
 *
 */

#include <fstream>
#include <iostream>
#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "PECGyroscope.h"
#include "PETypes.h"

class PECGyroscopeTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }

   bool Call_IsAllValueInRange( PE::CGyroscope& obj, const PE::TValue& v1, const PE::TValue& v2, const PE::TValue& min, const PE::TValue& max)
   {
      return obj.IsAllValueInRange(v1, v2, min, max);
   }
};


/**
 * checks IsAllValueInRange
 */
TEST_F(PECGyroscopeTest, test_IsAllValueInRange)
{
   PE::TValue HEADING_INTERVAL_1000MS           = 1.0;
   PE::TValue HEADING_INTERVAL_HYSTERESIS_100MS = 0.1;
   PE::TValue HEADING_MIN                       = 0.0;
   PE::TValue HEADING_MAX                       = 360.0;
   PE::TValue HEADING_ACCURACY_RATIO_5X         = 5;
   PE::TValue GYRO_INTERVAL_50MS                = 0.050;
   PE::TValue GYRO_INTERVAL_HYSTERESIS_5MS      = 0.005;
   PE::TValue GYRO_MIN                          = 0;
   PE::TValue GYRO_MAX                          = 4096;

   PE::CGyroscope gyro( HEADING_INTERVAL_1000MS,
                        HEADING_INTERVAL_HYSTERESIS_100MS,
                        HEADING_MIN,
                        HEADING_MAX,
                        HEADING_ACCURACY_RATIO_5X,
                        GYRO_INTERVAL_50MS,
                        GYRO_INTERVAL_HYSTERESIS_5MS,
                        GYRO_MIN,
                        GYRO_MAX);

   //test all range are correct
   EXPECT_TRUE(Call_IsAllValueInRange(gyro, -100, 100,-100,100));

   //test NaN
   EXPECT_FALSE(Call_IsAllValueInRange(gyro, std::numeric_limits<PE::TValue>::quiet_NaN(), 100,-100,100));
   EXPECT_FALSE(Call_IsAllValueInRange(gyro, -100, std::numeric_limits<PE::TValue>::quiet_NaN(),-100,100));
   EXPECT_FALSE(Call_IsAllValueInRange(gyro, -100, 100,std::numeric_limits<PE::TValue>::quiet_NaN(),100));
   EXPECT_FALSE(Call_IsAllValueInRange(gyro, -100, 100,-100,std::numeric_limits<PE::TValue>::quiet_NaN()));

   //test value 1 is out of range
   EXPECT_FALSE(Call_IsAllValueInRange(gyro, -101, 100,-100,100));
   EXPECT_FALSE(Call_IsAllValueInRange(gyro,  101, 100,-100,100));

   //test value 2 is out of range
   EXPECT_FALSE(Call_IsAllValueInRange(gyro, -100, 101,-100,100));
   EXPECT_FALSE(Call_IsAllValueInRange(gyro,  100, -101,-100,100));
}


/**
 * test Set/Get reference value
 */
TEST_F(PECGyroscopeTest, test_set_get_ref_value)
{
   PE::TValue HEADING_INTERVAL_100MS           = 0.100;
   PE::TValue HEADING_INTERVAL_HYSTERESIS_10MS = 0.010;
   PE::TValue HEADING_MIN                       = 0.0;
   PE::TValue HEADING_MAX                       = 360.0;
   PE::TValue HEADING_ACCURACY_RATIO_2X         = 2;
   PE::TValue GYRO_INTERVAL_50MS                = 0.050;
   PE::TValue GYRO_INTERVAL_HYSTERESIS_5MS      = 0.005;
   PE::TValue GYRO_MIN                          = 0;
   PE::TValue GYRO_MAX                          = 4096;

   PE::CGyroscope gyro( HEADING_INTERVAL_100MS,
                        HEADING_INTERVAL_HYSTERESIS_10MS,
                        HEADING_MIN,
                        HEADING_MAX,
                        HEADING_ACCURACY_RATIO_2X,
                        GYRO_INTERVAL_50MS,
                        GYRO_INTERVAL_HYSTERESIS_5MS,
                        GYRO_MIN,
                        GYRO_MAX);

   //init values have to be NaN
   EXPECT_TRUE( PE::isnan(gyro.GetRefValue()) );

   PE::TValue HEADING_090DEG                    = 090.0;
   PE::TValue HEADING_100DEG                    = 100.0;
   PE::TValue HEADING_140DEG                    = 140.0;
   PE::TValue HEADING_180DEG                    = 180.0;
   PE::TValue HEADING_381DEG                    = 381.0;
   PE::TValue HEADING_ACCURACY_10DEG            =  10.0;

   //set first heading 100deg -> reference value still NaN
   EXPECT_FALSE( gyro.SetRefValue(0.0, 0.100, HEADING_100DEG, HEADING_ACCURACY_10DEG));
   EXPECT_TRUE ( PE::isnan(gyro.GetRefValue()) );

   //set second heading 180deg -> reference value -800deg/s
   EXPECT_TRUE ( gyro.SetRefValue(0.100, 0.200, HEADING_180DEG, HEADING_ACCURACY_10DEG));
   EXPECT_NEAR ( -800.0, gyro.GetRefValue(), 0.1 );

   //set heading 100deg -> reference value +800deg/s
   EXPECT_TRUE ( gyro.SetRefValue(0.200, 0.300, HEADING_100DEG, HEADING_ACCURACY_10DEG));
   EXPECT_NEAR ( +800.0, gyro.GetRefValue(), 0.1 );

   /////////////////////////////////
   //set wrong heading 381deg -> NaN
   EXPECT_FALSE( gyro.SetRefValue(0.300, 0.400, HEADING_381DEG, HEADING_ACCURACY_10DEG));
   EXPECT_TRUE ( PE::isnan(gyro.GetRefValue()) );

   //set correct heading 100deg -> NaN because last heading was incorrect
   EXPECT_FALSE( gyro.SetRefValue(0.400, 0.500, HEADING_100DEG, HEADING_ACCURACY_10DEG));
   EXPECT_TRUE ( PE::isnan(gyro.GetRefValue()) );

   //set correct heading 180deg -> -800[deg/s] since last heading was correct
   EXPECT_TRUE ( gyro.SetRefValue(0.500, 0.600, HEADING_180DEG, HEADING_ACCURACY_10DEG));
   EXPECT_NEAR ( -800.0, gyro.GetRefValue(), 0.1 );

   //////////////////////////////////////////
   //set wrong heading interval 180deg -> NaN
   EXPECT_FALSE( gyro.SetRefValue(0.600, 0.711, HEADING_180DEG, HEADING_ACCURACY_10DEG));
   EXPECT_TRUE ( PE::isnan(gyro.GetRefValue()) );

   //set correct heading 100deg -> NaN because last heading was incorrect
   EXPECT_TRUE ( gyro.SetRefValue(0.600, 0.700, HEADING_100DEG, HEADING_ACCURACY_10DEG));
   EXPECT_NEAR ( +800.0, gyro.GetRefValue(), 0.1 );

   //////////////////////////////////////////
   //test speed accuracy more then heading (100(acc=10)-140(acc=10)) ratio=x2 -> NaN
   EXPECT_FALSE( gyro.SetRefValue(0.600, 0.700, HEADING_140DEG, HEADING_ACCURACY_10DEG));
   EXPECT_TRUE ( PE::isnan(gyro.GetRefValue()) );

   //set correct heading 90deg -> +500[deg/s] since last heading was correct
   EXPECT_TRUE ( gyro.SetRefValue(0.700, 0.800, HEADING_090DEG, HEADING_ACCURACY_10DEG));
   EXPECT_NEAR ( +500.0, gyro.GetRefValue(), 0.1 );
}


/**
 * test Set/Get sensor value
 */
TEST_F(PECGyroscopeTest, test_set_get_sen_value)
{
   PE::TValue HEADING_INTERVAL_100MS           = 0.100;
   PE::TValue HEADING_INTERVAL_HYSTERESIS_10MS = 0.010;
   PE::TValue HEADING_MIN                       = 0.0;
   PE::TValue HEADING_MAX                       = 360.0;
   PE::TValue HEADING_ACCURACY_RATIO_2X         = 2;
   PE::TValue GYRO_INTERVAL_50MS                = 0.050;
   PE::TValue GYRO_INTERVAL_HYSTERESIS_5MS      = 0.005;
   PE::TValue GYRO_MIN                          = 0;
   PE::TValue GYRO_MAX                          = 4096;

   PE::CGyroscope gyro( HEADING_INTERVAL_100MS,
                        HEADING_INTERVAL_HYSTERESIS_10MS,
                        HEADING_MIN,
                        HEADING_MAX,
                        HEADING_ACCURACY_RATIO_2X,
                        GYRO_INTERVAL_50MS,
                        GYRO_INTERVAL_HYSTERESIS_5MS,
                        GYRO_MIN,
                        GYRO_MAX);

   //init values have to be NaN
   EXPECT_TRUE( PE::isnan(gyro.GetSenValue()) );

   PE::TTimestamp HEAD_TS_0100MS     = 0.100;
   PE::TTimestamp HEAD_TS_0125MS     = 0.125;
   PE::TTimestamp GYRO_TS_0100MS     = 0.100;
   PE::TTimestamp GYRO_TS_0150MS     = 0.150;
   PE::TValue     GYRO_1000          = 1000.0;
   PE::TValue     GYRO_2000          = 2000.0;
   bool           GYRO_VALID         = true;
   bool           GYRO_INVALID       = false;

   //set first valid gyro value 1000 -> sensor value still NaN
   EXPECT_FALSE( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_1000, GYRO_VALID));
   EXPECT_TRUE ( PE::isnan(gyro.GetSenValue()) );

   //set second valid gyro value 1000 -> sensor value still NaN
   EXPECT_TRUE ( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_1000, GYRO_VALID));
   EXPECT_NEAR ( 1000.0, gyro.GetSenValue(), 0.1 );

   //////////////////////
   //set one gyro invalid
   EXPECT_FALSE( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_1000, GYRO_INVALID));
   EXPECT_TRUE ( PE::isnan(gyro.GetSenValue()) );

   //since last gyro was invalid sensor value will be also invalid
   EXPECT_FALSE( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_1000, GYRO_VALID));
   EXPECT_TRUE ( PE::isnan(gyro.GetSenValue()) );

   //now second valid gyro allow to have a new sensor value
   EXPECT_TRUE ( gyro.SetSenValue(HEAD_TS_0125MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_2000, GYRO_VALID));
   EXPECT_NEAR ( 1500.0, gyro.GetSenValue(), 0.1 );

   //////////////////////
   //set wrong gyro value
   EXPECT_FALSE( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_MAX+1, GYRO_VALID));
   EXPECT_TRUE ( PE::isnan(gyro.GetSenValue()) );

   //since last gyro value was out of valid range sensor value will be also invalid
   EXPECT_FALSE( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_1000, GYRO_VALID));
   EXPECT_TRUE ( PE::isnan(gyro.GetSenValue()) );

   //now correct gyro value allow to have a new sensor value
   EXPECT_TRUE ( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_2000, GYRO_VALID));
   EXPECT_NEAR ( 1000.0, gyro.GetSenValue(), 0.1 );

   /////////////////////////
   //set wrong gyro interval
   EXPECT_FALSE( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS+0.006, GYRO_1000, GYRO_VALID));
   EXPECT_TRUE ( PE::isnan(gyro.GetSenValue()) );

   //second time it provides sensor data since now interval is correct and last gyro was also correct
   EXPECT_TRUE ( gyro.SetSenValue(HEAD_TS_0125MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_2000, GYRO_VALID));
   EXPECT_NEAR ( 1500.0, gyro.GetSenValue(), 0.1 );
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
