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
};


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

   //set first correct heading 100deg -> reference value still NaN
   EXPECT_TRUE ( gyro.SetRefValue(0.0, 0.100, HEADING_100DEG, HEADING_ACCURACY_10DEG));
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
   EXPECT_TRUE ( gyro.SetRefValue(0.400, 0.500, HEADING_100DEG, HEADING_ACCURACY_10DEG));
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
   EXPECT_TRUE ( PE::isnan(gyro.GetRefValue()) );

   //////////////////////////////////////////
   //set correct heading but speed accuracy more then heading (100(acc=10)-140(acc=10)) ratio=x2 -> NaN
   EXPECT_TRUE ( gyro.SetRefValue(0.600, 0.700, HEADING_140DEG, HEADING_ACCURACY_10DEG));
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

   //set first correct gyro value 1000 -> sensor value still NaN
   EXPECT_TRUE ( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_1000, GYRO_VALID));
   EXPECT_TRUE ( PE::isnan(gyro.GetSenValue()) );

   //set second correct gyro value 1000 -> sensor value  1000.0
   EXPECT_TRUE ( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_1000, GYRO_VALID));
   EXPECT_NEAR ( 1000.0, gyro.GetSenValue(), 0.1 );

   //////////////////////
   //set one gyro invalid -> sensor value still NaN
   EXPECT_FALSE( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_1000, GYRO_INVALID));
   EXPECT_TRUE ( PE::isnan(gyro.GetSenValue()) );

   //set correct gyro since last gyro was invalid  -> sensor value still NaN
   EXPECT_TRUE ( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_1000, GYRO_VALID));
   EXPECT_TRUE ( PE::isnan(gyro.GetSenValue()) );

   //now second correct gyro allow to have a new sensor value
   EXPECT_TRUE ( gyro.SetSenValue(HEAD_TS_0125MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_2000, GYRO_VALID));
   EXPECT_NEAR ( 1500.0, gyro.GetSenValue(), 0.1 );

   //////////////////////
   //set wrong gyro value
   EXPECT_FALSE( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_MAX+1, GYRO_VALID));
   EXPECT_TRUE ( PE::isnan(gyro.GetSenValue()) );

   //set correct gyro since last gyro was invalid  -> sensor value still NaN
   EXPECT_TRUE ( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_1000, GYRO_VALID));
   EXPECT_TRUE ( PE::isnan(gyro.GetSenValue()) );

   //now correct gyro value allow to have a new sensor value
   EXPECT_TRUE ( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_2000, GYRO_VALID));
   EXPECT_NEAR ( 1000.0, gyro.GetSenValue(), 0.1 );

   /////////////////////////
   //set wrong gyro interval
   EXPECT_FALSE( gyro.SetSenValue(HEAD_TS_0100MS, GYRO_TS_0100MS, GYRO_TS_0150MS+0.006, GYRO_1000, GYRO_VALID));
   EXPECT_TRUE ( PE::isnan(gyro.GetSenValue()) );

   //set correct gyro and interval is correct  -> sensor value still NaN
   EXPECT_TRUE ( gyro.SetSenValue(HEAD_TS_0125MS, GYRO_TS_0100MS, GYRO_TS_0150MS, GYRO_2000, GYRO_VALID));
   EXPECT_TRUE ( PE::isnan(gyro.GetSenValue()) );
}


/**
 * Test simple circle driving
 */
TEST_F(PECGyroscopeTest, test_simple_circle_driving)
{
   PE::TValue HEADING_INTERVAL_1S               = 1.000;
   PE::TValue HEADING_INTERVAL_HYSTERESIS_100MS = 0.100;
   PE::TValue HEADING_MIN                       = 0.0;
   PE::TValue HEADING_MAX                       = 360.0;
   PE::TValue HEADING_ACCURACY_RATIO_2X         = 2;
   PE::TValue GYRO_INTERVAL_500MS               = 0.500;
   PE::TValue GYRO_INTERVAL_HYSTERESIS_25MS     = 0.025;
   PE::TValue GYRO_MIN                          = 0;
   PE::TValue GYRO_MAX                          = 4096;

   PE::TValue RAW_GYRO_BASE                     = 2048;
   PE::TValue HEADING_DEVIATION_DEG             = 0.1;

   PE::CGyroscope gyro( HEADING_INTERVAL_1S,
                        HEADING_INTERVAL_HYSTERESIS_100MS,
                        HEADING_MIN,
                        HEADING_MAX,
                        HEADING_ACCURACY_RATIO_2X,
                        GYRO_INTERVAL_500MS,
                        GYRO_INTERVAL_HYSTERESIS_25MS,
                        GYRO_MIN,
                        GYRO_MAX);
   EXPECT_FALSE( gyro.AddHeading( 1.000,                   0, HEADING_DEVIATION_DEG)); //first adding is always false
   EXPECT_FALSE( gyro.AddGyro   ( 1.100,       RAW_GYRO_BASE, true)); //first adding is always false
   EXPECT_TRUE ( gyro.AddGyro   ( 1.600, RAW_GYRO_BASE +  50, true));
   EXPECT_TRUE ( gyro.AddHeading( 2.000,                   5, HEADING_DEVIATION_DEG));
   EXPECT_TRUE ( gyro.AddGyro   ( 2.100, RAW_GYRO_BASE +  50, true));
   EXPECT_NEAR (  0.0, gyro.Base() ,0.01);
   EXPECT_NEAR (  0.0, gyro.Scale() ,0.01);
   EXPECT_NEAR (  0.0, gyro.CalibartedTo() ,0.01);
   EXPECT_TRUE ( gyro.AddGyro   ( 2.600, RAW_GYRO_BASE + 100, true));
   EXPECT_TRUE ( gyro.AddHeading( 3.000,                  15, HEADING_DEVIATION_DEG));//-10
   EXPECT_TRUE ( gyro.AddGyro   ( 3.100, RAW_GYRO_BASE + 100, true));
   EXPECT_NEAR (  0.0, gyro.Base() ,0.01);
   EXPECT_NEAR (  0.0, gyro.Scale() ,0.01);
   EXPECT_NEAR (  0.0, gyro.CalibartedTo() ,0.01);
   EXPECT_TRUE ( gyro.AddGyro   ( 3.600, RAW_GYRO_BASE + 200, true));
   EXPECT_TRUE ( gyro.AddHeading( 4.000,                  35, HEADING_DEVIATION_DEG));//-20
   EXPECT_TRUE ( gyro.AddGyro   ( 4.100, RAW_GYRO_BASE + 200, true));
   EXPECT_NEAR (  0.0, gyro.Base() ,0.01);
   EXPECT_NEAR (  0.0, gyro.Scale() ,0.01);
   EXPECT_NEAR (  0.0, gyro.CalibartedTo() ,0.01);
   EXPECT_TRUE ( gyro.AddGyro   ( 4.600, RAW_GYRO_BASE + 250, true));
   EXPECT_TRUE ( gyro.AddHeading( 5.000,                  60, HEADING_DEVIATION_DEG));//-25
   EXPECT_TRUE ( gyro.AddGyro   ( 5.100, RAW_GYRO_BASE + 250, true));
   EXPECT_NEAR ( 2048.00, gyro.Base() ,0.01);
   EXPECT_NEAR (   -0.10, gyro.Scale() ,0.01);
   EXPECT_NEAR (   50.00, gyro.CalibartedTo() ,0.01);
   EXPECT_TRUE ( gyro.AddGyro   ( 5.600, RAW_GYRO_BASE + 300, true));
   EXPECT_TRUE ( gyro.AddHeading( 6.000,                  90, HEADING_DEVIATION_DEG));//-30
   EXPECT_TRUE ( gyro.AddGyro   ( 6.100, RAW_GYRO_BASE + 300, true));
   EXPECT_NEAR ( 2048.00, gyro.Base() ,0.01);
   EXPECT_NEAR (   -0.10, gyro.Scale() ,0.01);
   EXPECT_NEAR (   66.67, gyro.CalibartedTo() ,0.01);
   EXPECT_TRUE ( gyro.AddGyro   ( 6.600, RAW_GYRO_BASE + 400, true));
   EXPECT_TRUE ( gyro.AddHeading( 7.000,                 130, HEADING_DEVIATION_DEG));//-40
   EXPECT_TRUE ( gyro.AddGyro   ( 7.100, RAW_GYRO_BASE + 400, true));
   EXPECT_NEAR ( 2048.00, gyro.Base() ,0.01);
   EXPECT_NEAR (   -0.10, gyro.Scale() ,0.01);
   EXPECT_NEAR (   75.00, gyro.CalibartedTo() ,0.01);
   EXPECT_TRUE ( gyro.AddGyro   ( 7.600, RAW_GYRO_BASE + 100, true));
   EXPECT_TRUE ( gyro.AddHeading( 8.000,                 140, HEADING_DEVIATION_DEG));//-10
   EXPECT_TRUE ( gyro.AddGyro   ( 8.100, RAW_GYRO_BASE + 100, true));
   EXPECT_NEAR ( 2048.00, gyro.Base() ,0.01);
   EXPECT_NEAR (   -0.10, gyro.Scale() ,0.01);
   EXPECT_NEAR (   80.00, gyro.CalibartedTo() ,0.01);
   EXPECT_TRUE ( gyro.AddGyro   ( 8.600, RAW_GYRO_BASE +  50, true));
   EXPECT_TRUE ( gyro.AddHeading( 9.000,                 145, HEADING_DEVIATION_DEG));//-5
   EXPECT_TRUE ( gyro.AddGyro   ( 9.100, RAW_GYRO_BASE +  50, true));
   EXPECT_NEAR (    9.10, gyro.TimeStamp() ,0.01);
   EXPECT_NEAR (   -5.00, gyro.Value() ,0.01);
   EXPECT_NEAR (    0.00, gyro.Accuracy() ,0.01);
   EXPECT_NEAR ( 2048.00, gyro.Base() ,0.01);
   EXPECT_NEAR (   -0.10, gyro.Scale() ,0.01);
   EXPECT_NEAR (   83.33, gyro.CalibartedTo() ,0.01);
   EXPECT_TRUE ( gyro.AddGyro   ( 9.600, RAW_GYRO_BASE - 1.234567, true));
   EXPECT_NEAR (   83.33, gyro.CalibartedTo() ,0.01);
   EXPECT_NEAR (    9.60, gyro.TimeStamp() ,0.01);
   EXPECT_NEAR (    0.1234567, gyro.Value() ,0.0000001);
   EXPECT_NEAR (    0.00, gyro.Accuracy() ,0.01);
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
