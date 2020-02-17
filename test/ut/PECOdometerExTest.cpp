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
 * Unit test of the PE::COdometer class.
 *
 * Code under test:
 *
 */

#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "PECOdometerEx.h"
#include "PETools.h"

class PECOdometerExTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }
};


/**
 * test odometer all is valid
 */
TEST_F(PECOdometerExTest, test_odo_all_is_valid)
{
   PE::TValue SPEED_INTERVAL_100MS           = 0.100;
   PE::TValue SPEED_INTERVAL_HYSTERESIS_10MS = 0.010;
   PE::TValue SPEED_MIN                      = 0.0;
   PE::TValue SPEED_MAX                      = 343.0; //sound speed ~1235km/h
   PE::TValue SPEED_ACCURACY_RATIO_2X        = 2;
   PE::TValue ODO_INTERVAL_25MS                = 0.025;
   PE::TValue ODO_INTERVAL_HYSTERESIS_5MS      = 0.005;
   PE::TValue ODO_MIN                          = 0;
   PE::TValue ODO_MAX                          = 1000000;
   PE::COdometerEx odo( SPEED_INTERVAL_100MS,
                        SPEED_INTERVAL_HYSTERESIS_10MS,
                        SPEED_MIN,
                        SPEED_MAX,
                        SPEED_ACCURACY_RATIO_2X,
                        ODO_INTERVAL_25MS,
                        ODO_INTERVAL_HYSTERESIS_5MS,
                        ODO_MIN,
                        ODO_MAX);

   //init values have to be NaN
   EXPECT_TRUE( PE::isnan(odo.GetRefValue()) );
   EXPECT_TRUE( PE::isnan(odo.GetSenValue()) );

   //first adding is always false
   EXPECT_FALSE( odo.AddSpeed(1.000,   0, 0.1) );
   EXPECT_FALSE( odo.AddTicks(1.010,   0, true) );
   EXPECT_TRUE ( odo.AddTicks(1.035,  25, true) );
   EXPECT_TRUE ( odo.AddTicks(1.060,  50, true) );
   EXPECT_TRUE ( odo.AddTicks(1.085,  75, true) );
   EXPECT_TRUE ( odo.AddSpeed(1.100,   5, 0.1) );
   EXPECT_TRUE ( odo.AddTicks(1.110, 100, true) );
   EXPECT_TRUE ( odo.AddTicks(1.135, 100, true) );
   EXPECT_TRUE ( odo.AddTicks(1.160, 100, true) );
   EXPECT_TRUE ( odo.AddTicks(1.185, 150, true) );
   EXPECT_TRUE ( odo.AddSpeed(1.200,  10, 0.1) );
   EXPECT_TRUE ( odo.AddTicks(1.210, 200, true) );
   EXPECT_TRUE ( odo.AddTicks(1.235,   0, true) );
   EXPECT_TRUE ( odo.AddTicks(1.260,   0, true) );
   EXPECT_TRUE ( odo.AddTicks(1.285, 200, true) );
   EXPECT_TRUE ( odo.AddSpeed(1.300,  40, 0.1) );
   EXPECT_TRUE ( odo.AddTicks(1.310, 400, true) );
   EXPECT_TRUE ( odo.AddTicks(1.335,   0, true) );
   EXPECT_TRUE ( odo.AddTicks(1.360,   0, true) );
   EXPECT_TRUE ( odo.AddTicks(1.385,  50, true) );
   EXPECT_TRUE ( odo.AddSpeed(1.400,  10, 0.1) );
   EXPECT_TRUE ( odo.AddTicks(1.410, 100, true) );
   EXPECT_TRUE ( odo.AddTicks(1.435,   0, true) );
   EXPECT_TRUE ( odo.AddTicks(1.460,   0, true) );
   EXPECT_TRUE ( odo.AddTicks(1.485, 100, true) );
   EXPECT_TRUE ( odo.AddSpeed(1.500,  20, 0.1) );
   EXPECT_TRUE ( odo.AddTicks(1.510, 200, true) );
   EXPECT_TRUE ( odo.AddTicks(1.535,   0, true) );
   EXPECT_TRUE ( odo.AddTicks(1.560,   0, true) );
   EXPECT_TRUE ( odo.AddTicks(1.585, 150, true) );
   EXPECT_TRUE ( odo.AddSpeed(1.600,  30, 0.1) );
   EXPECT_TRUE ( odo.AddTicks(1.610, 300, true) );
   EXPECT_TRUE ( odo.AddTicks(1.635,   0, true) );
   EXPECT_TRUE ( odo.AddTicks(1.660,   0, true) );
   EXPECT_TRUE ( odo.AddTicks(1.685,   5, true) );
   EXPECT_TRUE ( odo.AddSpeed(1.700,   1, 0.1) );
   EXPECT_TRUE ( odo.AddTicks(1.710,  10, true) );

   EXPECT_NEAR (   0.000, odo.Base() ,0.001);
   EXPECT_NEAR (   0.005, odo.Scale() ,0.001);
   EXPECT_NEAR (  41.590, odo.CalibratedTo() ,0.001);
   EXPECT_NEAR (   1.710, odo.TimeStamp(),0.001);
   EXPECT_NEAR (   1.000, odo.Value(),0.001);
   EXPECT_NEAR (   0.000, odo.Accuracy(),0.001);

   EXPECT_TRUE ( odo.AddTicks(1.735, 110, true) );
   EXPECT_NEAR (   1.735, odo.TimeStamp(),0.001);
   EXPECT_NEAR (  20.000, odo.Value(),0.001);
   EXPECT_NEAR (   0.000, odo.Accuracy(),0.001);
}


/**
 * test set/get reference value
 */
TEST_F(PECOdometerExTest, test_set_get_reference_value)
{
   PE::TValue SPEED_INTERVAL_100MS           = 0.100;
   PE::TValue SPEED_INTERVAL_HYSTERESIS_10MS = 0.010;
   PE::TValue SPEED_MIN                      = 0.0;
   PE::TValue SPEED_MAX                      = 343.0; //sound speed ~1235km/h
   PE::TValue SPEED_ACCURACY_RATIO_2X        = 2;
   PE::TValue ODO_INTERVAL_50MS                = 0.050;
   PE::TValue ODO_INTERVAL_HYSTERESIS_5MS      = 0.005;
   PE::TValue ODO_MIN                          = 0;
   PE::TValue ODO_MAX                          = 2048;
   PE::COdometerEx odo( SPEED_INTERVAL_100MS,
                        SPEED_INTERVAL_HYSTERESIS_10MS,
                        SPEED_MIN,
                        SPEED_MAX,
                        SPEED_ACCURACY_RATIO_2X,
                        ODO_INTERVAL_50MS,
                        ODO_INTERVAL_HYSTERESIS_5MS,
                        ODO_MIN,
                        ODO_MAX);

   //init values have to be NaN
   EXPECT_TRUE( PE::isnan(odo.GetRefValue()) );

   //simulate first ref value, oldTimestamp is 0
   EXPECT_FALSE( odo.SetRefValue(0, 1.000, 10.0, 0.1) );

}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
