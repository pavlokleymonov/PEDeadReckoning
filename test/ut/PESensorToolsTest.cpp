/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2017 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */


/**
 * Unit test of the PE::Sensor namespace.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "PESensorTools.h"

class PESensorTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }
};


/**
 * checks PredictValue
 */
TEST_F(PESensorTest, test_PredictValue )
{
   /**
    *  Case: speed in between of two odometers
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1005s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1010s |    20 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR ( 15, PE::Sensor::PredictValue( 1005, 1000, 1010, 10,20), PE::EPSILON );

   /**
    *  Case: speed same as first odometer timestamp
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1000s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1010s |    20 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR ( 10, PE::Sensor::PredictValue( 1000, 1000, 1010, 10,20), PE::EPSILON );

   /**
    *  Case: speed same as last odometer timestamp
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1010s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1010s |    20 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR ( 20, PE::Sensor::PredictValue( 1010, 1000, 1010, 10,20), PE::EPSILON );

   /**
    *  Case: speed close to the first odometer timestamp
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1001s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1010s |    20 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR ( 11, PE::Sensor::PredictValue( 1001, 1000, 1010, 10,20), PE::EPSILON );

   /**
    *  Case: speed close to the last odometer timestamp
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1009s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1010s |    20 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR ( 19, PE::Sensor::PredictValue( 1009, 1000, 1010, 10,20), PE::EPSILON );

   /**
    *  Case: odometer values are equal
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1001s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1010s |    10 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR ( 10, PE::Sensor::PredictValue( 1001, 1000, 1010, 10,10), PE::EPSILON );

   /**
    *  Case: odometer timestam and values are equal -- wrong case - has to be avoided!!! - NaN value
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1000s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1000s |    10 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_TRUE ( PE::isnan(PE::Sensor::PredictValue( 1000, 1000, 1000, 10, 10)) );

   /**
    *  Case: odometer timestam are equal but value are different -- wrong case - has to be avoided!!! - NaN value
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1000s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1000s |    20 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_TRUE ( PE::isnan(PE::Sensor::PredictValue( 1000, 1000, 1000, 10, 20)) );

   /**
    *  Case: speed timestamp is before the first odometer speed -- wrong case - has to be avoided!!!
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |      999s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1000s |    10 | First odometer                                      |
    *  |     1010s |    20 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR (  9, PE::Sensor::PredictValue( 999, 1000, 1010, 10,20), PE::EPSILON );

   /**
    *  Case: speed timestamp is after the last odometer speed -- wrong case - has to be avoided!!!
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1010s |    20 | Last odometer                                       |
    *  |     1011s |  ???? | Predicted odometer for timestamp of reference speed |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR ( 21, PE::Sensor::PredictValue( 1011, 1000, 1010, 10,20), PE::EPSILON );
}


/**
 * checks IsIntervalOk
 */
TEST_F(PESensorTest, test_IsIntervalOk )
{
   double       INTERVAL_1000MS = 1.000;
   double       HYSTERESIS_10MS = 0.010;

   //interval 1 second +/-10ms
   //all is valid
   EXPECT_TRUE (PE::Sensor::IsIntervalOk(       1.000, INTERVAL_1000MS, HYSTERESIS_10MS));
   //timeinterval almost zero
   EXPECT_FALSE(PE::Sensor::IsIntervalOk( PE::EPSILON, INTERVAL_1000MS, HYSTERESIS_10MS));
   //timeinterval is out +10ms
   EXPECT_FALSE(PE::Sensor::IsIntervalOk(       1.010001, INTERVAL_1000MS, HYSTERESIS_10MS));
   //timeinterval is out -10ms
   EXPECT_FALSE(PE::Sensor::IsIntervalOk(       0.989999, INTERVAL_1000MS, HYSTERESIS_10MS));
}


/**
 * checks IsAccuracyOk
 */
TEST_F(PESensorTest, test_IsAccuracyOk )
{
   uint32_t   ACCURACY_RATIO_x2 = 2;
   uint32_t  ACCURACY_RATIO_x10 = 10;
   uint32_t   ACCURACY_RATIO_x0 = 0;

   //all is valid
   EXPECT_TRUE (PE::Sensor::IsAccuracyOk(  10.000, 0.1, ACCURACY_RATIO_x2));
   EXPECT_TRUE (PE::Sensor::IsAccuracyOk(  10.000, 0.1, ACCURACY_RATIO_x10));
   EXPECT_TRUE (PE::Sensor::IsAccuracyOk(  10.000, 0.1, ACCURACY_RATIO_x0));
   //value is out of accuracy ratio
   EXPECT_FALSE(PE::Sensor::IsAccuracyOk(  10.000, 1.0, ACCURACY_RATIO_x10));
}


/**
 * checks IsInRange
 */
TEST_F(PESensorTest, test_IsInRange )
{
   //data is in range from left
   EXPECT_TRUE (PE::Sensor::IsInRange( 10, 10, 100));
   //data is in range from right
   EXPECT_TRUE (PE::Sensor::IsInRange( 100, 10, 100));
   //data positive is in range in between
   EXPECT_TRUE (PE::Sensor::IsInRange(  50, 10, 100));
   //data negative is in range in between
   EXPECT_TRUE (PE::Sensor::IsInRange( -5, -10, 100));
   //data is out of range from the left
   EXPECT_FALSE(PE::Sensor::IsInRange(   1, 10, 100));
   //data is out of range from the right
   EXPECT_FALSE(PE::Sensor::IsInRange( 101, 10, 100));
   //wrong range definition
   EXPECT_FALSE(PE::Sensor::IsInRange(  50, 100, 10));
   //tets NaN value
   EXPECT_FALSE(PE::Sensor::IsInRange(  std::numeric_limits<double>::quiet_NaN(), 10, 100));
   EXPECT_FALSE(PE::Sensor::IsInRange(  std::numeric_limits<double>::quiet_NaN(), 10, 100));
   EXPECT_FALSE(PE::Sensor::IsInRange(  50, std::numeric_limits<double>::quiet_NaN(), 100));
   EXPECT_FALSE(PE::Sensor::IsInRange(  50, 10, std::numeric_limits<double>::quiet_NaN()));
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
