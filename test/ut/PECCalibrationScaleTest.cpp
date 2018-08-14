/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2018 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */


/**
 * Unit test of the PECCalibrationScale class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PECCalibrationScale.h"


class PECCalibrationScaleTest : public ::testing::Test
{
public:
   virtual void SetUp()
   {}
   virtual void TearDown()
   {}
};


/**
 * tests creation 
 */
TEST_F(PECCalibrationScaleTest, test_create)
{
   PE::CNormalisation scale;
   PE::CCalibrationScale   calib(scale, 10, 1);

   EXPECT_FALSE( calib.GetSensor(PE::SBasicSensor()).IsValid() );
   EXPECT_NEAR(0.00, calib.CalibratedTo(), 0.01);
}


/**
 * tests  
 */
TEST_F(PECCalibrationScaleTest, test_no_reference)
{
   PE::CNormalisation scale;
   PE::CCalibrationScale   calib(scale, 10, 1);

   calib.AddSensor(PE::SBasicSensor(10,0.1));
   calib.AddSensor(PE::SBasicSensor(11,0.1));
   calib.AddSensor(PE::SBasicSensor(12,0.1));
   calib.AddSensor(PE::SBasicSensor(13,0.1));
   calib.AddSensor(PE::SBasicSensor(14,0.1));
   calib.AddSensor(PE::SBasicSensor(15,0.1));
   EXPECT_FALSE( calib.GetSensor(PE::SBasicSensor(15,0.1)).IsValid() );
   EXPECT_NEAR(0.00, calib.CalibratedTo(), 0.01);
}


/**
 * tests  
 */
TEST_F(PECCalibrationScaleTest, test_no_sensor)
{
   PE::CNormalisation scale;
   PE::CCalibrationScale   calib(scale, 3, 1);

   calib.AddReference(PE::SBasicSensor(10,0.1));
   calib.AddReference(PE::SBasicSensor(20,0.1));
   calib.AddReference(PE::SBasicSensor(30,0.1));
   calib.AddReference(PE::SBasicSensor(40,0.1));
   calib.AddReference(PE::SBasicSensor(50,0.1));
   EXPECT_FALSE( calib.GetSensor(PE::SBasicSensor(0.0,0.1)).IsValid() );
   EXPECT_NEAR(0.00, calib.CalibratedTo(), 0.01);
}


/**
 * tests  
 */
TEST_F(PECCalibrationScaleTest, test_simple_calibartion)
{
   PE::CNormalisation ok;
   PE::CCalibrationScale   calib_ok(  ok, 3, 1);

   calib_ok.AddSensor(PE::SBasicSensor(90,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(100,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(110,0.1));
   calib_ok.AddReference(PE::SBasicSensor(10,0.1));
   EXPECT_FALSE( calib_ok.GetSensor(PE::SBasicSensor(110,0.1)).IsValid() );
   EXPECT_NEAR( 0.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(120,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(130,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(140,0.1));
   calib_ok.AddReference(PE::SBasicSensor(13,0.1));
   EXPECT_FALSE( calib_ok.GetSensor(PE::SBasicSensor(140,0.1)).IsValid() );
   EXPECT_NEAR( 0.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(140,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(120,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(100,0.1));
   calib_ok.AddReference(PE::SBasicSensor(12,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 9.00, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.01);
   EXPECT_NEAR(50.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(90,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(80,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(70,0.1));
   calib_ok.AddReference(PE::SBasicSensor(8,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 9.00, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.01);
   EXPECT_NEAR(66.66, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(60,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(30,0.1));
   calib_ok.AddSensor(PE::SBasicSensor( 0,0.1));
   calib_ok.AddReference(PE::SBasicSensor(3,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(110,0.1)).IsValid() );
   EXPECT_NEAR(11.00, calib_ok.GetSensor(PE::SBasicSensor(110,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(110,0.1)).Accuracy,0.01);
   EXPECT_NEAR(75.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(10,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(20,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(30,0.1));
   calib_ok.AddReference(PE::SBasicSensor(2,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(140,0.1)).IsValid() );
   EXPECT_NEAR(14.00, calib_ok.GetSensor(PE::SBasicSensor(140,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(140,0.1)).Accuracy,0.01);
   EXPECT_NEAR(80.00, calib_ok.CalibratedTo(), 0.01);
}


/**
 * tests  
 */
TEST_F(PECCalibrationScaleTest, test_calibartion_within_threshold_limit)
{
   PE::CNormalisation ok;
   PE::CCalibrationScale   calib_ok(  ok, 3, 1);

   calib_ok.AddSensor(PE::SBasicSensor(90,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(100,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(110,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(100,0.1)); //sensors still in a threshold limit
   calib_ok.AddReference(PE::SBasicSensor(10,0.1));
   EXPECT_FALSE( calib_ok.GetSensor(PE::SBasicSensor(110,0.1)).IsValid() );
   EXPECT_NEAR( 0.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(120,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(130,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(130,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(140,0.1)); //sensors still in a threshold limit
   calib_ok.AddReference(PE::SBasicSensor(13,0.1));
   EXPECT_FALSE( calib_ok.GetSensor(PE::SBasicSensor(140,0.1)).IsValid() );
   EXPECT_NEAR( 0.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(140,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(120,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(120,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(100,0.1)); //sensors still in a threshold limit
   calib_ok.AddReference(PE::SBasicSensor(12,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 9.00, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.01);
   EXPECT_NEAR(50.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(90,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(80,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(80,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(70,0.1)); //sensors still in a threshold limit
   calib_ok.AddReference(PE::SBasicSensor(8,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 9.00, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.01);
   EXPECT_NEAR(66.66, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(60,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(30,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(30,0.1));
   calib_ok.AddSensor(PE::SBasicSensor( 0,0.1)); //sensors still in a threshold limit
   calib_ok.AddReference(PE::SBasicSensor(3,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(110,0.1)).IsValid() );
   EXPECT_NEAR(11.00, calib_ok.GetSensor(PE::SBasicSensor(110,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(110,0.1)).Accuracy,0.01);
   EXPECT_NEAR(75.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(10,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(20,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(20,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(30,0.1)); //sensors still in a threshold limit
   calib_ok.AddReference(PE::SBasicSensor(2,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(140,0.1)).IsValid() );
   EXPECT_NEAR(14.00, calib_ok.GetSensor(PE::SBasicSensor(140,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(140,0.1)).Accuracy,0.01);
   EXPECT_NEAR(80.00, calib_ok.CalibratedTo(), 0.01);
}


/**
 * tests  
 */
TEST_F(PECCalibrationScaleTest, test_calibartion_over_ratio)
{
   PE::CNormalisation ok;
   PE::CCalibrationScale   calib_ok(  ok, 3, 1);

   calib_ok.AddSensor(PE::SBasicSensor(90,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(100,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(110,0.1));
   calib_ok.AddReference(PE::SBasicSensor(10,0.1));
   EXPECT_FALSE( calib_ok.GetSensor(PE::SBasicSensor(110,0.1)).IsValid() );
   EXPECT_NEAR( 0.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(120,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(130,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(140,0.1));
   calib_ok.AddReference(PE::SBasicSensor(13,0.1));
   EXPECT_FALSE( calib_ok.GetSensor(PE::SBasicSensor(140,0.1)).IsValid() );
   EXPECT_NEAR( 0.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(140,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(120,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(100,0.1));
   calib_ok.AddReference(PE::SBasicSensor(12,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 9.00, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.01);
   EXPECT_NEAR(50.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(90,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(80,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(70,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(80,0.1)); //sensors still in a threshold limit
   calib_ok.AddSensor(PE::SBasicSensor(80,0.1)); //sensors over ratio +/-threshold limit and will be rejected
   calib_ok.AddReference(PE::SBasicSensor(8,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 9.00, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.01);
   EXPECT_NEAR(50.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddReference(PE::SBasicSensor(3,0.1));
   calib_ok.AddReference(PE::SBasicSensor(3,0.1));
   calib_ok.AddSensor(PE::SBasicSensor( 0,0.1)); //sensors over ratio +/-threshold limit and will be rejected
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(110,0.1)).IsValid() );
   EXPECT_NEAR(11.00, calib_ok.GetSensor(PE::SBasicSensor(110,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(110,0.1)).Accuracy,0.01);
   EXPECT_NEAR(50.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(10,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(20,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(30,0.1));
   calib_ok.AddReference(PE::SBasicSensor(2,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(140,0.1)).IsValid() );
   EXPECT_NEAR(14.00, calib_ok.GetSensor(PE::SBasicSensor(140,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(140,0.1)).Accuracy,0.01);
   EXPECT_NEAR(33.33, calib_ok.CalibratedTo(), 0.01);
}


/**
 * tests  
 */
TEST_F(PECCalibrationScaleTest, test_calibartion_with_invalid_ref_value)
{
   PE::CNormalisation ok;
   PE::CCalibrationScale   calib_ok(  ok, 3, 1);

   calib_ok.AddSensor(PE::SBasicSensor(90,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(100,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(110,0.1));
   calib_ok.AddReference(PE::SBasicSensor(10,0.1));
   EXPECT_FALSE( calib_ok.GetSensor(PE::SBasicSensor(110,0.1)).IsValid() );
   EXPECT_NEAR( 0.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(120,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(130,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(140,0.1));
   calib_ok.AddReference(PE::SBasicSensor(13,0.1));
   EXPECT_FALSE( calib_ok.GetSensor(PE::SBasicSensor(140,0.1)).IsValid() );
   EXPECT_NEAR( 0.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(0xFFFF,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(0xFFFF,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(0xFFFF,0.1));
   calib_ok.AddReference(PE::SBasicSensor()); //invalid reference value will be ignored
   EXPECT_FALSE( calib_ok.GetSensor(PE::SBasicSensor(10,0.1)).IsValid() );
   EXPECT_NEAR( 0.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(0xFFFF,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(0xFFFF,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(0xFFFF,0.1));
   calib_ok.AddReference(PE::SBasicSensor(8,0.1)); //since it will over limit ratio all data before will be rejected
   EXPECT_FALSE( calib_ok.GetSensor(PE::SBasicSensor(10,0.1)).IsValid() );
   EXPECT_NEAR( 0.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(140,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(120,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(100,0.1));
   calib_ok.AddReference(PE::SBasicSensor(12,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 9.00, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.01);
   EXPECT_NEAR(50.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(90,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(80,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(70,0.1));
   calib_ok.AddReference(PE::SBasicSensor(8,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 9.00, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.01);
   EXPECT_NEAR(66.66, calib_ok.CalibratedTo(), 0.01);
}


/**
 * tests  
 */
TEST_F(PECCalibrationScaleTest, test_calibartion_with_invalid_raw_value)
{
   PE::CNormalisation ok;
   PE::CCalibrationScale   calib_ok(  ok, 3, 1);

   calib_ok.AddSensor(PE::SBasicSensor(90,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(100,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(110,0.1));
   calib_ok.AddReference(PE::SBasicSensor(10,0.1));
   EXPECT_FALSE( calib_ok.GetSensor(PE::SBasicSensor(110,0.1)).IsValid() );
   EXPECT_NEAR( 0.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(120,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(130,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(140,0.1));
   calib_ok.AddReference(PE::SBasicSensor(13,0.1));
   EXPECT_FALSE( calib_ok.GetSensor(PE::SBasicSensor(140,0.1)).IsValid() );
   EXPECT_NEAR( 0.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(140,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(120,0.1));
   calib_ok.AddSensor(PE::SBasicSensor()); //will be simple ignored
   calib_ok.AddSensor(PE::SBasicSensor()); //will be simple ignored
   calib_ok.AddSensor(PE::SBasicSensor()); //will be simple ignored
   calib_ok.AddSensor(PE::SBasicSensor()); //will be simple ignored
   calib_ok.AddSensor(PE::SBasicSensor()); //will be simple ignored
   calib_ok.AddSensor(PE::SBasicSensor(100,0.1));
   calib_ok.AddReference(PE::SBasicSensor(12,0.1)); //no over limit since invalid sensors were ignored
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 9.00, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.01);
   EXPECT_NEAR(50.00, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor(PE::SBasicSensor(90,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(80,0.1));
   calib_ok.AddSensor(PE::SBasicSensor(70,0.1));
   calib_ok.AddReference(PE::SBasicSensor(8,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 9.00, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.01);
   EXPECT_NEAR(66.66, calib_ok.CalibratedTo(), 0.01);
}


/**
 * tests
 */
TEST_F(PECCalibrationScaleTest, test_calibartion_raw_is_first_for_one_cycle_before_reference)
{
   PE::CNormalisation ok;
   PE::CCalibrationScale   calib_ok(  ok, 1, 1);

   calib_ok.AddSensor   (PE::SBasicSensor( 0,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 0,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(10,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 0,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(20,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 1,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(40,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 2,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(70,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 4,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 7.53, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.093, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.001);
   EXPECT_NEAR(20.79, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor   (PE::SBasicSensor(20,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 7,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(90,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 2,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(30,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 9,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(10,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 3,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(10,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 1,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 7.81, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.097, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.001);
   EXPECT_NEAR(56.23, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor   (PE::SBasicSensor(20,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 1,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(80,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 2,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(50,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 8,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(10,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 5,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 8.20, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.101, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.001);
   EXPECT_NEAR(67.53, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor   (PE::SBasicSensor( 0,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 1,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor( 0,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 0,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 8.35, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.103, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.001);
   EXPECT_NEAR(71.03, calib_ok.CalibratedTo(), 0.01);
}


/**
 * tests
 */
TEST_F(PECCalibrationScaleTest, test_calibartion_reference_is_first_for_one_cycle_before_raw)
{
   PE::CNormalisation ok;
   PE::CCalibrationScale   calib_ok(  ok, 1, 1);

   calib_ok.AddSensor   (PE::SBasicSensor( 0,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 0,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor( 0,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 2,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(20,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 5,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(50,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 2,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(20,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 1,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 9.48, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.120, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.001);
   EXPECT_NEAR(10.13, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor   (PE::SBasicSensor(10,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 0,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor( 0,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 3,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(30,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 8,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 9.47, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.117, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.001);
   EXPECT_NEAR(44.64, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor   (PE::SBasicSensor(80,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 4,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(40,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 3,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 9.35, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.114, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.001);
   EXPECT_NEAR(56.33, calib_ok.CalibratedTo(), 0.01);

   calib_ok.AddSensor   (PE::SBasicSensor(30,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 0,0.1));
   calib_ok.AddSensor   (PE::SBasicSensor(00,0.1));
   calib_ok.AddReference(PE::SBasicSensor( 0,0.1));
   EXPECT_TRUE( calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).IsValid() );
   EXPECT_NEAR( 9.30, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Value,0.01);
   EXPECT_NEAR( 0.112, calib_ok.GetSensor(PE::SBasicSensor(90,0.1)).Accuracy,0.001);
   EXPECT_NEAR(64.38, calib_ok.CalibratedTo(), 0.01);
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

