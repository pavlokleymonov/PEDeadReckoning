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
 * Unit test of the CCalibrationBase class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PECCalibrationBase.h"


class PECCalibrationBaseTest : public ::testing::Test
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
TEST_F(PECCalibrationBaseTest, test_create_destroy)
{
   PE::CNormalisation base;
   PE::CCalibration* calib = new PE::CCalibrationBase(&base, 10, 1);

   EXPECT_FALSE( calib->GetSensor(PE::SBasicSensor()).IsValid() );
   EXPECT_NEAR(0.00, base.GetReliable(), 0.01);

   delete calib;
}


/**
 * tests invalid normalisation 
 */
TEST_F(PECCalibrationBaseTest, test_invalid_normalisation)
{
   PE::CCalibration* calib = new PE::CCalibrationBase(0, 3, 1);

   calib->AddSensor(PE::SBasicSensor(1,0.1));
   calib->AddSensor(PE::SBasicSensor(2,0.1));
   calib->AddSensor(PE::SBasicSensor(3,0.1));
   calib->AddReference(PE::SBasicSensor(1,0.1));
   calib->AddSensor(PE::SBasicSensor(11,0.1));
   calib->AddSensor(PE::SBasicSensor(11,0.1));
   calib->AddSensor(PE::SBasicSensor(11,0.1));
   calib->AddReference(PE::SBasicSensor(10,0.1));
   EXPECT_FALSE( calib->GetSensor(PE::SBasicSensor()).IsValid() );

   delete calib;
}


/**
 * tests  
 */
TEST_F(PECCalibrationBaseTest, test_no_reference)
{
   PE::CNormalisation base;
   PE::CCalibration* calib = new PE::CCalibrationBase(&base, 10, 1);

   calib->AddSensor(PE::SBasicSensor(10,0.1));
   calib->AddSensor(PE::SBasicSensor(11,0.1));
   calib->AddSensor(PE::SBasicSensor(12,0.1));
   calib->AddSensor(PE::SBasicSensor(13,0.1));
   calib->AddSensor(PE::SBasicSensor(14,0.1));
   calib->AddSensor(PE::SBasicSensor(15,0.1));
   EXPECT_FALSE( calib->GetSensor(PE::SBasicSensor(15,0.1)).IsValid() );
   EXPECT_NEAR(0.00, base.GetReliable(), 0.01);
   delete calib;
}


/**
 * tests  
 */
TEST_F(PECCalibrationBaseTest, test_no_sensor)
{
   PE::CNormalisation base;
   PE::CCalibrationBase   calib(&base, 3, 1);

   calib.AddReference(PE::SBasicSensor(10,0.1));
   calib.AddReference(PE::SBasicSensor(20,0.1));
   calib.AddReference(PE::SBasicSensor(30,0.1));
   calib.AddReference(PE::SBasicSensor(40,0.1));
   calib.AddReference(PE::SBasicSensor(50,0.1));
   EXPECT_FALSE( calib.GetSensor(PE::SBasicSensor(0.0,0.1)).IsValid() );
   EXPECT_NEAR(0.00, base.GetReliable(), 0.01);
}


/**
 * tests  
 */
TEST_F(PECCalibrationBaseTest, test_simple_calibartion)
{
   PE::CNormalisation base;
   PE::CCalibrationBase   calib( &base, 3, 1);

   calib.AddSensor(PE::SBasicSensor(1,0.1));
   calib.AddSensor(PE::SBasicSensor(2,0.1));
   calib.AddSensor(PE::SBasicSensor(3,0.1));
   calib.AddReference(PE::SBasicSensor(1,0.1));
   EXPECT_FALSE( calib.GetSensor(PE::SBasicSensor(2,0.1)).IsValid() );
   EXPECT_NEAR( 0.00, base.GetReliable(), 0.01);

   calib.AddSensor(PE::SBasicSensor(11,0.1));
   calib.AddSensor(PE::SBasicSensor(11,0.1));
   calib.AddSensor(PE::SBasicSensor(11,0.1));
   calib.AddReference(PE::SBasicSensor(10,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(11,0.1)).IsValid() );
   EXPECT_NEAR( 10.00, calib.GetSensor(PE::SBasicSensor(11,0.1)).Value,0.01);
   EXPECT_NEAR(  0.10, calib.GetSensor(PE::SBasicSensor(11,0.1)).Accuracy,0.01);
   EXPECT_NEAR( 50.00, base.GetReliable(), 0.01);

   calib.AddSensor(PE::SBasicSensor(181,0.1));
   calib.AddSensor(PE::SBasicSensor(182,0.1));
   calib.AddSensor(PE::SBasicSensor(183,0.1));
   calib.AddReference(PE::SBasicSensor(181,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(182,0.1)).IsValid() );
   EXPECT_NEAR(181.00, calib.GetSensor(PE::SBasicSensor(182,0.1)).Value,0.01);
   EXPECT_NEAR(  0.10, calib.GetSensor(PE::SBasicSensor(182,0.1)).Accuracy,0.01);
   EXPECT_NEAR( 66.66, base.GetReliable(), 0.01);

   calib.AddSensor(PE::SBasicSensor(0,0.1));
   calib.AddSensor(PE::SBasicSensor(1,0.1));
   calib.AddSensor(PE::SBasicSensor(2,0.1));
   calib.AddReference(PE::SBasicSensor(0,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(0,0.1)).IsValid() );
   EXPECT_NEAR(-1.00, calib.GetSensor(PE::SBasicSensor(0,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib.GetSensor(PE::SBasicSensor(0,0.1)).Accuracy,0.01);
   EXPECT_NEAR(75.00, base.GetReliable(), 0.01);

   calib.AddSensor(PE::SBasicSensor(100,0.1));
   calib.AddSensor(PE::SBasicSensor( 50,0.1));
   calib.AddSensor(PE::SBasicSensor(  0,0.1));
   calib.AddReference(PE::SBasicSensor(49,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(51,0.1)).IsValid() );
   EXPECT_NEAR(50.00, calib.GetSensor(PE::SBasicSensor(51,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib.GetSensor(PE::SBasicSensor(51,0.1)).Accuracy,0.01);
   EXPECT_NEAR(80.00, base.GetReliable(), 0.01);
}

/**
 * tests  
 */
TEST_F(PECCalibrationBaseTest, test_calibartion_over_ratio)
{
   PE::CNormalisation base;
   PE::CCalibrationBase   calib(&base, 3, 1);

   calib.AddSensor(PE::SBasicSensor(10,0.1));
   calib.AddSensor(PE::SBasicSensor(11,0.1));
   calib.AddSensor(PE::SBasicSensor(12,0.1));
   calib.AddReference(PE::SBasicSensor(10,0.1));
   EXPECT_FALSE( calib.GetSensor(PE::SBasicSensor(11,0.1)).IsValid() );
   EXPECT_NEAR( 0.00, base.GetReliable(), 0.01);

   calib.AddSensor(PE::SBasicSensor(11,0.1));
   calib.AddSensor(PE::SBasicSensor(12,0.1));
   calib.AddSensor(PE::SBasicSensor(13,0.1));
   calib.AddReference(PE::SBasicSensor(11,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(11,0.1)).IsValid() );
   EXPECT_NEAR(10.00, calib.GetSensor(PE::SBasicSensor(11,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib.GetSensor(PE::SBasicSensor(11,0.1)).Accuracy,0.01);
   EXPECT_NEAR(50.00, base.GetReliable(), 0.01);

   calib.AddSensor(PE::SBasicSensor(14,0.1));
   calib.AddSensor(PE::SBasicSensor(13,0.1));
   calib.AddSensor(PE::SBasicSensor(12,0.1));
   calib.AddSensor(PE::SBasicSensor(13,0.1)); //sensors still in a threshold limit
   calib.AddReference(PE::SBasicSensor(12,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(13,0.1)).IsValid() );
   EXPECT_NEAR(12.00, calib.GetSensor(PE::SBasicSensor(13,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib.GetSensor(PE::SBasicSensor(13,0.1)).Accuracy,0.01);
   EXPECT_NEAR(66.66, base.GetReliable(), 0.01);

   calib.AddSensor(PE::SBasicSensor(15,0.1));
   calib.AddSensor(PE::SBasicSensor(15,0.1)); //sensors still in a threshold limit
   calib.AddReference(PE::SBasicSensor(14,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(15,0.1)).IsValid() );
   EXPECT_NEAR(14.00, calib.GetSensor(PE::SBasicSensor(15,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib.GetSensor(PE::SBasicSensor(15,0.1)).Accuracy,0.01);
   EXPECT_NEAR(75.00, base.GetReliable(), 0.01);

   calib.AddSensor(PE::SBasicSensor(100,0.1));
   calib.AddSensor(PE::SBasicSensor(100,0.1));
   calib.AddSensor(PE::SBasicSensor(100,0.1));
   calib.AddSensor(PE::SBasicSensor(100,0.1)); //sensors still in a threshold limit
   calib.AddSensor(PE::SBasicSensor(100,0.1)); //sensors over ration +/-threshold limit and will be REJECTED
   calib.AddReference(PE::SBasicSensor(1,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(91,0.1)).IsValid() );
   EXPECT_NEAR(90.00, calib.GetSensor(PE::SBasicSensor(91,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib.GetSensor(PE::SBasicSensor(91,0.1)).Accuracy,0.01);
   EXPECT_NEAR(75.00, base.GetReliable(), 0.01);

   calib.AddReference(PE::SBasicSensor(100,0.1));
   calib.AddReference(PE::SBasicSensor(100,0.1));
   calib.AddSensor(PE::SBasicSensor( 0,0.1)); //sensors over ration +/-threshold limit and will be REJECTED
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(21,0.1)).IsValid() );
   EXPECT_NEAR(20.00, calib.GetSensor(PE::SBasicSensor(21,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib.GetSensor(PE::SBasicSensor(21,0.1)).Accuracy,0.01);
   EXPECT_NEAR(75.00, base.GetReliable(), 0.01);

   calib.AddSensor(PE::SBasicSensor(20,0.1));
   calib.AddSensor(PE::SBasicSensor(21,0.1));
   calib.AddSensor(PE::SBasicSensor(22,0.1));
   calib.AddReference(PE::SBasicSensor(20,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(21,0.1)).IsValid() );
   EXPECT_NEAR(20.00, calib.GetSensor(PE::SBasicSensor(21,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib.GetSensor(PE::SBasicSensor(21,0.1)).Accuracy,0.01);
   EXPECT_NEAR(80.00, base.GetReliable(), 0.01);
}


/**
 * tests
 */
TEST_F(PECCalibrationBaseTest, test_calibartion_raw_is_first_for_one_cycle_before_reference)
{
   PE::CNormalisation base;
   PE::CCalibrationBase   calib(&base, 1, 1);

   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -9,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -8,0.1));
   calib.AddReference(PE::SBasicSensor(   1,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -7,0.1));
   calib.AddReference(PE::SBasicSensor(   2,0.1));
   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   3,0.1));
   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(10,0.1)).IsValid() );
   EXPECT_NEAR(19.73, calib.GetSensor(PE::SBasicSensor(10,0.1)).Value,0.01);
   EXPECT_NEAR( 0.396, calib.GetSensor(PE::SBasicSensor(10,0.1)).Accuracy,0.001);
   EXPECT_NEAR(49.19, base.GetReliable(), 0.01);

   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -9,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -8,0.1));
   calib.AddReference(PE::SBasicSensor(   1,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -7,0.1));
   calib.AddReference(PE::SBasicSensor(   2,0.1));
   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   3,0.1));
   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(10,0.1)).IsValid() );
   EXPECT_NEAR(19.82, calib.GetSensor(PE::SBasicSensor(10,0.1)).Value,0.01);
   EXPECT_NEAR( 0.313, calib.GetSensor(PE::SBasicSensor(10,0.1)).Accuracy,0.001);
   EXPECT_NEAR(71.53, base.GetReliable(), 0.01);

   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -9,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -8,0.1));
   calib.AddReference(PE::SBasicSensor(   1,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -7,0.1));
   calib.AddReference(PE::SBasicSensor(   2,0.1));
   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   3,0.1));
   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(10,0.1)).IsValid() );
   EXPECT_NEAR(19.86, calib.GetSensor(PE::SBasicSensor(10,0.1)).Value,0.01);
   EXPECT_NEAR( 0.276, calib.GetSensor(PE::SBasicSensor(10,0.1)).Accuracy,0.001);
   EXPECT_NEAR(79.92, base.GetReliable(), 0.01);
}


/**
 * tests
 */
TEST_F(PECCalibrationBaseTest, test_calibartion_reference_is_first_for_one_cycle_before_raw)
{
   PE::CNormalisation base;
   PE::CCalibrationBase   calib(&base, 1, 1);

   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   1,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -9,0.1));
   calib.AddReference(PE::SBasicSensor(   2,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -8,0.1));
   calib.AddReference(PE::SBasicSensor(   3,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -7,0.1));
   calib.AddReference(PE::SBasicSensor(   3,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -7,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(10,0.1)).IsValid() );
   EXPECT_NEAR(20.36, calib.GetSensor(PE::SBasicSensor(10,0.1)).Value,0.01);
   EXPECT_NEAR( 0.379, calib.GetSensor(PE::SBasicSensor(10,0.1)).Accuracy,0.001);
   EXPECT_NEAR(49.90, base.GetReliable(), 0.01);

   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   1,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -9,0.1));
   calib.AddReference(PE::SBasicSensor(   2,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -8,0.1));
   calib.AddReference(PE::SBasicSensor(   3,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -7,0.1));
   calib.AddReference(PE::SBasicSensor(   3,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -7,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(10,0.1)).IsValid() );
   EXPECT_NEAR(20.24, calib.GetSensor(PE::SBasicSensor(10,0.1)).Value,0.01);
   EXPECT_NEAR( 0.314, calib.GetSensor(PE::SBasicSensor(10,0.1)).Accuracy,0.001);
   EXPECT_NEAR(71.50, base.GetReliable(), 0.01);

   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   1,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -9,0.1));
   calib.AddReference(PE::SBasicSensor(   2,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -8,0.1));
   calib.AddReference(PE::SBasicSensor(   3,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -7,0.1));
   calib.AddReference(PE::SBasicSensor(   3,0.1));
   calib.AddSensor   (PE::SBasicSensor(  -7,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   calib.AddSensor   (PE::SBasicSensor( -10,0.1));
   calib.AddReference(PE::SBasicSensor(   0,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(10,0.1)).IsValid() );
   EXPECT_NEAR(20.18, calib.GetSensor(PE::SBasicSensor(10,0.1)).Value,0.01);
   EXPECT_NEAR( 0.285, calib.GetSensor(PE::SBasicSensor(10,0.1)).Accuracy,0.001);
   EXPECT_NEAR(79.68, base.GetReliable(), 0.01);
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

