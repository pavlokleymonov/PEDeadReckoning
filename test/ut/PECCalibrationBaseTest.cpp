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
TEST_F(PECCalibrationBaseTest, test_create)
{
   PE::CNormalisation base;
   PE::CCalibrationBase   calib(base, 10, 1);

   EXPECT_FALSE( calib.GetSensor(PE::SBasicSensor()).IsValid() );
   EXPECT_NEAR(0.00, calib.CalibratedTo(), 0.01);
}


/**
 * tests  
 */
TEST_F(PECCalibrationBaseTest, test_no_reference)
{
   PE::CNormalisation base;
   PE::CCalibrationBase   calib(base, 10, 1);

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
TEST_F(PECCalibrationBaseTest, test_no_sensor)
{
   PE::CNormalisation base;
   PE::CCalibrationBase   calib(base, 3, 1);

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
TEST_F(PECCalibrationBaseTest, test_simple_calibartion)
{
   PE::CNormalisation base;
   PE::CCalibrationBase   calib(  base, 3, 1);

   calib.AddSensor(PE::SBasicSensor(1,0.1));
   calib.AddSensor(PE::SBasicSensor(2,0.1));
   calib.AddSensor(PE::SBasicSensor(3,0.1));
   calib.AddReference(PE::SBasicSensor(1,0.1));
   EXPECT_FALSE( calib.GetSensor(PE::SBasicSensor(2,0.1)).IsValid() );
   EXPECT_NEAR( 0.00, calib.CalibratedTo(), 0.01);

   calib.AddSensor(PE::SBasicSensor(11,0.1));
   calib.AddSensor(PE::SBasicSensor(11,0.1));
   calib.AddSensor(PE::SBasicSensor(11,0.1));
   calib.AddReference(PE::SBasicSensor(10,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(11,0.1)).IsValid() );
   EXPECT_NEAR( 10.00, calib.GetSensor(PE::SBasicSensor(11,0.1)).Value,0.01);
   EXPECT_NEAR(  0.10, calib.GetSensor(PE::SBasicSensor(11,0.1)).Accuracy,0.01);
   EXPECT_NEAR( 50.00, calib.CalibratedTo(), 0.01);

   calib.AddSensor(PE::SBasicSensor(181,0.1));
   calib.AddSensor(PE::SBasicSensor(182,0.1));
   calib.AddSensor(PE::SBasicSensor(183,0.1));
   calib.AddReference(PE::SBasicSensor(181,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(182,0.1)).IsValid() );
   EXPECT_NEAR(181.00, calib.GetSensor(PE::SBasicSensor(182,0.1)).Value,0.01);
   EXPECT_NEAR(  0.10, calib.GetSensor(PE::SBasicSensor(182,0.1)).Accuracy,0.01);
   EXPECT_NEAR( 66.66, calib.CalibratedTo(), 0.01);

   calib.AddSensor(PE::SBasicSensor(0,0.1));
   calib.AddSensor(PE::SBasicSensor(1,0.1));
   calib.AddSensor(PE::SBasicSensor(2,0.1));
   calib.AddReference(PE::SBasicSensor(0,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(0,0.1)).IsValid() );
   EXPECT_NEAR(-1.00, calib.GetSensor(PE::SBasicSensor(0,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib.GetSensor(PE::SBasicSensor(0,0.1)).Accuracy,0.01);
   EXPECT_NEAR(75.00, calib.CalibratedTo(), 0.01);

   calib.AddSensor(PE::SBasicSensor(100,0.1));
   calib.AddSensor(PE::SBasicSensor( 50,0.1));
   calib.AddSensor(PE::SBasicSensor(  0,0.1));
   calib.AddReference(PE::SBasicSensor(49,0.1));
   EXPECT_TRUE( calib.GetSensor(PE::SBasicSensor(51,0.1)).IsValid() );
   EXPECT_NEAR(50.00, calib.GetSensor(PE::SBasicSensor(51,0.1)).Value,0.01);
   EXPECT_NEAR( 0.10, calib.GetSensor(PE::SBasicSensor(51,0.1)).Accuracy,0.01);
   EXPECT_NEAR(80.00, calib.CalibratedTo(), 0.01);
}

//TODO check with cycle shift
//TODO check with out of ratio

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

