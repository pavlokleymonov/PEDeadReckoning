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
 * Unit test of the PECCalibrationBaseTest class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
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
 * ignore sensor data because no reference
 */
TEST_F(PECCalibrationBaseTest, ignore_sensor_because_no_ref_data )
{
   PE::CNormalisation norm;

   PE::CCalibrationBase calib(norm,10); //10 meters accuracy

   calib.AddSensor(100);
   calib.AddSensor(100);
   calib.AddSensor(100);
   calib.AddSensor(100);
   EXPECT_EQ(0,calib.GetBase());
   EXPECT_EQ(0,calib.GetCalibration());
}


/**
 * ignore sensor data because reference above limit
 */
TEST_F(PECCalibrationBaseTest, ignore_sensor_because_ref_data_above_limit )
{
   PE::CNormalisation norm;

   const PE::TValue ACC_LIMIT = 10; //10 meters accuracy

   PE::CCalibrationBase calib(norm,ACC_LIMIT);

   calib.AddReference(0, ACC_LIMIT+1);
   calib.AddSensor(100);
   calib.AddSensor(100);
   calib.AddSensor(100);
   calib.AddSensor(100);
   calib.AddReference(0, ACC_LIMIT+1);
   EXPECT_EQ(0,calib.GetBase());
   EXPECT_EQ(0,calib.GetCalibration());
}

/**
 * ignore sensor data because reference is not a zero
 */
TEST_F(PECCalibrationBaseTest, ignore_sensor_because_ref_data_not_zero )
{
   PE::CNormalisation norm;

   const PE::TValue ACC_LIMIT = 10; //10 meters accuracy

   PE::CCalibrationBase calib(norm,ACC_LIMIT);

   calib.AddReference(1, ACC_LIMIT-1);
   calib.AddSensor(100);
   calib.AddSensor(100);
   calib.AddSensor(100);
   calib.AddSensor(100);
   calib.AddReference(1, ACC_LIMIT-1);
   EXPECT_EQ(0,calib.GetBase());
   EXPECT_EQ(0,calib.GetCalibration());
}

/**
 * calculate base 200 since all conditions are satisfied
 */
TEST_F(PECCalibrationBaseTest, base_222_all_cond_satisfied )
{
   PE::CNormalisation norm;

   const PE::TValue ACC_LIMIT = 10; //10 meters accuracy

   PE::CCalibrationBase calib(norm,ACC_LIMIT);

   calib.AddReference(0, ACC_LIMIT-1);
   calib.AddSensor(150);
   calib.AddSensor(250);
   calib.AddSensor(200);
   calib.AddReference(0, ACC_LIMIT-1);
   calib.AddSensor(180);
   calib.AddSensor(190);
   calib.AddSensor(200);
   calib.AddSensor(230);
   calib.AddReference(0, ACC_LIMIT-1);
   EXPECT_EQ(200,calib.GetBase());
   EXPECT_NEAR(0,calib.GetAccuracy(),0.1);
   EXPECT_NEAR(100,calib.GetCalibration(),0.1);
}

/**
 * ignore sensors since there is no coherently valid reference value
 * two times accuracy limit was not setisfied
 */
TEST_F(PECCalibrationBaseTest, ignore_no_coherently_valid_ref_by_accuracy )
{
   PE::CNormalisation norm;

   const PE::TValue ACC_LIMIT = 10; //10 meters accuracy

   PE::CCalibrationBase calib(norm,ACC_LIMIT);

   calib.AddReference(0, ACC_LIMIT); //accuracy limit was not setisfied
   calib.AddSensor(150);
   calib.AddSensor(250);
   calib.AddSensor(200);
   calib.AddReference(0, ACC_LIMIT-1);
   calib.AddSensor(180);
   calib.AddSensor(190);
   calib.AddSensor(200);
   calib.AddSensor(230);
   calib.AddReference(0, ACC_LIMIT); //accuracy limit was not setisfied
   EXPECT_EQ(0,calib.GetBase());
   EXPECT_EQ(0,calib.GetCalibration());
}

/**
 * ignore sensors since there is no coherently valid reference value
 * two times reference value is not zero
 */
TEST_F(PECCalibrationBaseTest, ignore_no_coherently_valid_ref_by_above_zero )
{
   PE::CNormalisation norm;

   const PE::TValue ACC_LIMIT = 10; //10 meters accuracy

   PE::CCalibrationBase calib(norm,ACC_LIMIT);

   calib.AddReference(1, ACC_LIMIT-1); //reference value is not zero
   calib.AddSensor(150);
   calib.AddSensor(250);
   calib.AddSensor(200);
   calib.AddReference(0, ACC_LIMIT-1);
   calib.AddSensor(180);
   calib.AddSensor(190);
   calib.AddSensor(200);
   calib.AddSensor(230);
   calib.AddReference(1, ACC_LIMIT-1); //reference value is not zero
   EXPECT_EQ(0,calib.GetBase());
   EXPECT_EQ(0,calib.GetCalibration());
}

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

