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
 * Unit test of the PECalibrationBaseTest class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include "PECalibrationBase.h"

class PECalibrationBaseTest : public ::testing::Test
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
TEST_F(PECalibrationBaseTest, ignore_sensor_because_no_ref_data )
{
   PE::normalisation norm;

   PE::calibration_base calib(norm,10); //10 meters accuracy

   calib.add_sensor(100);
   calib.add_sensor(100);
   calib.add_sensor(100);
   calib.add_sensor(100);
   EXPECT_EQ(0,calib.get_base());
   EXPECT_EQ(0,calib.get_calibration());
}


/**
 * ignore sensor data because reference above limit
 */
TEST_F(PECalibrationBaseTest, ignore_sensor_because_ref_data_above_limit )
{
   PE::normalisation norm;

   const PE::TValue ACC_LIMIT = 10; //10 meters accuracy

   PE::calibration_base calib(norm,ACC_LIMIT);

   calib.add_reference(0, ACC_LIMIT+1);
   calib.add_sensor(100);
   calib.add_sensor(100);
   calib.add_sensor(100);
   calib.add_sensor(100);
   calib.add_reference(0, ACC_LIMIT+1);
   EXPECT_EQ(0,calib.get_base());
   EXPECT_EQ(0,calib.get_calibration());
}

/**
 * ignore sensor data because reference is not a zero
 */
TEST_F(PECalibrationBaseTest, ignore_sensor_because_ref_data_not_zero )
{
   PE::normalisation norm;

   const PE::TValue ACC_LIMIT = 10; //10 meters accuracy

   PE::calibration_base calib(norm,ACC_LIMIT);

   calib.add_reference(1, ACC_LIMIT-1);
   calib.add_sensor(100);
   calib.add_sensor(100);
   calib.add_sensor(100);
   calib.add_sensor(100);
   calib.add_reference(1, ACC_LIMIT-1);
   EXPECT_EQ(0,calib.get_base());
   EXPECT_EQ(0,calib.get_calibration());
}

/**
 * calculate base 200 since all conditions are satisfied
 */
TEST_F(PECalibrationBaseTest, base_222_all_cond_satisfied )
{
   PE::normalisation norm;

   const PE::TValue ACC_LIMIT = 10; //10 meters accuracy

   PE::calibration_base calib(norm,ACC_LIMIT);

   calib.add_reference(0, ACC_LIMIT-1);
   calib.add_sensor(150);
   calib.add_sensor(250);
   calib.add_sensor(200);
   calib.add_reference(0, ACC_LIMIT-1);
   calib.add_sensor(180);
   calib.add_sensor(190);
   calib.add_sensor(200);
   calib.add_sensor(230);
   calib.add_reference(0, ACC_LIMIT-1);
   EXPECT_EQ(200,calib.get_base());
   EXPECT_NEAR(0,calib.get_accuracy(),0.1);
   EXPECT_NEAR(100,calib.get_calibration(),0.1);
}

/**
 * ignore sensors since there is no coherently valid reference value
 * two times accuracy limit was not setisfied
 */
TEST_F(PECalibrationBaseTest, ignore_no_coherently_valid_ref_by_accuracy )
{
   PE::normalisation norm;

   const PE::TValue ACC_LIMIT = 10; //10 meters accuracy

   PE::calibration_base calib(norm,ACC_LIMIT);

   calib.add_reference(0, ACC_LIMIT); //accuracy limit was not setisfied
   calib.add_sensor(150);
   calib.add_sensor(250);
   calib.add_sensor(200);
   calib.add_reference(0, ACC_LIMIT-1);
   calib.add_sensor(180);
   calib.add_sensor(190);
   calib.add_sensor(200);
   calib.add_sensor(230);
   calib.add_reference(0, ACC_LIMIT); //accuracy limit was not setisfied
   EXPECT_EQ(0,calib.get_base());
   EXPECT_EQ(0,calib.get_calibration());
}

/**
 * ignore sensors since there is no coherently valid reference value
 * two times reference value is not zero
 */
TEST_F(PECalibrationBaseTest, ignore_no_coherently_valid_ref_by_above_zero )
{
   PE::normalisation norm;

   const PE::TValue ACC_LIMIT = 10; //10 meters accuracy

   PE::calibration_base calib(norm,ACC_LIMIT);

   calib.add_reference(1, ACC_LIMIT-1); //reference value is not zero
   calib.add_sensor(150);
   calib.add_sensor(250);
   calib.add_sensor(200);
   calib.add_reference(0, ACC_LIMIT-1);
   calib.add_sensor(180);
   calib.add_sensor(190);
   calib.add_sensor(200);
   calib.add_sensor(230);
   calib.add_reference(1, ACC_LIMIT-1); //reference value is not zero
   EXPECT_EQ(0,calib.get_base());
   EXPECT_EQ(0,calib.get_calibration());
}

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

