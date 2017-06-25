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
 * Unit test of the PECalibrationTest class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "mock_PENormalisation.h"
#include "PECalibrationScale.h"

using ::testing::_;
using ::testing::Eq;
using ::testing::ByRef;


class PECalibrationScaleTest : public ::testing::Test
{
public:
   virtual void SetUp()
   {}
   virtual void TearDown()
   {}
};

/**
 * tests 10 sensors 3 valid reference 2 invalid references
 * callback check
 */
TEST_F(PECalibrationScaleTest, _10_sensors_3_valid_2_invalid_ref_test_CB)
{
//TODO
   PE::Mock_normalisation mock;

   PE::calibration_scale calib(mock,10); //10 meters accuracy

   calib.add_sensor(1000); //has to be ignored before first valid ref
   calib.add_sensor(1000); //has to be ignored before first valid ref

   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.add_sensor(500);
   calib.add_sensor(500);
   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.add_sensor(500);
   calib.add_sensor(500);
   //Expecte call has to be set before real call
   PE::TValue ratio = (500+500+500+500)/(100+100);
   EXPECT_CALL(mock,add_sensor(Eq(ByRef(ratio))));
   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.add_sensor(500);
   calib.add_sensor(500);
}



/**
 * tests 6 sensors 3 valid reference
 * callback check
 */
TEST_F(PECalibrationScaleTest, _6_sensors_3_valid_ref_test_CB)
{
   PE::Mock_normalisation mock;

   PE::calibration_scale calib(mock,10); //10 meters accuracy

   calib.add_sensor(1000); //has to be ignored before first valid ref
   calib.add_sensor(1000); //has to be ignored before first valid ref

   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.add_sensor(500);
   calib.add_sensor(500);
   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.add_sensor(500);
   calib.add_sensor(500);
   //Expecte call has to be set before real call
   PE::TValue ratio = (500+500+500+500)/(100+100);
   EXPECT_CALL(mock,add_sensor(Eq(ByRef(ratio))));
   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.add_sensor(500);
   calib.add_sensor(500);
}

/**
 * tests 6 sensors 3 valid reference
 */
TEST_F(PECalibrationScaleTest, _6_sensors_3_valid_ref_test)
{
   PE::normalisation norm;
   PE::calibration_scale calib(norm,10); //10 meters accuracy

   calib.add_sensor(1000); //has to be ignored before first valid ref
   calib.add_sensor(1000); //has to be ignored before first valid ref

   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.add_sensor(500);
   calib.add_sensor(500);
   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.add_sensor(500);
   calib.add_sensor(500);
   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 

   EXPECT_EQ(10, calib.get_scale());
   EXPECT_EQ(0, calib.get_accuracy());
   EXPECT_EQ(100, calib.get_calibration());
   calib.add_sensor(500);
   calib.add_sensor(500);
}

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

