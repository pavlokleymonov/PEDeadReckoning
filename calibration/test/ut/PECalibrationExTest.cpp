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
 * Unit test of the PECalibrationEx class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "mock_PENormalisation.h"
#include "PECalibrationEx.h"

using ::testing::_;
using ::testing::Eq;
using ::testing::DoubleEq;
using ::testing::DoubleNear;
using ::testing::ByRef;


class PECalibrationExTest : public ::testing::Test
{
public:
   virtual void SetUp()
   {}
   virtual void TearDown()
   {}
};


/**
 * tests reference value above accuracy limit inside incoming data
 * callback check
 */
TEST_F(PECalibrationExTest, ref_above_limit_in_between_CB)
{
   PE::Mock_normalisation mock;

   PE::calibration_ex calib(mock,10); //10 meters accuracy

   //start accumulating of data
   calib.add_sensor(100);
   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.add_sensor(100);
   calib.add_reference(100,11); //added 100 meters with accuracy above limit
   //resets all accumulated data
   calib.add_sensor(100);
   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   //starts accumulating new data
   calib.add_sensor(400);
   calib.add_reference(60,5); //added 100 meters with accuracy +/-5meters 
   calib.add_sensor(600);
   //Expecte call has to be set before real call
   PE::TValue ratio = (400+600)/(60+40);
   EXPECT_CALL(mock,add_sensor(Eq(ByRef(ratio))));
   calib.add_reference(40,5); //added 100 meters with accuracy +/-5meters 
}


/**
 * tests ignore all references values till first sensor data
 * callback check
 */
TEST_F(PECalibrationExTest, ignore_unused_ref_data_CB)
{
   PE::Mock_normalisation mock;

   PE::calibration_ex calib(mock,10); //10 meters accuracy

   //start accumulating of data
   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   //all reference values have to be ignored since no sensordata so far.
   calib.add_sensor(100);
   calib.add_reference(10,5); //added 10 meters with accuracy +/-5meters 
   calib.add_sensor(200);
   calib.add_reference(20,5); //added 20 meters with accuracy +/-5meters 
   calib.add_sensor(300);
   //Expecte call has to be set before real call
   PE::TValue ratio = (100+200+300)/(10+20+30);
   EXPECT_CALL(mock,add_sensor(Eq(ByRef(ratio))));
   calib.add_reference(30,5); //added 30 meters with accuracy +/-5meters 
}


/**
 * tests 10 sensors 3 valid reference 2 invalid references
 * callback check
 */
TEST_F(PECalibrationExTest, _10_sensors_3_valid_2_invalid_ref_test_CB)
{

   PE::Mock_normalisation mock;

   PE::calibration_ex calib(mock,10); //10 meters accuracy

   calib.add_sensor(1000); //has to be ignored before first valid ref
   calib.add_sensor(1000); //has to be ignored before first valid ref
   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.add_sensor(500);
   calib.add_sensor(500);
   calib.add_sensor(500);
   calib.add_reference(100,11); //added 100 meters with accuracy above the limit
   calib.add_sensor(5000);
   calib.add_sensor(5000);
   //till now all sensors has to be ignored
   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.add_sensor(500);
   calib.add_sensor(300);
   calib.add_sensor(200);
   calib.add_reference(10,5); //added 100 meters with accuracy +/-5meters 
   calib.add_sensor(100);
   calib.add_sensor(100);
   calib.add_sensor(100);
   //Expecte call has to be set before real call
   PE::TValue ratio = (500+300+200+100+100+100)/(10+3);
   EXPECT_CALL(mock,add_sensor(Eq(ByRef(ratio))));
   calib.add_reference(3,5); //added 100 meters with accuracy +/-5meters 
}



/**
 * tests 6 sensors 3 valid reference
 * callback check
 */
TEST_F(PECalibrationExTest, _6_sensors_3_valid_ref_test_CB)
{
   PE::Mock_normalisation mock;

   PE::calibration_ex calib(mock,10); //10 meters accuracy

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
TEST_F(PECalibrationExTest, _6_sensors_3_valid_ref_test)
{
   PE::normalisation norm;
   PE::calibration_ex calib(norm,10); //10 meters accuracy

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

