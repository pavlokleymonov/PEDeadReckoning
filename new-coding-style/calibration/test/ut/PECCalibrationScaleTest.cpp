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
 * Unit test of the PECCalibrationScaleTest class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PECNormalisationMock.h"
#include "PECCalibrationScale.h"

using ::testing::_;
using ::testing::Eq;
using ::testing::DoubleEq;
using ::testing::DoubleNear;
using ::testing::ByRef;


class PECCalibrationScaleTest : public ::testing::Test
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
TEST_F(PECCalibrationScaleTest, ref_above_limit_in_between_CB)
{
   PE::CNormalisationMock norm_scale;

   PE::CCalibrationScale calib(norm_scale,10); //10 meters accuracy

   //start accumulating of data
   calib.AddSensor(100);
   calib.AddReference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.AddSensor(100);
   calib.AddReference(100,11); //added 100 meters with accuracy above limit
   //resets all accumulated data
   calib.AddSensor(100);
   calib.AddReference(100,5); //added 100 meters with accuracy +/-5meters 
   //starts accumulating new data
   calib.AddSensor(400);
   calib.AddReference(60,5); //added 100 meters with accuracy +/-5meters 
   calib.AddSensor(600);
   //Expecte call has to be set before real call
   PE::TValue ratio = (400+600)/(60+40);
   EXPECT_CALL(norm_scale,AddSensor(Eq(ByRef(ratio))));
   calib.AddReference(40,5); //added 100 meters with accuracy +/-5meters 
}


/**
 * tests ignore all references values till first sensor data
 * callback check
 */
TEST_F(PECCalibrationScaleTest, ignore_unused_ref_data_CB)
{
   PE::CNormalisationMock norm_scale;

   PE::CCalibrationScale calib(norm_scale,10); //10 meters accuracy

   //start accumulating of data
   calib.AddReference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.AddReference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.AddReference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.AddReference(100,5); //added 100 meters with accuracy +/-5meters 
   //all reference values have to be ignored since no sensordata so far.
   calib.AddSensor(100);
   calib.AddReference(10,5); //added 10 meters with accuracy +/-5meters 
   calib.AddSensor(200);
   calib.AddReference(20,5); //added 20 meters with accuracy +/-5meters 
   calib.AddSensor(300);
   //Expecte call has to be set before real call
   PE::TValue ratio = (100+200+300)/(10+20+30);
   EXPECT_CALL(norm_scale,AddSensor(Eq(ByRef(ratio))));
   calib.AddReference(30,5); //added 30 meters with accuracy +/-5meters 
}


/**
 * tests 10 sensors 3 valid reference 2 invalid references
 * callback check
 */
TEST_F(PECCalibrationScaleTest, _10_sensors_3_valid_2_invalid_ref_test_CB)
{

   PE::CNormalisationMock norm_scale;

   PE::CCalibrationScale calib(norm_scale,10); //10 meters accuracy

   calib.AddSensor(1000); //has to be ignored before first valid ref
   calib.AddSensor(1000); //has to be ignored before first valid ref
   calib.AddReference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.AddSensor(500);
   calib.AddSensor(500);
   calib.AddSensor(500);
   calib.AddReference(100,11); //added 100 meters with accuracy above the limit
   calib.AddSensor(5000);
   calib.AddSensor(5000);
   //till now all sensors has to be ignored
   calib.AddReference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.AddSensor(500);
   calib.AddSensor(300);
   calib.AddSensor(200);
   calib.AddReference(10,5); //added 100 meters with accuracy +/-5meters 
   calib.AddSensor(100);
   calib.AddSensor(100);
   calib.AddSensor(100);
   //Expecte call has to be set before real call
   PE::TValue ratio = (500+300+200+100+100+100)/(10+3);
   EXPECT_CALL(norm_scale,AddSensor(Eq(ByRef(ratio))));
   calib.AddReference(3,5); //added 100 meters with accuracy +/-5meters 
}


/**
 * tests 6 sensors 3 valid reference
 * callback check
 */
TEST_F(PECCalibrationScaleTest, _6_sensors_3_valid_ref_test_CB)
{
   PE::CNormalisationMock mock;

   PE::CCalibrationScale calib(mock,10); //10 meters accuracy

   calib.AddSensor(1000); //has to be ignored before first valid ref
   calib.AddSensor(1000); //has to be ignored before first valid ref

   calib.AddReference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.AddSensor(500);
   calib.AddSensor(500);
   calib.AddReference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.AddSensor(500);
   calib.AddSensor(500);
   //Expecte call has to be set before real call
   PE::TValue ratio = (500+500+500+500)/(100+100);
   EXPECT_CALL(mock,AddSensor(Eq(ByRef(ratio))));
   calib.AddReference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.AddSensor(500);
   calib.AddSensor(500);
}

/**
 * tests 6 sensors 3 valid reference
 */
TEST_F(PECCalibrationScaleTest, _6_sensors_3_valid_ref_test)
{
   PE::CNormalisation norm;
   PE::CCalibrationScale calib(norm,10); //10 meters accuracy

   calib.AddSensor(1000); //has to be ignored before first valid ref
   calib.AddSensor(1000); //has to be ignored before first valid ref

   calib.AddReference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.AddSensor(500);
   calib.AddSensor(500);
   calib.AddReference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.AddSensor(500);
   calib.AddSensor(500);
   calib.AddReference(100,5); //added 100 meters with accuracy +/-5meters 

   EXPECT_EQ(10, calib.GetScale());
   EXPECT_EQ(0, calib.GetAccuracy());
   EXPECT_EQ(100, calib.GetCalibration());
   calib.AddSensor(500);
   calib.AddSensor(500);
}

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

