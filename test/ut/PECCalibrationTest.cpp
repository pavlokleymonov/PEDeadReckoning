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
 * Unit test of the PECCalibration class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PECCalibration.h"


class PECCalibrationTest : public ::testing::Test
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
TEST_F(PECCalibrationTest, test_create)
{
   PE::CNormalisation scale;
   PE::CNormalisation positive;
   PE::CNormalisation negative;
   PE::CCalibration   calib(scale,positive,negative,10);

   EXPECT_NEAR(0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetScaleReliable(), 0.000001);

   EXPECT_NEAR(0.000000, calib.GetBasePositive(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetBasePositiveMld(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetBasePositiveReliable(), 0.000001);

   EXPECT_NEAR(0.000000, calib.GetBaseNegative(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetBaseNegativeMld(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetBaseNegativeReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationTest, test_behinde_limit)
{
   PE::CNormalisation scale;
   PE::CNormalisation positive;
   PE::CNormalisation negative;
   PE::CCalibration   calib(scale,positive,negative,10);

   calib.AddReference(1);
   calib.AddSensor(10);
   calib.AddReference(2);
   calib.AddSensor(20);
   calib.AddReference(3);
   calib.AddSensor(30);
   calib.AddReference(4);
   calib.AddSensor(40);
   calib.AddReference(-1);
   calib.AddSensor(-10);
   calib.AddReference(-2);
   calib.AddSensor(-20);
   calib.AddReference(-3);
   calib.AddSensor(-30);

   EXPECT_NEAR(0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetScaleReliable(), 0.000001);

   EXPECT_NEAR(0.000000, calib.GetBasePositive(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetBasePositiveMld(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetBasePositiveReliable(), 0.000001);

   EXPECT_NEAR(0.000000, calib.GetBaseNegative(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetBaseNegativeMld(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetBaseNegativeReliable(), 0.000001);
}


/**
 * tests 
 */
TEST_F(PECCalibrationTest, test_above_limit_positive)
{
   PE::CNormalisation scale;
   PE::CNormalisation positive;
   PE::CNormalisation negative;
   PE::CCalibration   calib(scale,positive,negative,0);

   calib.AddReference(1);
   calib.AddSensor(10);
   calib.AddReference(2);
   calib.AddSensor(20);
   calib.AddReference(3);
   calib.AddSensor(30);
   calib.AddReference(4);
   calib.AddSensor(40);
   calib.AddReference(5);
   calib.AddSensor(50);
   calib.AddReference(6);
   calib.AddSensor(60);
   calib.AddReference(7);
   calib.AddSensor(70);

   EXPECT_NEAR( 0.1, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.0, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(42.857142, calib.GetScaleReliable(), 0.000001);

   EXPECT_NEAR( 0.000000, calib.GetBasePositive(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBasePositiveMld(), 0.000001);
   EXPECT_NEAR(85.714285, calib.GetBasePositiveReliable(), 0.000001);

   EXPECT_NEAR( 0.000000, calib.GetBaseNegative(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseNegativeMld(), 0.000001);
   EXPECT_NEAR(00.000000, calib.GetBaseNegativeReliable(), 0.000001);
}


/**
 * tests 
 */
TEST_F(PECCalibrationTest, test_above_limit_negative)
{
   PE::CNormalisation scale;
   PE::CNormalisation positive;
   PE::CNormalisation negative;
   PE::CCalibration   calib(scale,positive,negative,0);

   calib.AddReference(-1);
   calib.AddSensor(-10);
   calib.AddReference(-2);
   calib.AddSensor(-20);
   calib.AddReference(-3);
   calib.AddSensor(-30);
   calib.AddReference(-4);
   calib.AddSensor(-40);
   calib.AddReference(-5);
   calib.AddSensor(-50);
   calib.AddReference(-6);
   calib.AddSensor(-60);
   calib.AddReference(-7);
   calib.AddSensor(-70);

   EXPECT_NEAR( 0.1, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.0, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(42.857142, calib.GetScaleReliable(), 0.000001);

   EXPECT_NEAR( 0.000000, calib.GetBasePositive(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBasePositiveMld(), 0.000001);
   EXPECT_NEAR(00.000000, calib.GetBasePositiveReliable(), 0.000001);

   EXPECT_NEAR( 0.000000, calib.GetBaseNegative(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseNegativeMld(), 0.000001);
   EXPECT_NEAR(85.714285, calib.GetBaseNegativeReliable(), 0.000001);
}


/**
 * tests 
 */
TEST_F(PECCalibrationTest, test_above_limit)
{
   PE::CNormalisation scale;
   PE::CNormalisation positive;
   PE::CNormalisation negative;
   PE::CCalibration   calib(scale,positive,negative,0);

   calib.AddReference(-1);
   calib.AddSensor(-10);
   calib.AddReference(-2);
   calib.AddSensor(-20);
   calib.AddReference(-3);
   calib.AddSensor(-30);
   calib.AddReference(-4);
   calib.AddSensor(-40);
   calib.AddReference(5);
   calib.AddSensor(50);
   calib.AddReference(6);
   calib.AddSensor(60);
   calib.AddReference(7);
   calib.AddSensor(70);

   EXPECT_NEAR( 0.1, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.0, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(42.857142, calib.GetScaleReliable(), 0.000001);

   EXPECT_NEAR( 0.000000, calib.GetBasePositive(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBasePositiveMld(), 0.000001);
   EXPECT_NEAR(66.666667, calib.GetBasePositiveReliable(), 0.000001);

   EXPECT_NEAR( 0.000000, calib.GetBaseNegative(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseNegativeMld(), 0.000001);
   EXPECT_NEAR(85.714285, calib.GetBaseNegativeReliable(), 0.000001);
}


/**
 * tests 
 */
TEST_F(PECCalibrationTest, test_same_value)
{
   PE::CNormalisation scale;
   PE::CNormalisation positive;
   PE::CNormalisation negative;
   PE::CCalibration   calib(scale,positive,negative,0);

   calib.AddReference(1.001);
   calib.AddSensor(10.01);
   EXPECT_NEAR( 0.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.0, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(00.000000, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBasePositive(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBasePositiveMld(), 0.000001);
   EXPECT_NEAR(00.000000, calib.GetBasePositiveReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseNegative(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseNegativeMld(), 0.000001);
   EXPECT_NEAR(00.000000, calib.GetBaseNegativeReliable(), 0.000001);

   calib.AddReference(1.001);//if you will use 1 and 10 reliable will be decrease, probabli because inaccuracy for int to float convertion
   calib.AddSensor(10.01);
   EXPECT_NEAR( 0.1, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.0, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(50.000000, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBasePositive(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBasePositiveMld(), 0.000001);
   EXPECT_NEAR(50.000000, calib.GetBasePositiveReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseNegative(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseNegativeMld(), 0.000001);
   EXPECT_NEAR(00.000000, calib.GetBaseNegativeReliable(), 0.000001);

   calib.AddReference(1.001);
   calib.AddSensor(10.01);
   EXPECT_NEAR( 0.1, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.0, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(66.666666, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBasePositive(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBasePositiveMld(), 0.000001);
   EXPECT_NEAR(66.666666, calib.GetBasePositiveReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseNegative(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseNegativeMld(), 0.000001);
   EXPECT_NEAR(00.000000, calib.GetBaseNegativeReliable(), 0.000001);

   calib.AddReference(1.001);
   calib.AddSensor(10.01);
   EXPECT_NEAR( 0.1, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.0, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(75.000000, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBasePositive(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBasePositiveMld(), 0.000001);
   EXPECT_NEAR(75.000000, calib.GetBasePositiveReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseNegative(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseNegativeMld(), 0.000001);
   EXPECT_NEAR(00.000000, calib.GetBaseNegativeReliable(), 0.000001);
}

#ifdef TTT

/**
 * tests 
 */
TEST_F(PECCalibrationTest, test_negative_base)
{
   PE::CNormalisation scale;
   PE::CNormalisation positive;
   PE::CNormalisation negative;
   PE::CCalibration   calib(scale,positive,negative,0);

   calib.AddReference(10);
   calib.AddSensor(20);
   calib.AddReference(20);
   calib.AddSensor(30);
   calib.AddReference(30);
   calib.AddSensor(40);
   calib.AddReference(40);
   calib.AddSensor(50);
   calib.AddReference(50);
   calib.AddSensor(60);
   calib.AddReference(60);
   calib.AddSensor(70);
   calib.AddReference(70);
   calib.AddSensor(80);

   EXPECT_NEAR( 1.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.0, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(00.000000, calib.GetScaleReliable(), 0.000001);

   EXPECT_NEAR(-10.000000, calib.GetBasePositive(), 0.000001);
   EXPECT_NEAR(  0.000000, calib.GetBasePositiveMld(), 0.000001);
   EXPECT_NEAR( 00.000000, calib.GetBasePositiveReliable(), 0.000001);

   EXPECT_NEAR(-10.000000, calib.GetBaseNegative(), 0.000001);
   EXPECT_NEAR(  0.000000, calib.GetBaseNegativeMld(), 0.000001);
   EXPECT_NEAR( 00.000000, calib.GetBaseNegativeReliable(), 0.000001);
}
























/**
 * tests ignore all references values till first sensor data
 * callback check
 */
TEST_F(PECCalibrationTest, ignore_unused_ref_data_CB)
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
TEST_F(PECCalibrationTest, _10_sensors_3_valid_2_invalid_ref_test_CB)
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
TEST_F(PECCalibrationTest, _6_sensors_3_valid_ref_test_CB)
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
TEST_F(PECCalibrationTest, _6_sensors_3_valid_ref_test)
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

#endif

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

