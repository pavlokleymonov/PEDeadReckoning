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
   PE::CNormalisation base;
   PE::CCalibration   calib(scale,base);

   EXPECT_NEAR(0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetScaleReliable(), 0.000001);

   EXPECT_NEAR(0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetBaseReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationTest, test_empty_base_and_scale)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;

   PE::CCalibration   calib(scale,base);
   EXPECT_NEAR(0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetBaseReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationTest, test_empty_scale)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;

   base.AddSensor(10);
   base.AddSensor(10);
   base.AddSensor(10);
   base.AddSensor(10);
   EXPECT_NEAR(10.000000, base.GetMean(), 0.000001);
   PE::CCalibration   calib(scale,base);
   EXPECT_NEAR( 0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(10.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR(75.000000, calib.GetBaseReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationTest, test_empty_base)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;

   scale.AddSensor(10);
   scale.AddSensor(10);
   scale.AddSensor(10);
   scale.AddSensor(10);
   EXPECT_NEAR(10.000000, scale.GetMean(), 0.000001);
   PE::CCalibration   calib(scale,base);
   EXPECT_NEAR(10.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(75.000000, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationTest, test_do_calibration_no_sensors_no_reference)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;

   scale.AddSensor(10);
   scale.AddSensor(10);
   scale.AddSensor(10);
   scale.AddSensor(10);
   EXPECT_NEAR(10.000000, scale.GetMean(), 0.000001);
   PE::CCalibration   calib(scale,base);
   calib.DoCalibration();
   calib.DoCalibration();
   calib.DoCalibration();
   calib.DoCalibration();
   calib.DoCalibration();
   EXPECT_NEAR(10.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(75.000000, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseReliable(), 0.000001);
}


/**
 * tests
 */
TEST_F(PECCalibrationTest, test_valid_scale_but_no_sensor_data)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;

   scale.AddSensor(10);
   scale.AddSensor(10);
   scale.AddSensor(10);
   scale.AddSensor(10);
   EXPECT_NEAR(10.000000, scale.GetMean(), 0.000001);
   PE::CCalibration   calib(scale,base);
   calib.AddReference(100000);
   calib.DoCalibration();
   calib.AddReference(200000);
   calib.DoCalibration();
   calib.AddReference(100000);
   calib.DoCalibration();
   calib.AddReference(200000);
   calib.DoCalibration();
   //reference data without sensors data is not considered
   EXPECT_NEAR(10.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(75.000000, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationTest, test_valid_scale_but_no_reference_data)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;

   scale.AddSensor(10);
   scale.AddSensor(10);
   scale.AddSensor(10);
   scale.AddSensor(10);
   EXPECT_NEAR(10.000000, scale.GetMean(), 0.000001);
   PE::CCalibration   calib(scale,base);
   calib.AddSensor(100000);
   calib.DoCalibration();
   calib.AddSensor(200000);
   calib.DoCalibration();
   calib.AddSensor(100000);
   calib.DoCalibration();
   calib.AddSensor(200000);
   calib.DoCalibration();
   //reference data without sensors data is not considered
   EXPECT_NEAR(10.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(75.000000, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationTest, test_no_reference_data)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;

   PE::CCalibration   calib(scale,base);
   calib.AddSensor(1000);
   calib.DoCalibration();
   calib.AddSensor(1000);
   calib.DoCalibration();
   calib.AddSensor(1000);
   calib.DoCalibration();
   calib.AddSensor(1000);
   calib.DoCalibration();
   EXPECT_NEAR( 0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationTest, test_no_sensor_data)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;

   PE::CCalibration   calib(scale,base);
   calib.AddReference(1000);
   calib.DoCalibration();
   calib.AddReference(1000);
   calib.DoCalibration();
   calib.AddReference(1000);
   calib.DoCalibration();
   calib.AddReference(1000);
   calib.DoCalibration();
   EXPECT_NEAR( 0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationTest, test_same_reference_data)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;

   PE::CCalibration   calib(scale,base);
   calib.AddSensor(100.1);
   calib.AddReference(1000.1);
   calib.DoCalibration();
   calib.AddSensor(200.1);
   calib.AddReference(1000.1);
   calib.DoCalibration();
   calib.AddSensor(100.1);
   calib.AddReference(1000.1);
   calib.DoCalibration();
   calib.AddSensor(200.1);
   calib.AddReference(1000.1);
   calib.DoCalibration();
   EXPECT_NEAR( 0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationTest, test_same_sensor_data)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;

   PE::CCalibration   calib(scale,base);
   calib.AddSensor(100.1);
   calib.AddReference(1000.1);
   calib.DoCalibration();
   calib.AddSensor(100.1);
   calib.AddReference(2000.1);
   calib.DoCalibration();
   calib.AddSensor(100.1);
   calib.AddReference(1000.1);
   calib.DoCalibration();
   calib.AddSensor(100.1);
   calib.AddReference(2000.1);
   calib.DoCalibration();
   EXPECT_NEAR( 0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationTest, test_same_reference_and_sensor_data)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;

   PE::CCalibration   calib(scale,base);
   calib.AddSensor(100.1);
   calib.AddReference(1000.1);
   calib.DoCalibration();
   calib.AddSensor(100.1);
   calib.AddReference(1000.1);
   calib.DoCalibration();
   calib.AddSensor(100.1);
   calib.AddReference(1000.1);
   calib.DoCalibration();
   calib.AddSensor(100.1);
   calib.AddReference(1000.1);
   calib.DoCalibration();
   EXPECT_NEAR( 0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseReliable(), 0.000001);
}


/**
 * tests 
 */
TEST_F(PECCalibrationTest, test_sen_10_to_10K_ref_1_to_1K)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;
   PE::CCalibration   calib(scale,base);

   for (int i=1;i<1000;++i)
   {
      calib.AddReference(i);
      calib.AddSensor(i*10-1);
      calib.DoCalibration();
   }

   EXPECT_NEAR( 0.1, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.0, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(96.915807, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(-0.100000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR(98.258756, calib.GetBaseReliable(), 0.000001);
}


/**
 * tests 
 */
TEST_F(PECCalibrationTest, test_sen_minus10_to_plus90_ref_0_to_100)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;
   PE::CCalibration   calib(scale,base);

   calib.AddReference(0);
   calib.AddSensor(-10);
   calib.DoCalibration();
   calib.AddReference(10);
   calib.AddSensor(0);
   calib.DoCalibration();
   calib.AddReference(50);
   calib.AddSensor(40);
   calib.DoCalibration();
   calib.AddReference(60);
   calib.AddSensor(50);
   calib.DoCalibration();
   calib.AddReference(100);
   calib.AddSensor(90);
   calib.DoCalibration();
   EXPECT_NEAR(  1.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR( 75.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(-10.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR( 66.666667, calib.GetBaseReliable(), 0.000001);
}


/**
 * tests 
 */
TEST_F(PECCalibrationTest, test_sen_90_to_30_ref_100_to_40)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;
   PE::CCalibration   calib(scale,base);

   calib.AddReference(100);
   calib.AddSensor(90);
   calib.DoCalibration();
   calib.AddReference(60);
   calib.AddSensor(50);
   calib.DoCalibration();
   calib.AddReference(50);
   calib.AddSensor(40);
   calib.DoCalibration();
   calib.AddReference(45);
   calib.AddSensor(35);
   calib.DoCalibration();
   calib.AddReference(40);
   calib.AddSensor(30);
   calib.DoCalibration();
   EXPECT_NEAR(  1.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 75.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(-10.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 66.666667, calib.GetBaseReliable(), 0.000001);
}


/**
 * tests 
 */
TEST_F(PECCalibrationTest, test_sen_minus10_to_plus10_ref_minus20_to_plus180)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;
   PE::CCalibration   calib(scale,base);

   calib.AddReference(180);
   calib.AddSensor(10);
   calib.DoCalibration();
   calib.AddReference(80);
   calib.AddSensor(0);
   calib.DoCalibration();
   calib.AddReference(40);
   calib.AddSensor(-4);
   calib.DoCalibration();
   calib.AddReference(40);
   calib.AddSensor(-4);
   calib.DoCalibration();
   calib.AddReference(10);
   calib.AddSensor(-7);
   calib.DoCalibration();
   calib.AddReference(20);
   calib.AddSensor(-6);
   calib.DoCalibration();
   calib.AddReference(20);
   calib.AddSensor(-6);
   calib.DoCalibration();
   calib.AddReference(0);
   calib.AddSensor(-8);
   calib.DoCalibration();
   calib.AddReference(-20);
   calib.AddSensor(-10);
   calib.DoCalibration();
   EXPECT_NEAR(  10.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  83.333333, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( -80.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR(  85.714285, calib.GetBaseReliable(), 0.000001);
}

/**
 * tests
 */
TEST_F(PECCalibrationTest, test_negative_sen_and_positive_ref)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;
   PE::CCalibration   calib(scale,base);

   calib.AddReference(5.0);
   calib.AddSensor(-1000);
   calib.DoCalibration();
   calib.AddReference(5.5);
   calib.AddSensor(-900);
   calib.DoCalibration();
   calib.AddReference(6.0);
   calib.AddSensor(-800);
   calib.DoCalibration();
   calib.AddReference(9.5);
   calib.AddSensor(-100);
   calib.DoCalibration();
   calib.AddReference(9.0);
   calib.AddSensor(-200);
   calib.DoCalibration();
   calib.AddReference(9.0);
   calib.AddSensor(-200);
   calib.DoCalibration();
   calib.AddReference(8.5);
   calib.AddSensor(-300);
   calib.DoCalibration();
   calib.AddReference(9.0);
   calib.AddSensor(-200);
   calib.DoCalibration();
   calib.AddReference(9.5);
   calib.AddSensor(-100);
   calib.DoCalibration();
   EXPECT_NEAR(  0.005, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 85.714285, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(-10.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 85.714285, calib.GetBaseReliable(), 0.000001);

   EXPECT_NEAR( 5.0, -1000 * calib.GetScale() - calib.GetBase(), 0.000001);
   EXPECT_NEAR( 5.5,  -900 * calib.GetScale() - calib.GetBase(), 0.000001);
   EXPECT_NEAR( 6.0,  -800 * calib.GetScale() - calib.GetBase(), 0.000001);
   EXPECT_NEAR( 9.5,  -100 * calib.GetScale() - calib.GetBase(), 0.000001);
   EXPECT_NEAR( 9.0,  -200 * calib.GetScale() - calib.GetBase(), 0.000001);
   EXPECT_NEAR( 8.5,  -300 * calib.GetScale() - calib.GetBase(), 0.000001);
}

#ifdef TTT
TEST_F(PECCalibrationTest, test_gyro_hamburg_airport_b433_6_sec)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;
   PE::CCalibration   calib(scale,base);

   calib.AddSensor( -1100);
   calib.AddSensor( -1124);
   calib.AddSensor( -1136);
   calib.AddSensor( -1157);
   calib.AddSensor( -1172);
   calib.AddSensor( -1192);
   calib.AddSensor( -1201);
   calib.AddSensor( -1199);
   calib.AddSensor( -1200);
   calib.AddSensor( -1205);
   calib.AddSensor( -1206);
   calib.AddSensor( -1182);
   calib.AddSensor( -1156);
   calib.AddSensor( -1134);
   calib.AddSensor( -1111);
   calib.AddSensor( -1116);
   calib.AddSensor( -1129);
   calib.AddReference(-11.38860539);
   calib.DoCalibration();
   EXPECT_NEAR(  0.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBaseReliable(), 0.000001);

   calib.AddSensor( -1118);
   calib.AddSensor( -1101);
   calib.AddSensor( -1082);
   calib.AddSensor( -1045);
   calib.AddSensor( -988);
   calib.AddSensor( -935);
   calib.AddSensor( -887);
   calib.AddSensor( -868);
   calib.AddSensor( -850);
   calib.AddSensor( -846);
   calib.AddSensor( -840);
   calib.AddSensor( -855);
   calib.AddSensor( -839);
   calib.AddSensor( -836);
   calib.AddSensor( -823);
   calib.AddSensor( -827);
   calib.AddReference(-10.61061662);
   calib.DoCalibration();
   EXPECT_NEAR(  0.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBaseReliable(), 0.000001);

   calib.AddSensor( -830);
   calib.AddSensor( -849);
   calib.AddSensor( -843);
   calib.AddSensor( -849);
   calib.AddSensor( -859);
   calib.AddSensor( -846);
   calib.AddSensor( -880);
   calib.AddSensor( -866);
   calib.AddSensor( -907);
   calib.AddSensor( -926);
   calib.AddSensor( -966);
   calib.AddSensor( -964);
   calib.AddSensor( -945);
   calib.AddSensor( -954);
   calib.AddSensor( -967);
   calib.AddSensor( -1002);
   calib.AddSensor( -1032);
   calib.AddReference(-8.699997);
   calib.DoCalibration();
   EXPECT_NEAR(  0.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBaseReliable(), 0.000001);

   calib.AddSensor( -1047);
   calib.AddSensor( -1043);
   calib.AddSensor( -1034);
   calib.AddSensor( -1046);
   calib.AddSensor( -1065);
   calib.AddSensor( -1090);
   calib.AddSensor( -1109);
   calib.AddSensor( -1118);
   calib.AddSensor( -1122);
   calib.AddSensor( -1147);
   calib.AddSensor( -1129);
   calib.AddSensor( -1130);
   calib.AddSensor( -1111);
   calib.AddSensor( -1109);
   calib.AddSensor( -1096);
   calib.AddSensor( -1079);
   calib.AddSensor( -1073);
   calib.AddReference(-10.599991);
   calib.DoCalibration();
   EXPECT_NEAR(  0.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBaseReliable(), 0.000001);

   calib.AddSensor( -1052);
   calib.AddSensor( -1021);
   calib.AddSensor( -1003);
   calib.AddSensor( -961);
   calib.AddSensor( -947);
   calib.AddSensor( -943);
   calib.AddSensor( -936);
   calib.AddSensor( -951);
   calib.AddSensor( -930);
   calib.AddSensor( -927);
   calib.AddSensor( -920);
   calib.AddSensor( -931);
   calib.AddSensor( -922);
   calib.AddSensor( -928);
   calib.AddSensor( -926);
   calib.AddSensor( -922);
   calib.AddReference(-10.400009);
   calib.DoCalibration();
   EXPECT_NEAR(  0.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBaseReliable(), 0.000001);

   calib.AddSensor( -910);
   calib.AddSensor( -914);
   calib.AddSensor( -931);
   calib.AddSensor( -910);
   calib.AddSensor( -924);
   calib.AddSensor( -905);
   calib.AddSensor( -907);
   calib.AddSensor( -897);
   calib.AddSensor( -881);
   calib.AddSensor( -854);
   calib.AddSensor( -816);
   calib.AddSensor( -771);
   calib.AddSensor( -711);
   calib.AddSensor( -655);
   calib.AddSensor( -607);
   calib.AddSensor( -561);
   calib.AddSensor( -512);
   calib.AddReference(-8.999985);
   calib.DoCalibration();
   EXPECT_NEAR(  0.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBaseReliable(), 0.000001);

   calib.AddSensor( -469);
   calib.AddSensor( -399);
   calib.AddSensor( -296);
   calib.AddSensor( -222);
   calib.AddSensor( -156);
   calib.AddSensor( -133);
   calib.AddSensor( -94);
   calib.AddSensor( -60);
   calib.AddSensor( -27);
   calib.AddSensor( 5);
   calib.AddSensor( 3);
   calib.AddSensor( 4);
   calib.AddSensor( 30);
   calib.AddSensor( 53);
   calib.AddSensor( 95);
   calib.AddSensor( 111);
   calib.AddSensor( 125);
   calib.AddReference( -4.100006);
   calib.DoCalibration();
   EXPECT_NEAR(  0.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBaseReliable(), 0.000001);

   calib.AddSensor( 127);
   calib.AddSensor( 126);
   calib.AddSensor( 134);
   calib.AddSensor( 123);
   calib.AddSensor( 120);
   calib.AddSensor( 121);
   calib.AddSensor( 127);
   calib.AddSensor( 126);
   calib.AddSensor( 121);
   calib.AddSensor( 123);
   calib.AddSensor( 128);
   calib.AddSensor( 130);
   calib.AddSensor( 150);
   calib.AddSensor( 158);
   calib.AddSensor( 168);
   calib.AddSensor( 190);
   calib.AddReference( 1.299988);
   calib.DoCalibration();
   EXPECT_NEAR(  0.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBaseReliable(), 0.000001);

   calib.AddSensor( 198);
   calib.AddSensor( 190);
   calib.AddSensor( 186);
   calib.AddSensor( 190);
   calib.AddSensor( 176);
   calib.AddSensor( 178);
   calib.AddSensor( 159);
   calib.AddSensor( 169);
   calib.AddSensor( 168);
   calib.AddSensor( 168);
   calib.AddSensor( 176);
   calib.AddSensor( 187);
   calib.AddSensor( 201);
   calib.AddSensor( 210);
   calib.AddSensor( 223);
   calib.AddSensor( 241);
   calib.AddSensor( 250);
   calib.AddReference( 1.600006);
   calib.DoCalibration();
   EXPECT_NEAR(  0.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBaseReliable(), 0.000001);
}
#endif

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

