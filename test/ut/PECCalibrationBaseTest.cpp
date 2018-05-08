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
   EXPECT_NEAR(98.273694, calib.GetBaseReliable(), 0.000001);
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
   EXPECT_NEAR(  0.007026, calib.GetScale(), 0.000001);
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
   EXPECT_NEAR(  0.008247, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(  2.554835, calib.GetBase(), 0.000001);
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
   EXPECT_NEAR(  0.008289, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(  2.363263, calib.GetBase(), 0.000001);
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
   EXPECT_NEAR(  0.008358, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(  2.266762, calib.GetBase(), 0.000001);
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
   EXPECT_NEAR(  0.008252, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(  2.271008, calib.GetBase(), 0.000001);
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
   EXPECT_NEAR(  0.008495, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(  2.186609, calib.GetBase(), 0.000001);
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
   EXPECT_NEAR(  0.008658, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  64.79, calib.GetScaleReliable(), 0.01);
   EXPECT_NEAR(  2.086809, calib.GetBase(), 0.000001);
   EXPECT_NEAR(  55.89, calib.GetBaseReliable(), 0.01);
}


#ifdef TTT

TEST_F(PECCalibrationTest, test_gyro_hamburg_airport_b433_67_sec)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;
   PE::CCalibration   calib(scale,base);

   calib.AddSensor(-357.529411764706);
   calib.AddReference(-3.4034054054054);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-366.882352941176);
   calib.AddReference(-3.5964015984016);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-324.764705882353);
   calib.AddReference(-4.004004004004);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-302.705882352941);
   calib.AddReference(-2.699997);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-332.882352941176);
   calib.AddReference(-3.60000600000001);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-274.941176470588);
   calib.AddReference(-3.099999);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-324.823529411765);
   calib.AddReference(-3.0969010989011);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-348.470588235294);
   calib.AddReference(0.099997999999999);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-327.705882352941);
   calib.AddReference(-7.00700700700701);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-323.588235294118);
   calib.AddReference(-3.3966043956044);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-372.529411764706);
   calib.AddReference(-3.39999399999999);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-346.647058823529);
   calib.AddReference(-3.900001);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-317.470588235294);
   calib.AddReference(-3.200005);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-341.529411764706);
   calib.AddReference(-3.5);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-393.294117647059);
   calib.AddReference(-3.40339739739739);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-449.352941176471);
   calib.AddReference(-4.8951138861139);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-299.4375);
   calib.AddReference(-3.89999399999999);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-230.117647058824);
   calib.AddReference(-2.300003);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-277.705882352941);
   calib.AddReference(-2.699997);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-305.705882352941);
   calib.AddReference(-3);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-303.705882352941);
   calib.AddReference(-3.199997);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-270.823529411765);
   calib.AddReference(-3.199997);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-298.4375);
   calib.AddReference(-5.70001200000002);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-255.5625);
   calib.AddReference(0);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-210.352941176471);
   calib.AddReference(-2.11054371859295);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-311.941176470588);
   calib.AddReference(-2.300003);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-495.176470588235);
   calib.AddReference(-4.60000600000001);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-95.4705882352941);
   calib.AddReference(-3.09999099999999);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-73.1176470588235);
   calib.AddReference(-0.300003000000004);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-90);
   calib.AddReference(-0.800003000000004);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-85.1176470588235);
   calib.AddReference(-1.300003);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-62.2352941176471);
   calib.AddReference(-0.799987999999985);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(0);
   calib.AddReference(-0.400009000000011);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-14.8823529411765);
   calib.AddReference(0.199996999999996);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-427.823529411765);
   calib.AddReference(-2.09999099999999);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-875.5625);
   calib.AddReference(-7.199997);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-1045.23529411765);
   calib.AddReference(-10.200012);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-1160);
   calib.AddReference(-11.3886053946054);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-921.25);
   calib.AddReference(-10.6106166166166);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-910.882352941177);
   calib.AddReference(-8.699997);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-1091.05882352941);
   calib.AddReference(-10.599991);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-951.25);
   calib.AddReference(-10.400009);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-803.882352941177);
   calib.AddReference(-8.99998499999998);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-84.1176470588235);
   calib.AddReference(-4.10000600000001);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(135.75);
   calib.AddReference(1.29998799999998);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(192.352941176471);
   calib.AddReference(1.60000600000001);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(324.294117647059);
   calib.AddReference(2.4975024975025);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(537.6875);
   calib.AddReference(4.7047017017017);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(707.176470588235);
   calib.AddReference(6.90000900000001);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(763.764705882353);
   calib.AddReference(7.39999399999999);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(828.9375);
   calib.AddReference(8.39999399999999);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(761.882352941177);
   calib.AddReference(7.800003);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(703.352941176471);
   calib.AddReference(7.40000900000001);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(652);
   calib.AddReference(6.7931948051948);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(668);
   calib.AddReference(6.80680980980981);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(609.647058823529);
   calib.AddReference(6.40000900000001);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(621.9375);
   calib.AddReference(6.5);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(521.411764705882);
   calib.AddReference(5.699997);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(386.588235294118);
   calib.AddReference(4.800003);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(-53.9375);
   calib.AddReference(1.0988921078921);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(20.5294117647059);
   calib.AddReference(-0.899993999999992);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(165);
   calib.AddReference(1.800003);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(98.8125);
   calib.AddReference(1.5);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(54.2352941176471);
   calib.AddReference(0.300288288288273);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(46.7058823529412);
   calib.AddReference(0.900009000000011);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(32.5);
   calib.AddReference(0.399594405594398);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(26.1764705882353);
   calib.AddReference(0.500500500500501);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);

   calib.AddSensor(17.9411764705882);
   calib.AddReference(-0.5);
   calib.DoCalibration();
   EXPECT_NEAR(  0.000, calib.GetScale(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetScaleReliable(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBase(), 0.001);
   EXPECT_NEAR(  0.000, calib.GetBaseReliable(), 0.001);
}


#endif

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

