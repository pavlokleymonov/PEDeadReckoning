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
 * Unit test of the PECCalibrationScale class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PECCalibrationScale.h"


class PECCalibrationScaleTest : public ::testing::Test
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
TEST_F(PECCalibrationScaleTest, test_create)
{
   PE::CNormalisation scale;
   PE::CCalibrationScale   calib(scale);

   EXPECT_NEAR(0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetMean(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetMld(), 0.000001);
   EXPECT_NEAR(0.000000, calib.GetReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationScaleTest, test_not_empty_normalisation)
{
   PE::CNormalisation scale;

   scale.AddSensor(10);
   scale.AddSensor(10);
   scale.AddSensor(10);
   scale.AddSensor(10);
   EXPECT_NEAR(10.000000, scale.GetMean(), 0.000001);
   PE::CCalibrationScale   calib(scale);
   EXPECT_NEAR( 0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR(10.000000, calib.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetMld(), 0.000001);
   EXPECT_NEAR(75.000000, calib.GetReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationScaleTest, test_do_calibration_no_sensors_no_reference)
{
   PE::CNormalisation scale;

   scale.AddSensor(10);
   scale.AddSensor(10);
   scale.AddSensor(10);
   scale.AddSensor(10);
   EXPECT_NEAR(10.000000, scale.GetMean(), 0.000001);
   PE::CCalibrationScale   calib(scale);
   calib.DoCalibration();
   calib.DoCalibration();
   calib.DoCalibration();
   calib.DoCalibration();
   calib.DoCalibration();
   EXPECT_NEAR( 0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR(10.000000, calib.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetMld(), 0.000001);
   EXPECT_NEAR(75.000000, calib.GetReliable(), 0.000001);
}


/**
 * tests
 */
TEST_F(PECCalibrationScaleTest, test_same_ref_data)
{
   PE::CNormalisation scale;
   PE::CCalibrationScale   calib(scale);
   calib.AddSensor(200000);
   calib.AddSensor(100000);
   calib.AddReference(100000);
   calib.DoCalibration();

   calib.AddSensor(200000);
   calib.AddSensor(100000);
   calib.AddReference(100000);
   calib.DoCalibration();

   calib.AddSensor(200000);
   calib.AddSensor(100000);
   calib.AddReference(100000);
   calib.DoCalibration();

   calib.AddSensor(200000);
   calib.AddSensor(100000);
   calib.AddReference(100000);
   calib.DoCalibration();

   //sensors data without changed reference data is not considered
   EXPECT_NEAR( 0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetReliable(), 0.000001);
}


/**
 * tests
 */
TEST_F(PECCalibrationScaleTest, test_same_sen_data)
{
   PE::CNormalisation scale;
   PE::CCalibrationScale   calib(scale);
   calib.AddSensor(100000);
   calib.AddSensor(100000);
   calib.AddReference(200000);
   calib.DoCalibration();

   calib.AddSensor(100000);
   calib.AddSensor(100000);
   calib.AddReference(100000);
   calib.DoCalibration();

   calib.AddSensor(100000);
   calib.AddSensor(100000);
   calib.AddReference(200000);
   calib.DoCalibration();

   calib.AddSensor(100000);
   calib.AddSensor(100000);
   calib.AddReference(100000);
   calib.DoCalibration();

   //reference data without changed sensors data is not considered
   EXPECT_NEAR( 0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetReliable(), 0.000001);
}


/**
 * tests
 */
TEST_F(PECCalibrationScaleTest, test_same_sen_and_ref_data)
{
   PE::CNormalisation scale;
   PE::CCalibrationScale   calib(scale);
   calib.AddSensor(100000);
   calib.AddSensor(100000);
   calib.AddReference(200000);
   calib.DoCalibration();

   calib.AddSensor(100000);
   calib.AddSensor(100000);
   calib.AddReference(200000);
   calib.DoCalibration();

   calib.AddSensor(100000);
   calib.AddSensor(100000);
   calib.AddReference(200000);
   calib.DoCalibration();

   calib.AddSensor(100000);
   calib.AddSensor(100000);
   calib.AddReference(200000);
   calib.DoCalibration();

   //no differences in sensors and reference data will not be considered
   EXPECT_NEAR( 0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetReliable(), 0.000001);
}


/**
 * tests
 */
TEST_F(PECCalibrationScaleTest, test_sen_and_ref_data_scale_1)
{
   PE::CNormalisation scale;
   PE::CCalibrationScale   calib(scale);
   calib.AddSensor(100000);
   calib.AddReference(100000);
   calib.DoCalibration();

   calib.AddSensor(200000);
   calib.AddReference(200000);
   calib.DoCalibration();

   calib.AddSensor(100000);
   calib.AddReference(100000);
   calib.DoCalibration();

   calib.AddSensor(200000);
   calib.AddReference(200000);
   calib.DoCalibration();

   //no differences in sensors and reference data will not be considered
   EXPECT_NEAR( 1.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 1.000000, calib.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetMld(), 0.000001);
   EXPECT_NEAR(66.666667, calib.GetReliable(), 0.000001);
}


/**
 * tests 
 */
TEST_F(PECCalibrationScaleTest, test_sen_minus10_to_plus90_ref_0_to_100)
{
   PE::CNormalisation scale;
   PE::CCalibrationScale   calib(scale);

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
   EXPECT_NEAR(  1.0, calib.GetMean(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetMld(), 0.000001);
   EXPECT_NEAR( 75.0, calib.GetReliable(), 0.000001);
}


/**
 * tests 
 */
TEST_F(PECCalibrationScaleTest, test_sen_90_to_30_ref_100_to_40)
{
   PE::CNormalisation scale;
   PE::CCalibrationScale   calib(scale);

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
   EXPECT_NEAR(  1.0, calib.GetMean(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetMld(), 0.000001);
   EXPECT_NEAR( 75.0, calib.GetReliable(), 0.000001);
}


/**
 * tests 
 */
TEST_F(PECCalibrationScaleTest, test_sen_minus100_to_plus100_ref_0_to_20)
{
   PE::CNormalisation scale;
   PE::CCalibrationScale   calib(scale);

   calib.AddReference(0);
   calib.AddSensor(-100);
   calib.DoCalibration();

   calib.AddReference(10);
   calib.AddSensor(0);
   calib.DoCalibration();

   calib.AddReference(5);
   calib.AddSensor(-50);
   calib.DoCalibration();

   calib.AddReference(10);
   calib.AddSensor(0);
   calib.DoCalibration();

   calib.AddReference(0);
   calib.AddSensor(-100);
   calib.DoCalibration();

   calib.AddReference(20);
   calib.AddSensor(100);
   calib.DoCalibration();

   calib.AddReference(15);
   calib.AddSensor(50);
   calib.DoCalibration();
   EXPECT_NEAR(  0.1, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.1, calib.GetMean(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetMld(), 0.000001);
   EXPECT_NEAR( 40.0, calib.GetReliable(), 0.000001);
}


TEST_F(PECCalibrationScaleTest, test_gyro_hamburg_airport_b433_6_sec)
{
   PE::CNormalisation scale;
   PE::CCalibrationScale   calib(scale);

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
   EXPECT_NEAR( 0.000000, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetReliable(), 0.000001);

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
   EXPECT_NEAR( 0.003360, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetReliable(), 0.000001);

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
   EXPECT_NEAR( 0.010793, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.007076, calib.GetMean(), 0.000001);
   EXPECT_NEAR( 0.003716, calib.GetMld(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetReliable(), 0.000001);

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
   EXPECT_NEAR( 0.010688, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.008280, calib.GetMean(), 0.000001);
   EXPECT_NEAR( 0.003062, calib.GetMld(), 0.000001);
   EXPECT_NEAR(20.226010, calib.GetReliable(), 0.000001);

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
   EXPECT_NEAR( 0.008414, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.008314, calib.GetMean(), 0.000001);
   EXPECT_NEAR( 0.002074, calib.GetMld(), 0.000001);
   EXPECT_NEAR(39.766723, calib.GetReliable(), 0.000001);

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
   EXPECT_NEAR( 0.008637, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.008378, calib.GetMean(), 0.000001);
   EXPECT_NEAR( 0.001620, calib.GetMld(), 0.000001);
   EXPECT_NEAR(51.014668, calib.GetReliable(), 0.000001);

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
   EXPECT_NEAR( 0.007721, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.008269, calib.GetMean(), 0.000001);
   EXPECT_NEAR( 0.001406, calib.GetMld(), 0.000001);
   EXPECT_NEAR(57.879011, calib.GetReliable(), 0.000001);

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
   EXPECT_NEAR( 0.009956, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.008510, calib.GetMean(), 0.000001);
   EXPECT_NEAR( 0.001413, calib.GetMld(), 0.000001);
   EXPECT_NEAR(61.459420, calib.GetReliable(), 0.000001);

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
   EXPECT_NEAR( 0.009802, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.008671, calib.GetMean(), 0.000001);
   EXPECT_NEAR( 0.001373, calib.GetMld(), 0.000001);
   EXPECT_NEAR(64.805970, calib.GetReliable(), 0.000001);
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

