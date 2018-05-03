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
   calib.AddReference(200000);
   calib.AddReference(100000);
   calib.AddReference(200000);
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
   calib.AddSensor(200000);
   calib.AddSensor(100000);
   calib.AddSensor(200000);
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
   calib.AddSensor(1000);
   calib.AddSensor(1000);
   calib.AddSensor(1000);
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
   calib.AddReference(1000);
   calib.AddReference(1000);
   calib.AddReference(1000);
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
   calib.AddSensor(200.1);
   calib.AddReference(1000.1);
   calib.AddSensor(100.1);
   calib.AddReference(1000.1);
   calib.AddSensor(200.1);
   calib.AddReference(1000.1);
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
   calib.AddSensor(100.1);
   calib.AddReference(2000.1);
   calib.AddSensor(100.1);
   calib.AddReference(1000.1);
   calib.AddSensor(100.1);
   calib.AddReference(2000.1);
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
   calib.AddSensor(100.1);
   calib.AddReference(1000.1);
   calib.AddSensor(100.1);
   calib.AddReference(1000.1);
   calib.AddSensor(100.1);
   calib.AddReference(1000.1);
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
      calib.AddSensor(i*10);
   }

   EXPECT_NEAR( 0.1, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 0.0, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(96.915807, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.000000, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR(98.285920, calib.GetBaseReliable(), 0.000001);
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
   calib.AddReference(10);
   calib.AddSensor(0);
   calib.AddReference(50);
   calib.AddSensor(40);
   calib.AddReference(60);
   calib.AddSensor(50);
   calib.AddReference(100);
   calib.AddSensor(90);

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
   calib.AddReference(60);
   calib.AddSensor(50);
   calib.AddReference(50);
   calib.AddSensor(40);
   calib.AddReference(45);
   calib.AddSensor(35);
   calib.AddReference(40);
   calib.AddSensor(30);

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
   calib.AddReference(80);
   calib.AddSensor(0);
   calib.AddReference(40);
   calib.AddSensor(-4);
   calib.AddReference(40);
   calib.AddSensor(-4);
   calib.AddReference(10);
   calib.AddSensor(-7);
   calib.AddReference(20);
   calib.AddSensor(-6);
   calib.AddReference(20);
   calib.AddSensor(-6);
   calib.AddReference(0);
   calib.AddSensor(-8);
   calib.AddReference(-20);
   calib.AddSensor(-10);

   EXPECT_NEAR( 10.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 83.333333, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR( -8.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 85.714285, calib.GetBaseReliable(), 0.000001);
}

/**
 * tests ????
 */
TEST_F(PECCalibrationTest, test_negative_sen_and_positive_ref)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;
   PE::CCalibration   calib(scale,base);

   calib.AddReference(5.0);
   calib.AddSensor(-1000);
   calib.AddReference(5.5);
   calib.AddSensor(-900);
   calib.AddReference(6.0);
   calib.AddSensor(-800);
   calib.AddReference(9.5);
   calib.AddSensor(-100);
   calib.AddReference(9.0);
   calib.AddSensor(-200);
   calib.AddReference(9.0);
   calib.AddSensor(-200);
   calib.AddReference(8.5);
   calib.AddSensor(-300);
   calib.AddReference(9.0);
   calib.AddSensor(-200);
   calib.AddReference(9.5);
   calib.AddSensor(-100);

   EXPECT_NEAR(  0.005, calib.GetScale(), 0.000001);
   EXPECT_NEAR( 85.714285, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(-2000.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 85.714285, calib.GetBaseReliable(), 0.000001);

   EXPECT_NEAR( 5.0, (-1000 - calib.GetBase()) * calib.GetScale(), 0.000001);
   EXPECT_NEAR( 5.5, (-900 - calib.GetBase()) * calib.GetScale(), 0.000001);
   EXPECT_NEAR( 6.0, (-800 - calib.GetBase()) * calib.GetScale(), 0.000001);
   EXPECT_NEAR( 9.5, (-100 - calib.GetBase()) * calib.GetScale(), 0.000001);
   EXPECT_NEAR( 9.0, (-200 - calib.GetBase()) * calib.GetScale(), 0.000001);
   EXPECT_NEAR( 8.5, (-300 - calib.GetBase()) * calib.GetScale(), 0.000001);
}

#ifdef TTT
TEST_F(PECCalibrationTest, test_gyro_hamburg_airport_b433_6_sec)
{
   PE::CNormalisation scale;
   PE::CNormalisation base;
   PE::CCalibration   calib(scale,base);
   PE::TValue  gyro = 0;
   unsigned int cnt = 0;

   gyro += -1100; ++cnt;
   gyro += -1124; ++cnt;
   gyro += -1136; ++cnt;
   gyro += -1157; ++cnt;
   gyro += -1172; ++cnt;
   gyro += -1192; ++cnt;
   gyro += -1201; ++cnt;
   gyro += -1199; ++cnt;
   gyro += -1200; ++cnt;
   gyro += -1205; ++cnt;
   gyro += -1206; ++cnt;
   gyro += -1182; ++cnt;
   gyro += -1156; ++cnt;
   gyro += -1134; ++cnt;
   gyro += -1111; ++cnt;
   gyro += -1116; ++cnt;
   gyro += -1129; ++cnt;
   calib.AddReference(-11.3886053946054);
   calib.AddSensor(gyro/cnt);
   gyro = 0;   cnt = 0;
   gyro += -1118; ++cnt;
   gyro += -1101; ++cnt;
   gyro += -1082; ++cnt;
   gyro += -1045; ++cnt;
   gyro += -988; ++cnt;
   gyro += -935; ++cnt;
   gyro += -887; ++cnt;
   gyro += -868; ++cnt;
   gyro += -850; ++cnt;
   gyro += -846; ++cnt;
   gyro += -840; ++cnt;
   gyro += -855; ++cnt;
   gyro += -839; ++cnt;
   gyro += -836; ++cnt;
   gyro += -823; ++cnt;
   gyro += -827; ++cnt;
   calib.AddReference(-10.6106166166166);
   calib.AddSensor(gyro/cnt);
   gyro = 0;   cnt = 0;
   gyro += -830; ++cnt;
   gyro += -849; ++cnt;
   gyro += -843; ++cnt;
   gyro += -849; ++cnt;
   gyro += -859; ++cnt;
   gyro += -846; ++cnt;
   gyro += -880; ++cnt;
   gyro += -866; ++cnt;
   gyro += -907; ++cnt;
   gyro += -926; ++cnt;
   gyro += -966; ++cnt;
   gyro += -964; ++cnt;
   gyro += -945; ++cnt;
   gyro += -954; ++cnt;
   gyro += -967; ++cnt;
   gyro += -1002; ++cnt;
   gyro += -1032; ++cnt;
   calib.AddReference(-8.699997);
   calib.AddSensor(gyro/cnt);
   gyro = 0;   cnt = 0;
   gyro += -1047; ++cnt;
   gyro += -1043; ++cnt;
   gyro += -1034; ++cnt;
   gyro += -1046; ++cnt;
   gyro += -1065; ++cnt;
   gyro += -1090; ++cnt;
   gyro += -1109; ++cnt;
   gyro += -1118; ++cnt;
   gyro += -1122; ++cnt;
   gyro += -1147; ++cnt;
   gyro += -1129; ++cnt;
   gyro += -1130; ++cnt;
   gyro += -1111; ++cnt;
   gyro += -1109; ++cnt;
   gyro += -1096; ++cnt;
   gyro += -1079; ++cnt;
   gyro += -1073; ++cnt;
   calib.AddReference(-10.599991);
   calib.AddSensor(gyro/cnt);
   gyro = 0;   cnt = 0;
   gyro += -1052; ++cnt;
   gyro += -1021; ++cnt;
   gyro += -1003; ++cnt;
   gyro += -961; ++cnt;
   gyro += -947; ++cnt;
   gyro += -943; ++cnt;
   gyro += -936; ++cnt;
   gyro += -951; ++cnt;
   gyro += -930; ++cnt;
   gyro += -927; ++cnt;
   gyro += -920; ++cnt;
   gyro += -931; ++cnt;
   gyro += -922; ++cnt;
   gyro += -928; ++cnt;
   gyro += -926; ++cnt;
   gyro += -922; ++cnt;
   calib.AddReference(-10.400009);
   calib.AddSensor(gyro/cnt);
   gyro = 0;   cnt = 0;
   gyro += -910; ++cnt;
   gyro += -914; ++cnt;
   gyro += -931; ++cnt;
   gyro += -910; ++cnt;
   gyro += -924; ++cnt;
   gyro += -905; ++cnt;
   gyro += -907; ++cnt;
   gyro += -897; ++cnt;
   gyro += -881; ++cnt;
   gyro += -854; ++cnt;
   gyro += -816; ++cnt;
   gyro += -771; ++cnt;
   gyro += -711; ++cnt;
   gyro += -655; ++cnt;
   gyro += -607; ++cnt;
   gyro += -561; ++cnt;
   gyro += -512; ++cnt;
   calib.AddReference(-8.99998499999998);
   calib.AddSensor(gyro/cnt);

   EXPECT_NEAR(  0.0, calib.GetScale(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleMld(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetScaleReliable(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBase(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBaseMld(), 0.000001);
   EXPECT_NEAR(  0.0, calib.GetBaseReliable(), 0.000001);
}
#endif

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

