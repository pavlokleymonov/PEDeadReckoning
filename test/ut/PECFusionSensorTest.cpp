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
 * Unit test of the PECFusionSensor class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include "PECFusionSensor.h"
#include "PETools.h"


class PECFusionSensorTest : public ::testing::Test
{
public:
   virtual void SetUp()
   {}
   virtual void TearDown()
   {}

   static PE::TValue GetAngAcc(const PE::CFusionSensor& fusion)
   {
      return fusion.m_AngAcceleration;
   }
   static PE::TValue GetLinAcc(const PE::CFusionSensor& fusion)
   {
      return fusion.m_LineAcceleration;
   }
};

//test cases:
//check skipping sensors data with outdated timesatmp
//check if all start value are invalid and add only one set of sensors. -> getters have to return same value
//test do fusion wihtout any data
//compare simple and complex fusions
//test stored position, but no new sensors data
//test with invalid init speed/angSpeed
//test with zero init speed/angSpeed
//test add sensors with same timestamp after DoFusion
//test influence older sensors but with better accuracy to new sensors with worthe accuracy

//test adding several same sensors inside one fusion

#ifdef TTT
/**
 * create test
 */
TEST_F(PECFusionSensorTest, test_create )
{
   PE::SPosition pos = PE::SPosition(50.0,10.0,0.1);//lat=50 lon=10
   PE::SBasicSensor heading = PE::SBasicSensor(90.0,5.0);//90deg
   PE::SBasicSensor speed = PE::SBasicSensor(5.0,0.1); //5m/s
   PE::SBasicSensor angSpeed = PE::SBasicSensor(10.0,0.1);//10deg/s

   //all input data are valid
   PE::CFusionSensor fusion = PE::CFusionSensor(1000.0, pos, heading, angSpeed, speed);
   EXPECT_EQ(1000.0, fusion.GetTimestamp());

   EXPECT_TRUE(fusion.GetPosition().IsValid());
   EXPECT_EQ(50.0, fusion.GetPosition().Latitude);
   EXPECT_EQ(10.0, fusion.GetPosition().Longitude);
   EXPECT_EQ(0.1, fusion.GetPosition().HorizontalAcc);

   EXPECT_TRUE(fusion.GetHeading().IsValid());
   EXPECT_EQ(90.0, fusion.GetHeading().Value);
   EXPECT_EQ(5.0, fusion.GetHeading().Accuracy);

   EXPECT_TRUE(fusion.GetSpeed().IsValid());
   EXPECT_EQ(5.0,fusion.GetSpeed().Value);
   EXPECT_EQ(0.1,fusion.GetSpeed().Accuracy);

   EXPECT_TRUE(fusion.GetAngSpeed().IsValid());
   EXPECT_EQ(10.0, fusion.GetAngSpeed().Value);
   EXPECT_EQ(0.1, fusion.GetAngSpeed().Accuracy);

   //all input data are invalid
   PE::CFusionSensor fusion2 = PE::CFusionSensor(0.0,PE::SPosition(),PE::SBasicSensor(),PE::SBasicSensor(),PE::SBasicSensor());
   EXPECT_EQ(0.0, fusion2.GetTimestamp());
   EXPECT_FALSE(fusion2.GetPosition().IsValid());
   EXPECT_FALSE(fusion2.GetHeading().IsValid());
   EXPECT_FALSE(fusion2.GetSpeed().IsValid());
   EXPECT_FALSE(fusion2.GetAngSpeed().IsValid());
}


/**
 * Test incorrect timestamp of position.
 */
TEST_F(PECFusionSensorTest, test_incorrect_timestamp_pos )
{
   PE::SPosition pos = PE::SPosition(50.0, 10.0, 1);//lat=50 lon=10
   PE::SBasicSensor heading = PE::SBasicSensor(90.0,0.3);//90deg
   PE::SBasicSensor angSpeedInvalid;
   PE::SBasicSensor speedInvalid;
   PE::CFusionSensor fusion = PE::CFusionSensor(10.0, pos, heading,angSpeedInvalid,speedInvalid);

   EXPECT_TRUE(fusion.GetPosition().IsValid());
   EXPECT_TRUE(fusion.GetHeading().IsValid());
   EXPECT_NEAR(10.0000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000000, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.000, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.300, fusion.GetHeading().Accuracy,0.001);

   //smaller timestamp - ignored
   fusion.AddPosition(9.9, PE::SPosition(50.0000000, 10.0000500, 1));
   fusion.DoFusion();
   EXPECT_NEAR(10.0000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000000, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.000, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.300, fusion.GetHeading().Accuracy,0.001);

   //same timestamp
   fusion.AddPosition(10.0, PE::SPosition(50.0000000, 10.0001000, 1));
   fusion.DoFusion();
   EXPECT_NEAR(10.0000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000500, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.000, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.300, fusion.GetHeading().Accuracy,0.001);

   //correct timestamp
   fusion.AddPosition(11.0, PE::SPosition(50.0000000, 10.0001500, 1));
   fusion.DoFusion();
   EXPECT_NEAR(11.0000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0001166, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.333, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     1.117, fusion.GetHeading().Accuracy,0.001);
}



/**
 * !!! Test drive full circle turning left with ang speed 18 deg/s and 10m/s
 */
TEST_F(PECFusionSensorTest, test_full_circle_left_ang_18_speed_10 )
{
   PE::SBasicSensor  heading  ( 90.0,0.1);
   PE::SBasicSensor angSpeed  ( 18.0,0.1); //turn left 18deg/s
   PE::SBasicSensor    speed  ( 10.0,0.1); //10 m/s
   PE::SPosition         pos  ( 50.0,10.0, 0.1);
   PE::CFusionSensor fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);
   
   fusion.AddSpeed(1.0, speed);
   fusion.AddAngSpeed(1.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR(50.00001401, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(72.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(18.00000000, fusion.GetAngSpeed().Value     , 0.00000001);
   EXPECT_NEAR(10.00000000, fusion.GetSpeed().Value        , 0.00000001);
   EXPECT_NEAR( 0.23333526, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.23333333, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 0.13333333, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.13333333, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddSpeed(2.0, speed);
   fusion.AddAngSpeed(2.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR(50.00005467, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00026176, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(54.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 0.37879809, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.37878787, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(3.0, speed);
   fusion.AddAngSpeed(3.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR(50.00011800, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00036029, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(36.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 0.52765767, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.52762508, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(4.0, speed);
   fusion.AddAngSpeed(4.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR(50.00019780, fusion.GetPosition().Latitude,  0.0000001);
   EXPECT_NEAR(10.00042354, fusion.GetPosition().Longitude, 0.0000001);
   EXPECT_NEAR(18.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 0.67741261, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.67733269, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(5.0, speed);
   fusion.AddAngSpeed(5.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR(50.00028626, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00044534, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 0.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 0.82742564, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.82725948, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(10.0, speed);
   fusion.AddAngSpeed(10.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR( 50.00057252, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(270.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(  1.72816688, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  1.72721554, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(15.0, speed);
   fusion.AddAngSpeed(15.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR( 50.00028626, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(  9.99955464, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(180.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(  2.64623114, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  2.64246599, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(20.0, speed);
   fusion.AddAngSpeed(20.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 90.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 18.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  3.56966411, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  3.55901447, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR(  0.18330969, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR(  0.18330969, fusion.GetSpeed().Accuracy        , 0.00000001);
}

/**
 * !!! Test for reliable position only fusion driving on a circle.
 */
TEST_F(PECFusionSensorTest, test_reliable_position_only_fusion_circle_driving )
{
   PE::SBasicSensor  heading  ( 90.0,0.1);
   PE::SBasicSensor angSpeed  ( 18.0,0.1); //turn left 18deg/s
   PE::SBasicSensor    speed  ( 10.0,0.1); //10 m/s
   PE::SPosition         pos  ( 50.0,10.0, 0.1);
   PE::CFusionSensor fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);
   
   fusion.AddPosition(1.0, PE::SPosition(50.00001401, 10.00013761, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(50.00001401, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(71.99986483, fusion.GetHeading().Value     , 0.00000001); //72deg
   EXPECT_NEAR(18.00009011, fusion.GetAngSpeed().Value     , 0.00000001);//18 deg/s
   EXPECT_NEAR( 9.99965691, fusion.GetSpeed().Value        , 0.00000001);//10 m/s
   EXPECT_NEAR( 0.15000051, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.41543819, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 0.31797253, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.20000915, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(2.0, PE::SPosition(50.00005467, 10.00026176, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(50.00005467, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00026176, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(54.00014974, fusion.GetHeading().Value     , 0.00000001);//54 deg
   EXPECT_NEAR(17.99989887, fusion.GetAngSpeed().Value     , 0.00000001);
   EXPECT_NEAR( 9.99967569, fusion.GetSpeed().Value        , 0.00000001);
   EXPECT_NEAR( 0.16923604, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 1.09133661, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 0.90184654, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.30776740, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(3.0, PE::SPosition(50.00011800, 10.00036029, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(50.00011800, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00036029, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(36.00241666, fusion.GetHeading().Value     , 0.00000001);//36 deg
   EXPECT_NEAR(17.99844345, fusion.GetAngSpeed().Value     , 0.00000001);
   EXPECT_NEAR(10.00021398, fusion.GetSpeed().Value        , 0.00000001);
   EXPECT_NEAR( 0.17742086, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 2.26902105, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 2.24063332, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.37496218, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(4.0, PE::SPosition(50.00019780, 10.00042354, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(50.00019780, fusion.GetPosition().Latitude,  0.0000001);
   EXPECT_NEAR(10.00042354, fusion.GetPosition().Longitude, 0.0000001);
   EXPECT_NEAR(17.99712055, fusion.GetHeading().Value     , 0.00000001);//18 deg
   EXPECT_NEAR(18.00259803, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 9.99991003, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR( 0.18065424, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 4.21756531, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 4.87137928, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.40629933, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(5.0, PE::SPosition(50.00028626, 10.00044534, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(50.00028626, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00044534, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(18.00028373, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 9.99981863, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR( 0.00014539, fusion.GetHeading().Value     , 0.00000001);//0deg
   EXPECT_NEAR( 0.18219822, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 7.38964296, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 9.48590632, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.42127728, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(10.0, PE::SPosition(50.00057252, 10.00000000, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00057206, fusion.GetPosition().Latitude , 0.00000001); //~6cm difference
   EXPECT_NEAR( 10.00000070, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(270.00638774, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 17.99164588, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 9.139192203, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  0.19968218, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 16.11768597, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 24.58311172, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR(  0.68933860, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(15.0, PE::SPosition(50.00028626, 9.99955464, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00028671, fusion.GetPosition().Latitude , 0.00000001);//~8cm difference
   EXPECT_NEAR(  9.99955535, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(180.17176341, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 17.97589431, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR(  8.93569391, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  0.19967929, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 33.38621126, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 54.18549469, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR(  0.76884637, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(20.0, PE::SPosition(50.00000000, 10.00000000, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000041, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(  9.99999937, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 90.01976937, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 18.01489770, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 9.053004311, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  0.19971964, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 67.28671344, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR(112.09984702, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR(  0.77630108, fusion.GetSpeed().Accuracy        , 0.00000001);
}


/**
 * !!! Test direct driving immediately after turning by speed 10m/s and angular speed 0.
 */
TEST_F(PECFusionSensorTest, test_direct_driving_immediately_after_turning_speed_10_angspeed_20_to_0 )
{
   PE::SBasicSensor  heading    ( 90.0,0.1);
   PE::SBasicSensor angSpeed200 ( 20.0,0.1);
   PE::SBasicSensor angSpeed190 ( 19.0,0.1); 
   PE::SBasicSensor angSpeed175 ( 17.5,0.1);
   PE::SBasicSensor angSpeed165 ( 16.5,0.1);
   PE::SBasicSensor angSpeed160 ( 16.0,0.1);
   PE::SBasicSensor angSpeed150 ( 15.0,0.1);
   PE::SBasicSensor angSpeed120 ( 12.0,0.1);
   PE::SBasicSensor angSpeed090 (  9.0,0.1);
   PE::SBasicSensor angSpeed050 (  5.0,0.1);
   PE::SBasicSensor angSpeed020 (  2.0,0.1);
   PE::SBasicSensor angSpeed010 (  1.0,0.1);
   PE::SBasicSensor angSpeed000 (  0.0,0.1);
   PE::SBasicSensor    speed    ( 10.0,0.1); //10 m/s
   PE::SPosition         pos    ( 50.0,10.0, 0.1);
   PE::CFusionSensor fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed200,speed);

   //start dirrect driving 
   fusion.AddSpeed   (1.0, speed);
   fusion.AddAngSpeed(1.0, angSpeed200);
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 70.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 20.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
   fusion.AddSpeed   (2.0, speed);
   fusion.AddAngSpeed(2.0, angSpeed190);//Head51
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 51.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 19.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
   fusion.AddSpeed   (3.0, speed);
   fusion.AddAngSpeed(3.0, angSpeed175);
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 33.50000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 17.50000000, fusion.GetAngSpeed().Value    , 0.00000001);
   fusion.AddSpeed   (4.0, speed);
   fusion.AddAngSpeed(4.0, angSpeed165);//Head17
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 17.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 16.50000000, fusion.GetAngSpeed().Value    , 0.00000001);
   fusion.AddSpeed   (5.0, speed);
   fusion.AddAngSpeed(5.0, angSpeed090);//Head8
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(  8.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(  9.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
   fusion.AddSpeed   (6.0, speed);
   fusion.AddAngSpeed(6.0, angSpeed050);//Head3
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(  3.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(  5.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
   fusion.AddSpeed   (7.0, speed);
   fusion.AddAngSpeed(7.0, angSpeed020);//Head1
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(  1.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(  2.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
   fusion.AddSpeed   (8.0, speed);
   fusion.AddAngSpeed(8.0, angSpeed010);//Head0
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 00.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(  1.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
   fusion.AddSpeed   (9.0, speed);
   fusion.AddAngSpeed(9.0, angSpeed000);//Head0
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 00.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 00.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
}

#endif

#ifdef TTT
/**
 * Test position only fusion driving on a circle.
 */
TEST_F(PECFusionSensorTest, test_position_only_fusion_circle_driving )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(0.0, PE::SPosition(50.0, 10.0, 1), PE::SBasicSensor( 90.0, 0.10 ),PE::SBasicSensor(),PE::SBasicSensor());

   fusion.AddPosition(1.0, PE::SPosition(50.0000039, 10.0000696, 1));
   fusion.DoFusion();
   EXPECT_NEAR(50.0000026, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000464, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.333, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    89.822, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.392, fusion.GetHeading().Accuracy,0.001);
//TODO check coordinates with new fusion!!!
   fusion.AddPosition(2.0, PE::SPosition(50.0000155, 10.0001370, 1));
   fusion.DoFusion();
   EXPECT_NEAR(50.0000140, fusion.GetPosition().Latitude, 0.0000001);
   EXPECT_NEAR(10.0001320, fusion.GetPosition().Longitude, 0.0000001);
   EXPECT_NEAR( 1.7761303, fusion.GetPosition().HorizontalAcc, 0.0000001);
   EXPECT_NEAR(90.0000000, fusion.GetHeading().Value, 0.0000001);
   EXPECT_NEAR( 0.8000000, fusion.GetHeading().Accuracy, 0.0000001);

   fusion.AddPosition(3.0, PE::SPosition(50.0000345, 10.0002004, 1));
   fusion.DoFusion();
   EXPECT_NEAR(50.0000326, fusion.GetPosition().Latitude, 0.0000001);
   EXPECT_NEAR(10.0002009, fusion.GetPosition().Longitude, 0.0000001);
   EXPECT_NEAR( 1.8220510, fusion.GetPosition().HorizontalAcc, 0.0000001);
   EXPECT_NEAR(90, fusion.GetHeading().Value, 0.0000001);
   EXPECT_NEAR( 2.3000000, fusion.GetHeading().Accuracy, 0.0000001);

   fusion.AddPosition(4.0, PE::SPosition(50.0000602, 10.0002576, 1));
   fusion.DoFusion();
   EXPECT_NEAR(50.0000567, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0002599, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.723, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    74.621, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(    11.559, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(5.0, PE::SPosition(50.0000920, 10.0003070, 1));
   fusion.DoFusion();
   EXPECT_NEAR(50.0000894, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003097, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.757, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    55.774, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(    19.310, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(6.0, PE::SPosition(50.0001288, 10.0003471, 1));
   fusion.DoFusion();
   EXPECT_NEAR(50.0001278, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003487, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.821, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    36.637, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(    25.102, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(7.0, PE::SPosition(50.0001695, 10.0003766, 1));
   fusion.DoFusion();
   EXPECT_NEAR(50.0001694, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003768, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.932, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    23.144, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(    29.243, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(8.0, PE::SPosition(50.0002128, 10.0003947, 1));
   fusion.DoFusion();
   EXPECT_NEAR(50.0002100, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003935, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.872, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    13.887, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(    32.657, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(9.0, PE::SPosition(50.0002576, 10.0004008, 1));
   fusion.DoFusion();
   EXPECT_NEAR(50.0002545, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0004003, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.871, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(     5.258, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(    31.966, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(10.0, PE::SPosition(50.0003030, 10.0003970, 1));
   fusion.DoFusion();
   EXPECT_NEAR(50.0002999, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003972, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.872, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(   357.286, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(    31.919, fusion.GetHeading().Accuracy,0.001);
}

/**
 * Test position only fusion straight driving.
 */
TEST_F(PECFusionSensorTest, test_position_only_fusion_straight_driving )
{
   PE::SPosition    prevPos  = PE::SPosition(50.0, 10.0, 1);
   PE::SBasicSensor prevHead = PE::SBasicSensor( 90.0, 0.10);
   PE::TValue       distance = 5; //5[m]

   PE::CFusionSensor fusion = PE::CFusionSensor(10.0, prevPos, prevHead,PE::SBasicSensor(), PE::SBasicSensor());

   fusion.AddPosition(11.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0000466, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.333, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(     0.395, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(12.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0001115, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.571, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(     1.688, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(13.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0001795, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.675, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(     5.138, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(14.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0002488, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.723, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    11.820, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(15.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0003185, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.757, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    19.637, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(16.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0003884, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.823, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    25.524, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(17.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0004584, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.936, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    29.508, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(18.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0005238, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.872, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    32.697, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(19.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0005893, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.871, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    33.249, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(20.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0006548, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.871, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    33.612, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(21.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0007202, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.870, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    33.743, prevHead.Accuracy,0.001);
}

/**
 * Test position and heading fusion driving on a circle.
 */
TEST_F(PECFusionSensorTest, test_position_heading_fusion_circle_driving )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(10.0, PE::SPosition(50.0, 10.0, 1), PE::SBasicSensor( 90.0, 0.10 ),PE::SBasicSensor(),PE::SBasicSensor());

   fusion.AddPosition(11.0, PE::SPosition(50.0000039, 10.0000696, 1));
   fusion.AddHeading (11.0, PE::SBasicSensor( 75.0, 0.10 ));
   fusion.DoFusion();
   EXPECT_NEAR(50.0000026, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000464, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.333, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    78.015, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.159, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(12.0, PE::SPosition(50.0000155, 10.0001370, 1));
   fusion.AddHeading (12.0, PE::SBasicSensor( 65.0, 0.10 ));
   fusion.DoFusion();
   EXPECT_NEAR(50.0000140, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0001273, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.571, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    65.960, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.185, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(13.0, PE::SPosition(50.0000345, 10.0002004, 1));
   fusion.AddHeading (13.0, PE::SBasicSensor( 55.0, 0.10 ));
   fusion.DoFusion();
   EXPECT_NEAR(50.0000341, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0001987, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.675, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    55.424, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.192, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(14.0, PE::SPosition(50.0000602, 10.0002576, 1));
   fusion.AddHeading (14.0, PE::SBasicSensor( 45.0, 0.10 ));
   fusion.DoFusion();
   EXPECT_NEAR(50.0000603, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0002580, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.720, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    45.220, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.195, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(15.0, PE::SPosition(50.0000920, 10.0003070, 1));
   fusion.AddHeading (15.0, PE::SBasicSensor( 35.0, 0.10 ));
   fusion.DoFusion();
   EXPECT_NEAR(50.0000921, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003073, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.739, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    35.127, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.197, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(16.0, PE::SPosition(50.0001288, 10.0003471, 1));
   fusion.AddHeading (16.0, PE::SBasicSensor( 25.0, 0.10 ));
   fusion.DoFusion();
   EXPECT_NEAR(50.0001288, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003472, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.749, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    25.083, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.198, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(17.0, PE::SPosition(50.0001695, 10.0003766, 1));
   fusion.AddHeading (17.0, PE::SBasicSensor( 15.0, 0.10 ));
   fusion.DoFusion();
   EXPECT_NEAR(50.0001695, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003766, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.759, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    15.062, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.198, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(18.0, PE::SPosition(50.0002128, 10.0003947, 1));
   fusion.AddHeading (18.0, PE::SBasicSensor( 5.0, 0.10 ));
   fusion.DoFusion();
   EXPECT_NEAR(50.0002128, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003946, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.773, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(     5.051, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.199, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(19.0, PE::SPosition(50.0002576, 10.0004008, 1));
   fusion.AddHeading (19.0, PE::SBasicSensor( 0.0, 0.10 ));
   fusion.DoFusion();
   EXPECT_NEAR(50.0002575, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0004007, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.789, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(     0.023, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.199, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(20.0, PE::SPosition(50.0003030, 10.0003970, 1));
   fusion.AddHeading (20.0, PE::SBasicSensor( 355.0, 0.10 ));
   fusion.DoFusion();
   EXPECT_NEAR(50.0003029, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003973, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.803, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(   355.014, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.199, fusion.GetHeading().Accuracy,0.001);
}

/**
 * Test invariance between order of adding position and heading.
 */
TEST_F(PECFusionSensorTest, test_position_heading_fusion_invariance )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(10.0, PE::SPosition(50.0, 10.0, 1), PE::SBasicSensor( 90.0, 0.10 ),PE::SBasicSensor(),PE::SBasicSensor());

   fusion.AddPosition(11.0, PE::SPosition(50.0000039, 10.0000696, 1));
   fusion.AddHeading (11.0, PE::SBasicSensor( 85.0, 0.10 ));
   fusion.DoFusion();
   EXPECT_NEAR(50.0000026, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000464, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.333, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    85.997, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.159, fusion.GetHeading().Accuracy,0.001);

   PE::CFusionSensor fusion2 = PE::CFusionSensor(10.0, PE::SPosition(50.0, 10.0, 1), PE::SBasicSensor( 90.0, 0.10 ),PE::SBasicSensor(),PE::SBasicSensor());

   fusion2.AddHeading (11.0, PE::SBasicSensor( 85.0, 0.10 ));
   fusion2.AddPosition(11.0, PE::SPosition(50.0000039, 10.0000696, 1));
   fusion2.DoFusion();
   EXPECT_NEAR(50.0000026, fusion2.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000464, fusion2.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.333, fusion2.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    85.997, fusion2.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.159, fusion2.GetHeading().Accuracy,0.001);
}

#endif

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

