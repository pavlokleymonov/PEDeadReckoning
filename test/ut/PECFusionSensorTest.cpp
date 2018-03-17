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


TEST_F(PECFusionSensorTest, test_incorrect_timestamp )
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


TEST_F(PECFusionSensorTest, test_invariance_by_position_heading_angular_and_liniear_speed )
{
   PE::SBasicSensor  heading  ( 90.0,0.1);
   PE::SBasicSensor angSpeed  ( 18.0,0.1); //turn left 18deg/s
   PE::SBasicSensor    speed  ( 10.0,0.1); //10 m/s
   PE::SPosition         pos  ( 50.0,10.0, 0.1);
   PE::CFusionSensor fusion1 = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);
   PE::CFusionSensor fusion2 = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);

   fusion1.AddPosition(1.0, PE::SPosition(50.00001401, 10.00013761, 0.1));
   fusion2.AddAngSpeed(1.0, angSpeed);
   fusion1.AddHeading (1.0, PE::SBasicSensor(72.0, 0.1));
   fusion2.AddSpeed   (1.0, speed);
   fusion1.AddSpeed   (1.0, speed);
   fusion2.AddHeading (1.0, PE::SBasicSensor(72.0, 0.1));
   fusion1.AddAngSpeed(1.0, angSpeed);
   fusion2.AddPosition(1.0, PE::SPosition(50.00001401, 10.00013761, 0.1));
   fusion1.DoFusion();
   fusion2.DoFusion();
   EXPECT_NEAR(50.00001401, fusion1.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00013761, fusion1.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(71.99992566, fusion1.GetHeading().Value     , 0.00000001); //72deg
   EXPECT_NEAR(18.00005889, fusion1.GetAngSpeed().Value     , 0.00000001);//18 deg/s
   EXPECT_NEAR( 9.99972553, fusion1.GetSpeed().Value        , 0.00000001);//10 m/s
   EXPECT_NEAR( 0.14000030, fusion1.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.22847774, fusion1.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 0.20783140, fusion1.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.16000585, fusion1.GetSpeed().Accuracy        , 0.00000001);

   EXPECT_EQ(fusion1.GetPosition().Latitude     ,fusion2.GetPosition().Latitude);
   EXPECT_EQ(fusion1.GetPosition().Longitude    ,fusion2.GetPosition().Longitude);
   EXPECT_EQ(fusion1.GetHeading().Value         ,fusion2.GetHeading().Value);
   EXPECT_EQ(fusion1.GetAngSpeed().Value        ,fusion2.GetAngSpeed().Value);
   EXPECT_EQ(fusion1.GetSpeed().Value           ,fusion2.GetSpeed().Value);
   EXPECT_EQ(fusion1.GetPosition().HorizontalAcc,fusion2.GetPosition().HorizontalAcc);
   EXPECT_EQ(fusion1.GetHeading().Accuracy      ,fusion2.GetHeading().Accuracy);
   EXPECT_EQ(fusion1.GetAngSpeed().Accuracy     ,fusion2.GetAngSpeed().Accuracy);
   EXPECT_EQ(fusion1.GetSpeed().Accuracy        ,fusion2.GetSpeed().Accuracy);
}


TEST_F(PECFusionSensorTest, test_one_circle_left_by_position )
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


TEST_F(PECFusionSensorTest, test_one_circle_left_by_permanent_angular_and_linear_speed )
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
   EXPECT_NEAR( 0.23333505, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.22000000, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 0.12000000, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.13333333, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddSpeed(2.0, speed);
   fusion.AddAngSpeed(2.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR(50.00005467, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00026176, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(54.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 0.37879629, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.34071005, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(3.0, speed);
   fusion.AddAngSpeed(3.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR(50.00011800, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00036029, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(36.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 0.52765061, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.46142073, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(4.0, speed);
   fusion.AddAngSpeed(4.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR(50.00019780, fusion.GetPosition().Latitude,  0.0000001);
   EXPECT_NEAR(10.00042354, fusion.GetPosition().Longitude, 0.0000001);
   EXPECT_NEAR(18.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 0.67739318, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.58213141, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(5.0, speed);
   fusion.AddAngSpeed(5.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR(50.00028626, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00044534, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 0.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 0.82738222, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.70284209, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(10.0, speed);
   fusion.AddAngSpeed(10.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR( 50.00057252, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(270.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(  1.72775686, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  1.26123272, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(15.0, speed);
   fusion.AddAngSpeed(15.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR( 50.00028626, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(  9.99955464, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(180.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(  2.64434618, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  1.82333277, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(20.0, speed);
   fusion.AddAngSpeed(20.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 90.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 18.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  3.56398222, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  2.38511214, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR(  0.11235587, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR(  0.18330969, fusion.GetSpeed().Accuracy        , 0.00000001);
}


TEST_F(PECFusionSensorTest, test_quater_circle_left_by_decreasing_angular_and_permanent_linear_speed )
{
   PE::SBasicSensor  heading    ( 90.0,0.1);
   PE::SBasicSensor angSpeed200 ( 20.0,0.1); 
   PE::SBasicSensor angSpeed190 ( 19.0,0.1); 
   PE::SBasicSensor angSpeed160 ( 16.0,0.1);
   PE::SBasicSensor angSpeed120 ( 12.0,0.1);
   PE::SBasicSensor angSpeed090 (  9.0,0.1);
   PE::SBasicSensor angSpeed080 (  8.0,0.1);
   PE::SBasicSensor angSpeed070 (  7.0,0.1);
   PE::SBasicSensor angSpeed060 (  6.0,0.1);
   PE::SBasicSensor angSpeed050 (  5.0,0.1);
   PE::SBasicSensor angSpeed040 (  4.0,0.1);
   PE::SBasicSensor angSpeed030 (  3.0,0.1);
   PE::SBasicSensor angSpeed000 (  0.0,0.1);
   PE::SBasicSensor    speed    ( 10.0,0.1); //10 m/s
   PE::SPosition         pos    ( 50.0,10.0, 0.1);
   PE::CFusionSensor fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed200,speed);

   //start dirrect driving 
   fusion.AddSpeed   (1.0, speed);
   fusion.AddAngSpeed(1.0, angSpeed190); //71
   fusion.DoFusion();
//    With normal MergeSensor
//    EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
//    EXPECT_NEAR( 70.66666666, fusion.GetHeading().Value     , 0.00000001);
//    EXPECT_NEAR( 19.33333333, fusion.GetAngSpeed().Value    , 0.00000001);
//    EXPECT_NEAR( 00.13333333, fusion.GetAngSpeed().Accuracy , 0.00000001);
// With extended MergeSensorEx
   EXPECT_NEAR( 70.80000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 00.220000000, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 19.199999999, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.12000000, fusion.GetAngSpeed().Accuracy , 0.00000001);

   fusion.AddSpeed   (2.0, speed);
   fusion.AddAngSpeed(2.0, angSpeed160); //55
   fusion.DoFusion();
//    With normal MergeSensor
//    EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
//    EXPECT_NEAR( 53.93939393, fusion.GetHeading().Value     , 0.00000001);
//    EXPECT_NEAR( 16.72727272, fusion.GetAngSpeed().Value    , 0.00000001);
//    EXPECT_NEAR( 00.14545454, fusion.GetAngSpeed().Accuracy , 0.00000001);
// With extended MergeSensorEx
   EXPECT_NEAR( 54.44497041, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 00.34071005, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 16.35502958, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.12071005, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (3.0, speed);
   fusion.AddAngSpeed(3.0, angSpeed120); //43
   fusion.DoFusion();
//    With normal MergeSensor
//    EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
//    EXPECT_NEAR( 41.22621564, fusion.GetHeading().Value     , 0.00000001);
//    EXPECT_NEAR( 12.71317829, fusion.GetAngSpeed().Value    , 0.00000001);
//    EXPECT_NEAR( 00.14883720, fusion.GetAngSpeed().Accuracy , 0.00000001);
// With extended MergeSensorEx
   EXPECT_NEAR( 42.106667119, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 00.461420737, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 12.338303294, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (4.0, speed);
   fusion.AddAngSpeed(4.0, angSpeed090);//34
   fusion.DoFusion();
//    With normal MergeSensor ???
//    EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
//    EXPECT_NEAR( 17.00000000, fusion.GetHeading().Value     , 0.00000001);
//    EXPECT_NEAR( 16.50000000, fusion.GetAngSpeed().Value    , 0.00000001);
//    EXPECT_NEAR( 00.14883720, fusion.GetAngSpeed().Accuracy , 0.00000001);
// With extended MergeSensorEx
   EXPECT_NEAR( 32.906540883, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 00.582131415, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 09.200126235, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (5.0, speed);
   fusion.AddAngSpeed(5.0, angSpeed080);//26
   fusion.DoFusion();
//    With normal MergeSensor ???
//    EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
//    EXPECT_NEAR(  8.00000000, fusion.GetHeading().Value     , 0.00000001);
//    EXPECT_NEAR(  9.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
//    EXPECT_NEAR( 00.14883720, fusion.GetAngSpeed().Accuracy , 0.00000001);
// With extended MergeSensorEx
   EXPECT_NEAR( 24.901604892, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 00.702842093, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 08.004935990, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (6.0, speed);
   fusion.AddAngSpeed(6.0, angSpeed070);//19
   fusion.DoFusion();
//    With normal MergeSensor ???
//    EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
//    EXPECT_NEAR(  8.00000000, fusion.GetHeading().Value     , 0.00000001);
//    EXPECT_NEAR(  9.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
//    EXPECT_NEAR( 00.14883720, fusion.GetAngSpeed().Accuracy , 0.00000001);
// With extended MergeSensorEx
   EXPECT_NEAR( 17.758648556, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 00.823552771, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 07.142956336, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (7.0, speed);
   fusion.AddAngSpeed(7.0, angSpeed060);//13
   fusion.DoFusion();
   //    With normal MergeSensor ???
   //    EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   //    EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   //    EXPECT_NEAR(  8.00000000, fusion.GetHeading().Value     , 0.00000001);
   //    EXPECT_NEAR(  9.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
   //    EXPECT_NEAR( 00.14883720, fusion.GetAngSpeed().Accuracy , 0.00000001);
   // With extended MergeSensorEx
   EXPECT_NEAR( 11.713287341, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 00.944263449, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 06.045361214, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (8.0, speed);
   fusion.AddAngSpeed(8.0, angSpeed050);//7
   fusion.DoFusion();
//    With normal MergeSensor ???
//    EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
//    EXPECT_NEAR(  8.00000000, fusion.GetHeading().Value     , 0.00000001);
//    EXPECT_NEAR(  9.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
//    EXPECT_NEAR( 00.14883720, fusion.GetAngSpeed().Accuracy , 0.00000001);
// With extended MergeSensorEx
   EXPECT_NEAR( 06.598915954, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 01.064974127, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 05.114371387, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (9.0, speed);
   fusion.AddAngSpeed(9.0, angSpeed040);//3
   fusion.DoFusion();
//    With normal MergeSensor ???
//    EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
//    EXPECT_NEAR(  8.00000000, fusion.GetHeading().Value     , 0.00000001);
//    EXPECT_NEAR(  9.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
//    EXPECT_NEAR( 00.14883720, fusion.GetAngSpeed().Accuracy , 0.00000001);
// With extended MergeSensorEx
   EXPECT_NEAR( 02.533342128, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 01.185684806, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 04.065573825, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);

   fusion.AddSpeed   (10.0, speed);
   fusion.AddAngSpeed(10.0, angSpeed030);//0
   fusion.DoFusion();
//    With normal MergeSensor ???
//    EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
//    EXPECT_NEAR(  8.00000000, fusion.GetHeading().Value     , 0.00000001);
//    EXPECT_NEAR(  9.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
//    EXPECT_NEAR( 00.14883720, fusion.GetAngSpeed().Accuracy , 0.00000001);
// With extended MergeSensorEx
   EXPECT_NEAR(359.433263216, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 01.306395484, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 03.100078912, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);

   fusion.AddSpeed   (11.0, speed);
   fusion.AddAngSpeed(11.0, angSpeed000);//0
   fusion.DoFusion();
//    With normal MergeSensor ???
//    EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
//    EXPECT_NEAR(  8.00000000, fusion.GetHeading().Value     , 0.00000001);
//    EXPECT_NEAR(  9.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
//    EXPECT_NEAR( 00.14883720, fusion.GetAngSpeed().Accuracy , 0.00000001);
// With extended MergeSensorEx
   EXPECT_NEAR(359.064689865, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 01.427106162, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 00.368573350, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (12.0, speed);
   fusion.AddAngSpeed(12.0, angSpeed000);//0
   fusion.DoFusion();
//    With normal MergeSensor ???
//    EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
//    EXPECT_NEAR(  8.00000000, fusion.GetHeading().Value     , 0.00000001);
//    EXPECT_NEAR(  9.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
//    EXPECT_NEAR( 00.14883720, fusion.GetAngSpeed().Accuracy , 0.00000001);
// With extended MergeSensorEx
   EXPECT_NEAR(359.325310581, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 01.547816840, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR(-00.260620715, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (13.0, speed);
   fusion.AddAngSpeed(13.0, angSpeed000);//0
   fusion.DoFusion();
//    With normal MergeSensor ???
//    EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
//    EXPECT_NEAR(  8.00000000, fusion.GetHeading().Value     , 0.00000001);
//    EXPECT_NEAR(  9.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
//    EXPECT_NEAR( 00.14883720, fusion.GetAngSpeed().Accuracy , 0.00000001);
// With extended MergeSensorEx
   EXPECT_NEAR(359.141023906, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 01.668527518, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 00.184286675, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

