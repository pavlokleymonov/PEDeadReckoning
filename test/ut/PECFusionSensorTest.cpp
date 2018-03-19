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
   EXPECT_NEAR(10.0001300, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.200, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.640, fusion.GetHeading().Accuracy,0.001);
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
   EXPECT_NEAR(71.99998642, fusion1.GetHeading().Value     , 0.00000001); //72deg
   EXPECT_NEAR(18.00001028, fusion1.GetAngSpeed().Value     , 0.00000001);//18 deg/s
   EXPECT_NEAR( 9.99981838, fusion1.GetSpeed().Value        , 0.00000001);//10 m/s
   EXPECT_NEAR( 0.12054797, fusion1.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.13768000, fusion1.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 0.13533165, fusion1.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.14117846, fusion1.GetSpeed().Accuracy        , 0.00000001);

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
   EXPECT_NEAR(71.99992756, fusion.GetHeading().Value     , 0.00000001); //72deg
   EXPECT_NEAR(18.00002741, fusion.GetAngSpeed().Value     , 0.00000001);//18 deg/s
   EXPECT_NEAR( 9.99965692, fusion.GetSpeed().Value        , 0.00000001);//10 m/s
   EXPECT_NEAR( 0.11999991, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.36185902, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 0.23589552, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.20000915, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(2.0, PE::SPosition(50.00005467, 10.00026176, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(50.00005467, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00026176, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(54.00034495, fusion.GetHeading().Value     , 0.00000001);//54 deg
   EXPECT_NEAR(17.99993133, fusion.GetAngSpeed().Value     , 0.00000001);
   EXPECT_NEAR( 9.99983825, fusion.GetSpeed().Value        , 0.00000001);
   EXPECT_NEAR( 0.11497714, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.90006767, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 0.56726189, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.26187078, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(3.0, PE::SPosition(50.00011800, 10.00036029, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(50.00011800, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00036029, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(36.00218153, fusion.GetHeading().Value     , 0.00000001);//36 deg
   EXPECT_NEAR(17.99934603, fusion.GetAngSpeed().Value     , 0.00000001);
   EXPECT_NEAR(10.00025271, fusion.GetSpeed().Value        , 0.00000001);
   EXPECT_NEAR( 0.11288296, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 1.70302022, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 1.36610790, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.25972916, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(4.0, PE::SPosition(50.00019780, 10.00042354, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(50.00019780, fusion.GetPosition().Latitude,  0.0000001);
   EXPECT_NEAR(10.00042354, fusion.GetPosition().Longitude, 0.0000001);
   EXPECT_NEAR(17.99545459, fusion.GetHeading().Value     , 0.00000001);//18 deg
   EXPECT_NEAR(18.00203941, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 9.99977041, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR( 0.11295879, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 2.76931637, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 3.13882806, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.25760322, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(5.0, PE::SPosition(50.00028626, 10.00044534, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(50.00028626, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00044534, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 0.00477168, fusion.GetHeading().Value     , 0.00000001);//0deg
   EXPECT_NEAR(17.99675653, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 9.99988164, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR( 0.11293015, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 4.07637003, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 6.21310851, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.25855204, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(10.0, PE::SPosition(50.00057252, 10.00000000, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00057251, fusion.GetPosition().Latitude , 0.00000001); //~6cm difference
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(269.99650486, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 17.99917951, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR(10.000111622, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  0.10017246, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  4.86953517, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 10.34168083, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR(  0.34105640, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(15.0, PE::SPosition(50.00028626, 9.99955464, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00028626, fusion.GetPosition().Latitude , 0.00000001);//~8cm difference
   EXPECT_NEAR(  9.99955464, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(180.00597659, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 17.99933485, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR(  9.02189557, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  0.10016550, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  5.60098306, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 11.78163795, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR(  0.31619709, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(20.0, PE::SPosition(50.00000000, 10.00000000, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 89.99186036, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 18.00152919, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 8.982393507, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  0.10020033, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  6.34435398, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 13.44451550, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR(  0.31822952, fusion.GetSpeed().Accuracy        , 0.00000001);
}


TEST_F(PECFusionSensorTest, test_one_circle_left_by_position_and_heading )
{
   PE::SBasicSensor  heading  ( 90.0,0.1);
   PE::SBasicSensor angSpeed  ( 18.0,0.1); //turn left 18deg/s
   PE::SBasicSensor    speed  ( 10.0,0.1); //10 m/s
   PE::SPosition         pos  ( 50.0,10.0, 0.1);
   PE::CFusionSensor fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);

   fusion.AddHeading(5.0, PE::SBasicSensor(0.0, 0.1));
   fusion.AddPosition(5.0, PE::SPosition(50.00028626, 10.00044534, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00028626, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00044534, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(359.99998644, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 18.00004791, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR(  9.99989360, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  0.10311410, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.11529742, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR(  0.68031986, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR(  0.24067265, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(10.0, PE::SPosition(50.00057252, 10.00000000, 0.1));
   fusion.AddHeading(10.0, PE::SBasicSensor(270.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00057251, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(270.00001809, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 17.99979933, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR(  9.99980921, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  0.10126226, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.11179645, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR(  1.01829438, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR(  0.22791207, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddHeading(15.0, PE::SBasicSensor(180.0, 0.1));
   fusion.AddPosition(15.0, PE::SPosition(50.00028626, 9.99955464, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00028626, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(  9.99955464, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(180.00003832, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 17.99956415, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 10.00000407, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  0.10122417, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.11171861, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR(  0.96684302, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR(  0.22668775, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddPosition(20.0, PE::SPosition(50.00000000, 10.00000000, 0.1));
   fusion.AddHeading(20.0, PE::SBasicSensor(90.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 89.99995474, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 18.00050581, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 10.00015263, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  0.10124956, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.11174179, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR(  0.97079677, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR(  0.22674578, fusion.GetSpeed().Accuracy        , 0.00000001);
}


TEST_F(PECFusionSensorTest, test_one_circle_left_by_position_and_heading_with_one_fusion )
{
   PE::SBasicSensor  heading  ( 90.0,0.1);
   PE::SBasicSensor angSpeed  ( 18.0,0.1); //turn left 18deg/s
   PE::SBasicSensor    speed  ( 10.0,0.1); //10 m/s
   PE::SPosition         pos  ( 50.0,10.0, 0.1);
   PE::CFusionSensor fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);

   fusion.AddHeading ( 5.0, PE::SBasicSensor(0.0, 0.1));
   fusion.AddPosition( 5.0, PE::SPosition(50.00028626, 10.00044534, 0.1));
   fusion.AddPosition(10.0, PE::SPosition(50.00057252, 10.00000000, 0.1));
   fusion.AddHeading (10.0, PE::SBasicSensor(270.0, 0.1));
   fusion.AddHeading (15.0, PE::SBasicSensor(180.0, 0.1));
   fusion.AddPosition(15.0, PE::SPosition(50.00028626, 9.99955464, 0.1));
   fusion.AddPosition(20.0, PE::SPosition(50.00000000, 10.00000000, 0.1));
   fusion.AddHeading (20.0, PE::SBasicSensor(90.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 89.99995474, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 18.00050581, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 10.00015263, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  0.10124956, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.11174179, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR(  0.97079677, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR(  0.22674578, fusion.GetSpeed().Accuracy        , 0.00000001);
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
   EXPECT_NEAR( 0.22000162, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.22000000, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 0.12000000, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.12000000, fusion.GetSpeed().Accuracy        , 0.00000001);

   fusion.AddSpeed(2.0, speed);
   fusion.AddAngSpeed(2.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR(50.00005467, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00026176, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(54.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 0.34071770, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.34071005, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(3.0, speed);
   fusion.AddAngSpeed(3.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR(50.00011800, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00036029, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(36.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 0.46144334, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.46142073, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(4.0, speed);
   fusion.AddAngSpeed(4.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR(50.00019780, fusion.GetPosition().Latitude,  0.0000001);
   EXPECT_NEAR(10.00042354, fusion.GetPosition().Longitude, 0.0000001);
   EXPECT_NEAR(18.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 0.58218407, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.58213141, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(5.0, speed);
   fusion.AddAngSpeed(5.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR(50.00028626, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00044534, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 0.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 0.70294763, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.70284209, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(10.0, speed);
   fusion.AddAngSpeed(10.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR( 50.00057252, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(270.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(  1.26164392, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  1.26123272, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(15.0, speed);
   fusion.AddAngSpeed(15.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR( 50.00028626, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(  9.99955464, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(180.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(  1.82466783, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  1.82333277, fusion.GetHeading().Accuracy      , 0.00000001);

   fusion.AddSpeed(20.0, speed);
   fusion.AddAngSpeed(20.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 90.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 18.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  2.38851642, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  2.38511214, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR(  0.11235587, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR(  0.11235587, fusion.GetSpeed().Accuracy        , 0.00000001);
}


TEST_F(PECFusionSensorTest, test_one_circle_right_by_angular_linear_speeds_and_heading )
{
   PE::SBasicSensor  heading  ( 90.0,0.1);
   PE::SBasicSensor angSpeed  ( -18.0,0.1); //turn right 18deg/s
   PE::SBasicSensor    speed  ( 10.0,0.1); //10 m/s
   PE::SPosition         pos  ( 50.0,10.0, 0.1);
   PE::CFusionSensor fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);

   fusion.AddSpeed   (0.2, speed);
   fusion.AddAngSpeed(0.3, angSpeed);
   fusion.AddSpeed   (0.8, speed);
   fusion.AddAngSpeed(0.9, angSpeed);
   fusion.AddHeading (1.0, PE::SBasicSensor(108.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 49.99998598, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(108.00000000, fusion.GetHeading().Value     , 0.00000001);

   fusion.AddSpeed   (1.2, speed);
   fusion.AddAngSpeed(1.3, angSpeed);
   fusion.AddSpeed   (1.8, speed);
   fusion.AddAngSpeed(1.9, angSpeed);
   fusion.AddHeading (2.0, PE::SBasicSensor(126.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 49.99994532, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00026176, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(126.00000000, fusion.GetHeading().Value     , 0.00000001);

   fusion.AddSpeed   (2.2, speed);
   fusion.AddAngSpeed(2.3, angSpeed);
   fusion.AddSpeed   (2.8, speed);
   fusion.AddAngSpeed(2.9, angSpeed);
   fusion.AddHeading (3.0, PE::SBasicSensor(144.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 49.99988199, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00036029, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(144.00000000, fusion.GetHeading().Value     , 0.00000001);

   fusion.AddSpeed   (3.2, speed);
   fusion.AddAngSpeed(3.3, angSpeed);
   fusion.AddSpeed   (3.8, speed);
   fusion.AddAngSpeed(3.9, angSpeed);
   fusion.AddHeading (4.0, PE::SBasicSensor(162.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 49.99980219, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00042354, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(162.00000000, fusion.GetHeading().Value     , 0.00000001);

   fusion.AddSpeed   (4.2, speed);
   fusion.AddAngSpeed(4.3, angSpeed);
   fusion.AddSpeed   (4.8, speed);
   fusion.AddAngSpeed(4.9, angSpeed);
   fusion.AddHeading (5.0, PE::SBasicSensor(180.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 49.99971373, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00044534, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(180.00000000, fusion.GetHeading().Value     , 0.00000001);

   fusion.AddSpeed   ( 9.2, speed);
   fusion.AddAngSpeed( 9.3, angSpeed);
   fusion.AddSpeed   ( 9.8, speed);
   fusion.AddAngSpeed( 9.9, angSpeed);
   fusion.AddAngSpeed(10.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR( 49.99942747, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(270.00000000, fusion.GetHeading().Value     , 0.00000001);

   fusion.AddSpeed   (19.2, speed);
   fusion.AddAngSpeed(19.3, angSpeed);
   fusion.AddSpeed   (19.8, speed);
   fusion.AddAngSpeed(19.9, angSpeed);
   fusion.AddHeading (20.0, PE::SBasicSensor(90.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 90.00000000, fusion.GetHeading().Value     , 0.00000001);
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
// With extended MergeSensor
   EXPECT_NEAR( 00.220001621, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 70.800000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 00.220000000, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 19.199999999, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120000000, fusion.GetAngSpeed().Accuracy , 0.00000001);

   fusion.AddSpeed   (2.0, speed);
   fusion.AddAngSpeed(2.0, angSpeed160); //55
   fusion.DoFusion();
// With extended MergeSensor
   EXPECT_NEAR( 00.34071770, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 54.44497041, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 00.34071005, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 16.35502958, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.12071005, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (3.0, speed);
   fusion.AddAngSpeed(3.0, angSpeed120); //43
   fusion.DoFusion();
// With extended MergeSensor
   EXPECT_NEAR( 00.461443346, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 42.106667119, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 00.461420737, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 12.338303294, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (4.0, speed);
   fusion.AddAngSpeed(4.0, angSpeed090);//34
   fusion.DoFusion();
// With extended MergeSensor
   EXPECT_NEAR( 00.582184073, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 32.906540883, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 00.582131415, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 09.200126235, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (5.0, speed);
   fusion.AddAngSpeed(5.0, angSpeed080);//26
   fusion.DoFusion();
// With extended MergeSensor
   EXPECT_NEAR( 00.702947639, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 24.901604892, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 00.702842093, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 08.004935990, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (6.0, speed);
   fusion.AddAngSpeed(6.0, angSpeed070);//19
   fusion.DoFusion();
// With extended MergeSensor
   EXPECT_NEAR( 00.823743410 , fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 17.758648556, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 00.823552771, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 07.142956336, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (7.0, speed);
   fusion.AddAngSpeed(7.0, angSpeed060);//13
   fusion.DoFusion();
// With extended MergeSensor
   EXPECT_NEAR( 00.944582363, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 11.713287341, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 00.944263449, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 06.045361214, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (8.0, speed);
   fusion.AddAngSpeed(8.0, angSpeed050);//7
   fusion.DoFusion();
// With extended MergeSensor
   EXPECT_NEAR( 01.065477090, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 06.598915954, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 01.064974127, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 05.114371387, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (9.0, speed);
   fusion.AddAngSpeed(9.0, angSpeed040);//3
   fusion.DoFusion();
// With extended MergeSensor
   EXPECT_NEAR( 01.186441804, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 02.533342128, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 01.185684806, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 04.065573825, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);

   fusion.AddSpeed   (10.0, speed);
   fusion.AddAngSpeed(10.0, angSpeed030);//0
   fusion.DoFusion();
// With extended MergeSensor
   EXPECT_NEAR( 01.307492337, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(359.433263216, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 01.306395484, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 03.100078912, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);

   fusion.AddSpeed   (11.0, speed);
   fusion.AddAngSpeed(11.0, angSpeed000);//0
   fusion.DoFusion();
// With extended MergeSensor
   EXPECT_NEAR( 01.428646154, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(359.064689865, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 01.427106162, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 00.368573350, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (12.0, speed);
   fusion.AddAngSpeed(12.0, angSpeed000);//0
   fusion.DoFusion();
// With extended MergeSensor
   EXPECT_NEAR( 01.549922351, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(359.325310581, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 01.547816840, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR(-00.260620715, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);


   fusion.AddSpeed   (13.0, speed);
   fusion.AddAngSpeed(13.0, angSpeed000);//0
   fusion.DoFusion();
// With extended MergeSensor
   EXPECT_NEAR( 01.671341670, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(359.141023906, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR( 01.668527518, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR( 00.184286675, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR( 00.120710678, fusion.GetAngSpeed().Accuracy , 0.00000001);
}


TEST_F(PECFusionSensorTest, test_quater_circle_right_by_decreasing_angular_and_linear_speed_with_heading)
{
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

