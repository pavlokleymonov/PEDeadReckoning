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


TEST_F(PECFusionSensorTest, test_incorrect_first_timestamp )
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

   //same timestamp -ignored
   fusion.AddPosition(10.0, PE::SPosition(50.0000000, 10.0001000, 1));
   fusion.DoFusion();
   EXPECT_NEAR(10.0000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000000, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.000, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.300, fusion.GetHeading().Accuracy,0.001);

   //correct timestamp
   fusion.AddPosition(11.0, PE::SPosition(50.0000000, 10.0001500, 1));
   fusion.DoFusion();
   EXPECT_NEAR(11.0000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0001200, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.200, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.656, fusion.GetHeading().Accuracy,0.001);
}


TEST_F(PECFusionSensorTest, test_incorrect_position_timestamp )
{
   PE::SBasicSensor  heading  ( 90.0,0.1);
   PE::SBasicSensor angSpeed  (  0.0,0.1);
   PE::SBasicSensor    speed  (  0.0,0.1);
   PE::SPosition         pos  ( 50.0,10.0, 0.1);
   PE::CFusionSensor  fusion  ( 10.0,pos , heading, angSpeed, speed);

   fusion.AddPosition(10.0, PE::SPosition( 50.0,10.0, 0.1));
   fusion.AddPosition( 9.0, PE::SPosition( 50.1,10.1, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(50.0, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.0, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 0.1, fusion.GetPosition().HorizontalAcc, 0.00000001);
}


TEST_F(PECFusionSensorTest, test_incorrect_heading_timestamp )
{
   PE::SBasicSensor  heading  ( 90.0,0.1);
   PE::SBasicSensor angSpeed  (  0.0,0.1);
   PE::SBasicSensor    speed  (  0.0,0.1);
   PE::SPosition         pos  ( 50.0,10.0, 0.1);
   PE::CFusionSensor  fusion  ( 10.0,pos , heading, angSpeed, speed);

   fusion.AddHeading(10.0, heading);
   fusion.AddHeading( 9.0, PE::SBasicSensor(180.0, 0.02));
   fusion.DoFusion();
   EXPECT_NEAR(90.0, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR( 0.1, fusion.GetHeading().Accuracy, 0.00000001);
}


TEST_F(PECFusionSensorTest, test_incorrect_speed_timestamp )
{
   PE::SBasicSensor  heading  ( 90.0,0.1);
   PE::SBasicSensor angSpeed  (  0.0,0.1);
   PE::SBasicSensor    speed  (  1.0,0.1);
   PE::SPosition         pos  ( 50.0,10.0, 0.1);
   PE::CFusionSensor  fusion  ( 10.0,pos , heading, angSpeed, speed);

   fusion.AddSpeed(10.0, speed);
   fusion.AddSpeed( 9.0, PE::SBasicSensor(11.0, 0.02));
   fusion.DoFusion();
   EXPECT_NEAR( 1.0, fusion.GetSpeed().Value, 0.00000001);
   EXPECT_NEAR( 0.1, fusion.GetSpeed().Accuracy, 0.00000001);
}


TEST_F(PECFusionSensorTest, test_incorrect_ang_speed_timestamp )
{
   PE::SBasicSensor  heading  ( 90.0,0.1);
   PE::SBasicSensor angSpeed  (  0.0,0.1);
   PE::SBasicSensor    speed  (  0.0,0.1);
   PE::SPosition         pos  ( 50.0,10.0, 0.1);
   PE::CFusionSensor  fusion  ( 10.0,pos , heading, angSpeed, speed);

   fusion.AddAngSpeed(10.0, angSpeed);
   fusion.AddAngSpeed( 9.0, PE::SBasicSensor(99, 0.000001));
   fusion.DoFusion();
   EXPECT_NEAR(0.0, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR(0.1, fusion.GetAngSpeed().Accuracy, 0.00000001);
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
   EXPECT_NEAR(  0.22035583, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(108.00000000, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR(  0.12070878, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR(  0.13108763, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value, 0.00000001);
   EXPECT_NEAR(  0.14369224, fusion.GetSpeed().Accuracy, 0.00000001);

   fusion.AddSpeed   (1.2, speed);
   fusion.AddAngSpeed(1.3, angSpeed);
   fusion.AddSpeed   (1.8, speed);
   fusion.AddAngSpeed(1.9, angSpeed);
   fusion.AddHeading (2.0, PE::SBasicSensor(126.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 49.99994532, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00026176, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(126.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value     , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value     , 0.00000001);

   fusion.AddSpeed   (2.2, speed);
   fusion.AddAngSpeed(2.3, angSpeed);
   fusion.AddSpeed   (2.8, speed);
   fusion.AddAngSpeed(2.9, angSpeed);
   fusion.AddHeading (3.0, PE::SBasicSensor(144.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 49.99988199, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00036029, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(144.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value     , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value     , 0.00000001);

   fusion.AddSpeed   (3.2, speed);
   fusion.AddAngSpeed(3.3, angSpeed);
   fusion.AddSpeed   (3.8, speed);
   fusion.AddAngSpeed(3.9, angSpeed);
   fusion.AddHeading (4.0, PE::SBasicSensor(162.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 49.99980219, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00042354, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(162.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value     , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value     , 0.00000001);

   fusion.AddSpeed   (4.2, speed);
   fusion.AddAngSpeed(4.3, angSpeed);
   fusion.AddSpeed   (4.8, speed);
   fusion.AddAngSpeed(4.9, angSpeed);
   fusion.AddHeading (5.0, PE::SBasicSensor(180.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 49.99971373, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00044534, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(180.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value     , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value     , 0.00000001);

   fusion.AddSpeed   ( 9.2, speed);
   fusion.AddAngSpeed( 9.3, angSpeed);
   fusion.AddSpeed   ( 9.8, speed);
   fusion.AddAngSpeed( 9.9, angSpeed);
   fusion.AddAngSpeed(10.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR( 49.99942747, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(270.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value     , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value     , 0.00000001);

   fusion.AddSpeed   (19.2, speed);
   fusion.AddAngSpeed(19.3, angSpeed);
   fusion.AddSpeed   (19.8, speed);
   fusion.AddAngSpeed(19.9, angSpeed);
   fusion.AddHeading (20.0, PE::SBasicSensor(90.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(  2.72655522, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 90.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(  0.10072517, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR(  0.13049998, fusion.GetAngSpeed().Accuracy , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  0.14338150, fusion.GetSpeed().Accuracy    , 0.00000001);
}


TEST_F(PECFusionSensorTest, test_one_circle_right_by_full_sensors_set )
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
   PE::SPosition posByAngSpeedHead = fusion.GetPosition();
   EXPECT_NEAR( 49.99998598, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(108.00000000, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  0.22035583, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.12070878, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.13108763, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.14369224, fusion.GetSpeed().Accuracy, 0.00000001);

   //!!!-->POSITION ONLY<--!!!
   //reset fusion
   fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);
   fusion.AddPosition(1.0, posByAngSpeedHead);
   fusion.DoFusion();
   EXPECT_NEAR( 49.99998598, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(107.99999425, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR(-17.99999776, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 10.00000002, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  0.24826778, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.35729577, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.22840786, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.23376170, fusion.GetSpeed().Accuracy, 0.00000001);

   //!!!-->ANGULAR VELOCITY<--!!!
   //reset fusion
   fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);
   fusion.AddAngSpeed(1.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR( 49.99998598, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(108.00000000, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  0.30000221, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.22000000, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.12000000, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.20000000, fusion.GetSpeed().Accuracy, 0.00000001);

   //!!!-->SPEED<--!!!
   //reset fusion
   fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);
   fusion.AddSpeed(1.0, speed);
   fusion.DoFusion();
   EXPECT_NEAR( 49.99998598, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(108.00000000, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  0.22000301, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.30000000, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.20000000, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.12000000, fusion.GetSpeed().Accuracy, 0.00000001);

   //!!!-->HEAD<--!!!
   //reset fusion
   fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);
   fusion.AddHeading(1.0, PE::SBasicSensor(108.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 49.99998598, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(108.00000000, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  0.30000411, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.12000000, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.20000000, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.20000000, fusion.GetSpeed().Accuracy, 0.00000001);

   //!!!-->ANGULAR VELOCITY/SPEED/HEAD SAME TIMESTAMP<--!!!
   //reset fusion
   fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);
   fusion.AddAngSpeed(1.0, angSpeed);
   fusion.AddSpeed(1.0, speed);
   fusion.AddHeading(1.0, PE::SBasicSensor(108.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 49.99998598, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(108.00000000, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  0.22000162, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.12054794, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.12000000, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.12000000, fusion.GetSpeed().Accuracy, 0.00000001);


   //!!!-->ANGULAR VELOCITY/SPEED/HEAD CLOSED TIMESTAMP<--!!!
   //reset fusion
   fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);
   fusion.AddAngSpeed(0.99, angSpeed);
   fusion.AddSpeed(0.99, speed);
   fusion.AddHeading(1.0, PE::SBasicSensor(108.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 49.99998598, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(108.00000000, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  0.21997448, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.12054747, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.12115886, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.12115886, fusion.GetSpeed().Accuracy, 0.00000001);


   //!!!-->ANGULAR VELOCITY/SPEED/HEAD 2xTIMES CLOSED TIMESTAMP<--!!!
   //reset fusion
   fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);
   fusion.AddAngSpeed(0.98, angSpeed);
   fusion.AddSpeed(0.98, speed);
   fusion.AddAngSpeed(0.99, angSpeed);
   fusion.AddSpeed(0.99, speed);
   fusion.AddHeading(1.0, PE::SBasicSensor(108.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 49.99998598, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(108.00000000, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  0.21970556, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.12054298, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.10964537, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.10964537, fusion.GetSpeed().Accuracy, 0.00000001);

   //!!!-->ANGULAR VELOCITY/SPEED/HEAD 4xTIMES CLOSED TIMESTAMP ALL TIMESTAMP ARE DIFERENT<--!!!
   //reset fusion
   fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);
   fusion.AddAngSpeed(0.92, angSpeed);
   fusion.AddSpeed(0.93, speed);
   fusion.AddAngSpeed(0.94, angSpeed);
   fusion.AddSpeed(0.95, speed);
   fusion.AddAngSpeed(0.96, angSpeed);
   fusion.AddSpeed(0.97, speed);
   fusion.AddAngSpeed(0.98, angSpeed);
   fusion.AddSpeed(0.99, speed);
   fusion.AddHeading(1.0, PE::SBasicSensor(108.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 1.000000000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 49.99998598, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(108.00000000, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  0.28543866, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.12052571, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.10536281, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.10432531, fusion.GetSpeed().Accuracy, 0.00000001);


   //!!!-->ANGULAR VELOCITY/SPEED/HEAD 4xTIMES STABLE TIMESTAMP ITERVAL 0.25SEC<--!!!
   //reset fusion
   fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);
   fusion.AddAngSpeed(0.23, angSpeed);
   fusion.AddSpeed(0.24, speed);
   fusion.DoFusion();
   EXPECT_NEAR( 49.999999186, fusion.GetPosition().Latitude , 0.000000001);
   EXPECT_NEAR( 10.000033546, fusion.GetPosition().Longitude, 0.000000001);
   fusion.AddAngSpeed(0.48, angSpeed);
   fusion.AddSpeed(0.49, speed);
   fusion.DoFusion();
   EXPECT_NEAR( 49.999996614, fusion.GetPosition().Latitude , 0.000000001);
   EXPECT_NEAR( 10.000068285, fusion.GetPosition().Longitude, 0.000000001);
   fusion.AddAngSpeed(0.73, angSpeed);
   fusion.AddSpeed(0.74, speed);
   fusion.DoFusion();
   EXPECT_NEAR( 49.999992299, fusion.GetPosition().Latitude , 0.000000001);
   EXPECT_NEAR( 10.000102603, fusion.GetPosition().Longitude, 0.000000001);
   fusion.AddAngSpeed(0.98, angSpeed);
   fusion.AddSpeed(0.99, speed);
   fusion.AddHeading(1.0, PE::SBasicSensor(108.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 1.000000000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 49.99998598, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(108.00000000, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  0.23401438, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.12039786, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.11631371, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.11516883, fusion.GetSpeed().Accuracy, 0.00000001);


   //!!!-->GNSS/ANGULAR VELOCITY/SPEED/HEAD 4xTIMES STABLE TIMESTAMP ITERVAL 0.25SEC<--!!!
   //reset fusion
   fusion = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);
   fusion.AddAngSpeed(0.22, angSpeed);
   fusion.AddSpeed(0.23, speed);
   fusion.AddPosition(0.24, PE::SPosition(49.999999186, 10.000033546, 0.1));
   fusion.AddAngSpeed(0.47, angSpeed);
   fusion.AddSpeed(0.48, speed);
   fusion.AddPosition(0.49, PE::SPosition(49.999996614, 10.000068285, 0.1));
   fusion.AddAngSpeed(0.72, angSpeed);
   fusion.AddSpeed(0.73, speed);
   fusion.AddPosition(0.74, PE::SPosition(49.999992299, 10.000102603, 0.1));
   fusion.AddAngSpeed(0.98, angSpeed);
   fusion.AddSpeed(0.99, speed);
   fusion.AddHeading(1.0, PE::SBasicSensor(108.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 1.000000000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 49.99998598, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(108.00000000, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  0.15172528, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.12039957, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.11660811, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.11546026, fusion.GetSpeed().Accuracy, 0.00000001);
}


TEST_F(PECFusionSensorTest, test_one_circle_right_by_angular_linear_speeds_and_heading_one_fusion )
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
   fusion.AddSpeed   (1.2, speed);
   fusion.AddAngSpeed(1.3, angSpeed);
   fusion.AddSpeed   (1.8, speed);
   fusion.AddAngSpeed(1.9, angSpeed);
   fusion.AddHeading (2.0, PE::SBasicSensor(126.0, 0.1));
   fusion.AddSpeed   (2.2, speed);
   fusion.AddAngSpeed(2.3, angSpeed);
   fusion.AddSpeed   (2.8, speed);
   fusion.AddAngSpeed(2.9, angSpeed);
   fusion.AddHeading (3.0, PE::SBasicSensor(144.0, 0.1));
   fusion.AddSpeed   (3.2, speed);
   fusion.AddAngSpeed(3.3, angSpeed);
   fusion.AddSpeed   (3.8, speed);
   fusion.AddAngSpeed(3.9, angSpeed);
   fusion.AddHeading (4.0, PE::SBasicSensor(162.0, 0.1));
   fusion.AddSpeed   (4.2, speed);
   fusion.AddAngSpeed(4.3, angSpeed);
   fusion.AddSpeed   (4.8, speed);
   fusion.AddAngSpeed(4.9, angSpeed);
   fusion.AddHeading (5.0, PE::SBasicSensor(180.0, 0.1));
   fusion.AddSpeed   ( 9.2, speed);
   fusion.AddAngSpeed( 9.3, angSpeed);
   fusion.AddSpeed   ( 9.8, speed);
   fusion.AddAngSpeed( 9.9, angSpeed);
   fusion.AddAngSpeed(10.0, angSpeed);
   fusion.AddSpeed   (19.2, speed);
   fusion.AddAngSpeed(19.3, angSpeed);
   fusion.AddSpeed   (19.8, speed);
   fusion.AddAngSpeed(19.9, angSpeed);
   fusion.AddHeading (20.0, PE::SBasicSensor(90.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(  2.72655522, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 90.00000000, fusion.GetHeading().Value     , 0.00000001);
   EXPECT_NEAR(  0.10072517, fusion.GetHeading().Accuracy  , 0.00000001);
   EXPECT_NEAR(-18.00000000, fusion.GetAngSpeed().Value    , 0.00000001);
   EXPECT_NEAR(  0.13049998, fusion.GetAngSpeed().Accuracy , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetSpeed().Value       , 0.00000001);
   EXPECT_NEAR(  0.14338150, fusion.GetSpeed().Accuracy    , 0.00000001);
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


TEST_F(PECFusionSensorTest, test_gnss_based_only_fusion_hamburg_airport_b433)
{
   //test drive 12 sec.
   PE::CFusionSensor fusion = PE::CFusionSensor(0.0,PE::SPosition(), PE::SBasicSensor(), PE::SBasicSensor(), PE::SBasicSensor());

   //#0 UTC=2016-12-16T09:27:44
   fusion.AddPosition(6216.331, PE::SPosition(53.640096,10.004298,2.0));
   fusion.AddSpeed   (6216.331, PE::SBasicSensor(70.199998/3.6, 0.36/3.6)); //km/h -> m/s
   fusion.AddHeading (6216.331, PE::SBasicSensor(89.400002,0.3));
   fusion.DoFusion();
   EXPECT_NEAR( 6216.331000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.64009600, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00429800, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 89.40000200, fusion.GetHeading().Value, 0.00000001);
   EXPECT_FALSE( fusion.GetAngSpeed().IsValid() ); //expected false since there is no valid heading or angular speed before
   EXPECT_NEAR( 19.49999944, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  2.00000000, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.30000000, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.10000000, fusion.GetSpeed().Accuracy, 0.00000001);

   //#1 UTC=2016-12-16T09:27:45
   fusion.AddSpeed   (6217.332, PE::SBasicSensor(71.099998/3.6, 0.36/3.6)); //km/h -> m/s
   fusion.AddHeading (6217.332, PE::SBasicSensor(93,0.3));
   fusion.AddPosition(6217.332, PE::SPosition(53.640094,10.0046,2.0));
   fusion.DoFusion();
   EXPECT_NEAR( 6217.332000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.64009401, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00459748, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 92.27913128, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR( -2.47727200, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 19.70020953, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  2.16499545, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.38028969, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  6.28558873, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.12347274, fusion.GetSpeed().Accuracy, 0.00000001);

   //#2 UTC=2016-12-16T09:27:46
   fusion.AddPosition(6218.331, PE::SPosition(53.640079,10.004898,1.0));
   fusion.AddSpeed   (6218.331, PE::SBasicSensor(70.199997/3.6, 0.36/3.6)); //km/h -> m/s
   fusion.AddHeading (6218.331, PE::SBasicSensor(97,0.3));
   fusion.DoFusion();
   EXPECT_NEAR( 6218.331000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.64007965, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00489727, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 97.00018953, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR( -4.69158548, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 19.55690782, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  1.20695439, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.32473473, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  6.37191355, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.12510543, fusion.GetSpeed().Accuracy, 0.00000001);

   //#3 UTC=2016-12-16T09:27:47
   fusion.AddPosition(6219.331, PE::SPosition(53.640051,10.005193,1.0));
   fusion.AddSpeed   (6219.331, PE::SBasicSensor(70.199997/3.6, 0.54/3.6)); //km/h -> m/s
   fusion.AddHeading (6219.331, PE::SBasicSensor(99.699997,0.45));
   fusion.DoFusion();
   EXPECT_NEAR( 6219.331000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.64004999, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00519096, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 99.73697560, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR( -4.75741976, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 19.42636042, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  1.13928524, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.51742610, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  4.57717445, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.18937066, fusion.GetSpeed().Accuracy, 0.00000001);

   //#4 UTC=2016-12-16T09:27:48
   fusion.AddPosition(6220.331, PE::SPosition(53.640013,10.005485,1.0));
   fusion.AddSpeed   (6220.331, PE::SBasicSensor(69.300003/3.6, 0.54/3.6)); //km/h -> m/s
   fusion.AddHeading (6220.331, PE::SBasicSensor(103.300003,0.45));
   fusion.DoFusion();
   EXPECT_NEAR( 6220.331000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.64001434, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00548261, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(103.31352664, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR( -4.17385811, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 19.30644154, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  1.12145958, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.52237937, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  4.96300158, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.19490305, fusion.GetSpeed().Accuracy, 0.00000001);

   //#5 UTC=2016-12-16T09:27:49
   fusion.AddPosition(6221.331, PE::SPosition(53.639968,10.005767,1.0));
   fusion.AddSpeed   (6221.331, PE::SBasicSensor(69.300003/3.6, 0.54/3.6)); //km/h -> m/s
   fusion.AddHeading (6221.331, PE::SBasicSensor(106.400002,0.45));
   fusion.DoFusion();
   EXPECT_NEAR( 6221.331000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63996734, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00576533, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(106.42158813, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR( -4.35529031, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 19.21602529, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  1.11777796, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.52024493, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  5.01938211, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.19490943, fusion.GetSpeed().Accuracy, 0.00000001);

   //#6 UTC=2016-12-16T09:27:50
   fusion.AddPosition(6222.332, PE::SPosition(53.639916,10.006045,1.0));
   fusion.AddSpeed   (6222.332, PE::SBasicSensor(69.300003/3.6, 0.36/3.6)); //km/h -> m/s
   fusion.AddHeading (6222.332, PE::SBasicSensor(109.5,0.45));
   fusion.DoFusion();
   EXPECT_NEAR( 6222.332000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63991523, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00604446, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(109.47545130, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR( -1.77533967, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 19.26293060, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  1.09964935, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.51977932, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  5.03915757, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.12405258, fusion.GetSpeed().Accuracy, 0.00000001);

   //#7 UTC=2016-12-16T09:27:51
   fusion.AddPosition(6223.332, PE::SPosition(53.639858,10.00632,1.0));
   fusion.AddSpeed   (6223.332, PE::SBasicSensor(69.300003/3.6, 0.36/3.6)); //km/h -> m/s
   fusion.AddHeading (6223.332, PE::SBasicSensor(109.400002,1.2));
   fusion.DoFusion();
   EXPECT_NEAR( 6223.332000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63985751, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00631956, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(109.37768358, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR(  0.19771054, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 19.22455187, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  1.09503700, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  1.58845172, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  5.01249398, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.12719968, fusion.GetSpeed().Accuracy, 0.00000001);

   //#8 UTC=2016-12-16T09:27:52
   fusion.AddPosition(6224.331, PE::SPosition(53.639789,10.006588,1.0));
   fusion.AddSpeed   (6224.331, PE::SBasicSensor(69.300003/3.6, 0.54/3.6)); //km/h -> m/s
   fusion.AddHeading (6224.331, PE::SBasicSensor(116.400002,0.15));
   fusion.DoFusion();
   EXPECT_NEAR( 6224.331000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63979362, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00659100, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(116.39951335, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR( -5.35366055, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 19.28408139, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  1.11107173, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.15667239, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  7.33175294, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.19042826, fusion.GetSpeed().Accuracy, 0.00000001);

   //#9 UTC=2016-12-16T09:27:53
   fusion.AddPosition(6225.332, PE::SPosition(53.639706,10.006846,1.0));
   fusion.AddSpeed   (6225.332, PE::SBasicSensor(70.199997/3.6, 0.18/3.6)); //km/h -> m/s
   fusion.AddHeading (6225.332, PE::SBasicSensor(119.800003,0.15));
   fusion.DoFusion();
   EXPECT_NEAR( 6225.332000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63970361, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00684392, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(119.80945976, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR( -7.61057758, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 19.49464227, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  1.08425503, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.15819904, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  3.99591203, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.05702599, fusion.GetSpeed().Accuracy, 0.00000001);

   //#10 UTC=2016-12-16T09:27:54
   fusion.AddPosition(6226.332, PE::SPosition(53.639611,10.0071,1.0));
   fusion.AddSpeed   (6226.332, PE::SBasicSensor(71.099998/3.6, 0.18/3.6)); //km/h -> m/s
   fusion.AddHeading (6226.332, PE::SBasicSensor(123.199997,0.15));
   fusion.DoFusion();
   EXPECT_NEAR( 6226.332000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63961082, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00709967, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(123.19948209, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR( -3.20455335, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 19.75849075, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  1.06686481, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.15970190, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  4.01340997, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.06201705, fusion.GetSpeed().Accuracy, 0.00000001);

   //#11 UTC=2016-12-16T09:27:55
   fusion.AddPosition(6227.332, PE::SPosition(53.639508,10.007345,1.0));
   fusion.AddSpeed   (6227.332, PE::SBasicSensor(71.099998/3.6, 0.36/3.6)); //km/h -> m/s
   fusion.AddHeading (6227.332, PE::SBasicSensor(127.099998,0.15));
   fusion.DoFusion();
   EXPECT_NEAR( 6227.332000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63950880, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00734539, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(127.10017207, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR( -3.94000825, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 19.73618462, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  1.07811456, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.15974825, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  3.98120033, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.11493489, fusion.GetSpeed().Accuracy, 0.00000001);
}

TEST_F(PECFusionSensorTest, test_dr_1sec_fusion_hamburg_airport_b433)
{
   //test drive 12 sec.
   PE::CFusionSensor fusion = PE::CFusionSensor(0.0,PE::SPosition(), PE::SBasicSensor(), PE::SBasicSensor(), PE::SBasicSensor());

   //#0 UTC=2016-12-16T09:27:44
   fusion.AddPosition(6216.331, PE::SPosition(53.640096,10.004298,2.0));
   fusion.AddHeading (6216.331, PE::SBasicSensor(89.400002,0.3));
   fusion.AddPosition(6217.332, PE::SPosition(53.640094,10.0046,2.0));
   fusion.AddHeading (6217.332, PE::SBasicSensor(93,0.3));
   fusion.AddPosition(6218.331, PE::SPosition(53.640079,10.004898,1.0));
   fusion.AddHeading (6218.331, PE::SBasicSensor(97,0.3));
   fusion.AddPosition(6219.331, PE::SPosition(53.640051,10.005193,1.0));
   fusion.AddHeading (6219.331, PE::SBasicSensor(99.699997,0.45));
   fusion.AddPosition(6220.331, PE::SPosition(53.640013,10.005485,1.0));
   fusion.AddHeading (6220.331, PE::SBasicSensor(103.300003,0.45));
   fusion.AddPosition(6221.331, PE::SPosition(53.639968,10.005767,1.0));
   fusion.AddHeading (6221.331, PE::SBasicSensor(106.400002,0.45));
   fusion.AddPosition(6222.332, PE::SPosition(53.639916,10.006045,1.0));
   fusion.AddHeading (6222.332, PE::SBasicSensor(109.5,0.45));
   fusion.AddPosition(6223.332, PE::SPosition(53.639858,10.00632,1.0));
   fusion.AddHeading (6223.332, PE::SBasicSensor(109.400002,1.2));
   fusion.AddPosition(6224.331, PE::SPosition(53.639789,10.006588,1.0));
   fusion.AddHeading (6224.331, PE::SBasicSensor(116.400002,0.15));
   fusion.AddPosition(6225.332, PE::SPosition(53.639706,10.006846,1.0));
   fusion.AddHeading (6225.332, PE::SBasicSensor(119.800003,0.15));
   fusion.AddPosition(6226.332, PE::SPosition(53.639611,10.0071,1.0));
   fusion.AddHeading (6226.332, PE::SBasicSensor(123.199997,0.15));
   //No GNSS data during 1 sec.
   fusion.AddSpeed   (6226.341, PE::SBasicSensor(20.93563265,0.1));
   fusion.AddAngSpeed(6226.383, PE::SBasicSensor(-3.927827042,0.1));
   fusion.AddSpeed   (6226.391, PE::SBasicSensor(20.0709,0.1));
   fusion.AddSpeed   (6226.442, PE::SBasicSensor(19.67735294,0.1));
   fusion.AddAngSpeed(6226.443, PE::SBasicSensor(-4.0100852,0.1));
   fusion.AddSpeed   (6226.491, PE::SBasicSensor(20.4805102,0.1));
   fusion.AddAngSpeed(6226.503, PE::SBasicSensor(-3.835286614,0.1));
   fusion.AddSpeed   (6226.541, PE::SBasicSensor(20.0709,0.1));
   fusion.AddAngSpeed(6226.563, PE::SBasicSensor(-3.968956121,0.1));
   fusion.AddSpeed   (6226.591, PE::SBasicSensor(19.62488,0.1));
   fusion.AddAngSpeed(6226.623, PE::SBasicSensor(-3.896980233,0.1));
   fusion.AddSpeed   (6226.642, PE::SBasicSensor(19.67735294,0.1));
   fusion.AddAngSpeed(6226.683, PE::SBasicSensor(-3.794157535,0.1));
   fusion.AddSpeed   (6226.691, PE::SBasicSensor(20.4805102,0.1));
   fusion.AddSpeed   (6226.742, PE::SBasicSensor(19.67735294,0.1));
   fusion.AddAngSpeed(6226.743, PE::SBasicSensor(-3.753028456,0.1));
   fusion.AddSpeed   (6226.791, PE::SBasicSensor(20.4805102,0.1));
   fusion.AddAngSpeed(6226.803, PE::SBasicSensor(-3.732463917,0.1));
   fusion.AddSpeed   (6226.841, PE::SBasicSensor(20.0709,0.1));
   fusion.AddAngSpeed(6226.863, PE::SBasicSensor(-3.670770299,0.1));
   fusion.AddSpeed   (6226.891, PE::SBasicSensor(20.0709,0.1));
   fusion.AddAngSpeed(6226.923, PE::SBasicSensor(-3.60907668,0.1));
   fusion.AddSpeed   (6226.941, PE::SBasicSensor(19.62488,0.1));
   fusion.AddAngSpeed(6226.983, PE::SBasicSensor(-3.526818522,0.1));
   fusion.AddSpeed   (6226.991, PE::SBasicSensor(20.0709,0.1));
   fusion.AddSpeed   (6227.041, PE::SBasicSensor(20.0709,0.1));
   fusion.AddAngSpeed(6227.043, PE::SBasicSensor(-3.321173127,0.1));
   fusion.AddSpeed   (6227.092, PE::SBasicSensor(19.24007843,0.1));
   fusion.AddAngSpeed(6227.103, PE::SBasicSensor(-3.249197239,0.1));
   fusion.AddSpeed   (6227.141, PE::SBasicSensor(20.4805102,0.1));
   fusion.AddAngSpeed(6227.163, PE::SBasicSensor(-3.177221351,0.1));
   fusion.AddSpeed   (6227.191, PE::SBasicSensor(20.0709,0.1));
   fusion.AddAngSpeed(6227.223, PE::SBasicSensor(-3.053834114,0.1));
   fusion.AddSpeed   (6227.241, PE::SBasicSensor(19.62488,0.1));
   fusion.AddAngSpeed(6227.283, PE::SBasicSensor(-3.084680923,0.1));
   fusion.AddSpeed   (6227.292, PE::SBasicSensor(19.67735294,0.1));
   fusion.AddSpeed   (6227.342, PE::SBasicSensor(20.0709,0.1));
   fusion.AddAngSpeed(6227.343, PE::SBasicSensor(-2.981858226,0.1));

//    //#11 UTC=2016-12-16T09:27:55
//    fusion.AddPosition(6227.332, PE::SPosition(53.639508,10.007345,1.0));
//    fusion.AddHeading (6227.332, PE::SBasicSensor(127.099998,0.15));
   fusion.DoFusion();
   EXPECT_NEAR( 6227.343000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63950842, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00734669, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(126.82182062, fusion.GetHeading().Value, 0.00000001);
   EXPECT_NEAR( -3.00642859, fusion.GetAngSpeed().Value, 0.00000001);
   EXPECT_NEAR( 20.32352079, fusion.GetSpeed().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  1.23677904, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  0.30441393, fusion.GetHeading().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.10512642, fusion.GetAngSpeed().Accuracy, 0.00000001);
   EXPECT_NEAR(  0.10448272, fusion.GetSpeed().Accuracy, 0.00000001);
}

TEST_F(PECFusionSensorTest, test_same_position_with_same_time)
{
   PE::CFusionSensor fusion = PE::CFusionSensor(0.0,PE::SPosition(), PE::SBasicSensor(), PE::SBasicSensor(), PE::SBasicSensor());
   //same accuracy
   fusion.AddPosition(1000.00, PE::SPosition(53.00000000,10.00000000,2.0));
   fusion.AddPosition(1000.00, PE::SPosition(53.00000100,10.00000100,2.0));
   fusion.DoFusion();
   EXPECT_NEAR( 53.00000050, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000050, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(  2.00000000, fusion.GetPosition().HorizontalAcc, 0.00000001);

   //different accuracy
   fusion = PE::CFusionSensor(0.0,PE::SPosition(), PE::SBasicSensor(), PE::SBasicSensor(), PE::SBasicSensor());
   fusion.AddPosition(1000.00, PE::SPosition(53.00000000,10.00000000,1.0));
   fusion.AddPosition(1000.00, PE::SPosition(53.00000100,10.00000100,2.0));
   fusion.DoFusion();
   EXPECT_NEAR( 53.00000020, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000020, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(  1.20000000, fusion.GetPosition().HorizontalAcc, 0.00000001);
}

TEST_F(PECFusionSensorTest, test_same_heading_with_same_time)
{
   PE::CFusionSensor fusion = PE::CFusionSensor(0.0,PE::SPosition(50,10,1), PE::SBasicSensor(), PE::SBasicSensor(), PE::SBasicSensor());
   //same accuracy
   fusion.AddHeading(1000.00, PE::SBasicSensor(100.00000000,2.0));
   fusion.AddHeading(1000.00, PE::SBasicSensor(110.00000000,2.0));
   fusion.DoFusion();
   EXPECT_NEAR( 105.00000000, fusion.GetHeading().Value , 0.00000001);
   EXPECT_NEAR(   2.00000000, fusion.GetHeading().Accuracy, 0.00000001);

   //different accuracy
   fusion = PE::CFusionSensor(0.0,PE::SPosition(50,10,1), PE::SBasicSensor(), PE::SBasicSensor(), PE::SBasicSensor());
   fusion.AddHeading(1000.00, PE::SBasicSensor(100.00000000,1.0));
   fusion.AddHeading(1000.00, PE::SBasicSensor(110.00000000,2.0));
   fusion.DoFusion();
   EXPECT_NEAR( 102.00000000, fusion.GetHeading().Value , 0.00000001);
   EXPECT_NEAR(   1.20000000, fusion.GetHeading().Accuracy, 0.00000001);
}

TEST_F(PECFusionSensorTest, test_same_speed_with_same_time)
{
   PE::CFusionSensor fusion = PE::CFusionSensor(0.0,PE::SPosition(), PE::SBasicSensor(), PE::SBasicSensor(0,1), PE::SBasicSensor());
   //same accuracy
   fusion.AddSpeed(1000.00, PE::SBasicSensor(10.00000000,2.0));
   fusion.AddSpeed(1000.00, PE::SBasicSensor(11.00000000,2.0));
   fusion.DoFusion();
   EXPECT_NEAR( 10.50000000, fusion.GetSpeed().Value , 0.00000001);
   EXPECT_NEAR(   2.00000000, fusion.GetSpeed().Accuracy, 0.00000001);

   //different accuracy
   fusion = PE::CFusionSensor(0.0,PE::SPosition(), PE::SBasicSensor(), PE::SBasicSensor(0,1), PE::SBasicSensor());
   fusion.AddSpeed(1000.00, PE::SBasicSensor(10.00000000,1.0));
   fusion.AddSpeed(1000.00, PE::SBasicSensor(11.00000000,2.0));
   fusion.DoFusion();
   EXPECT_NEAR( 10.200000000, fusion.GetSpeed().Value , 0.00000001);
   EXPECT_NEAR(   1.20000000, fusion.GetSpeed().Accuracy, 0.00000001);
}

TEST_F(PECFusionSensorTest, test_same_angular_velocity_with_same_time)
{
   PE::CFusionSensor fusion = PE::CFusionSensor(0.0,PE::SPosition(), PE::SBasicSensor(), PE::SBasicSensor(), PE::SBasicSensor(0,1));
   //same accuracy
   fusion.AddAngSpeed(1000.00, PE::SBasicSensor(10.00000000,2.0));
   fusion.AddAngSpeed(1000.00, PE::SBasicSensor(11.00000000,2.0));
   fusion.DoFusion();
   EXPECT_NEAR( 10.50000000, fusion.GetAngSpeed().Value , 0.00000001);
   EXPECT_NEAR(   2.00000000, fusion.GetAngSpeed().Accuracy, 0.00000001);

   //different accuracy
   fusion = PE::CFusionSensor(0.0,PE::SPosition(), PE::SBasicSensor(), PE::SBasicSensor(), PE::SBasicSensor(0,1));
   fusion.AddAngSpeed(1000.00, PE::SBasicSensor(10.00000000,1.0));
   fusion.AddAngSpeed(1000.00, PE::SBasicSensor(11.00000000,2.0));
   fusion.DoFusion();
   EXPECT_NEAR( 10.200000000, fusion.GetAngSpeed().Value , 0.00000001);
   EXPECT_NEAR(   1.20000000, fusion.GetAngSpeed().Accuracy, 0.00000001);
}

TEST_F(PECFusionSensorTest, test_start_2_second_sensor_only_hamburg_airport_b433)
{
   //UTC: 2016-12-16T09:27:53
   PE::CFusionSensor fusion = PE::CFusionSensor(6225.332,PE::SPosition(53.639706, 10.006846, 1.0), PE::SBasicSensor(119.800003, 0.15), PE::SBasicSensor(), PE::SBasicSensor());
   fusion.AddSpeed   (6225.341, PE::SBasicSensor(20.4805102040816,1.0));
   fusion.AddAngSpeed(6225.363, PE::SBasicSensor(-3.44456036414687,1.0));
   fusion.AddSpeed   (6225.391, PE::SBasicSensor(19.62488,1.0));
   fusion.AddAngSpeed(6225.423, PE::SBasicSensor(-3.5062539826092,1.0));
   fusion.AddSpeed   (6225.442, PE::SBasicSensor(19.6773529411765,1.0));
   fusion.AddAngSpeed(6225.483, PE::SBasicSensor(-3.61935894979014,1.0));
   fusion.AddSpeed   (6225.491, PE::SBasicSensor(20.025387755102,1.0));
   fusion.AddSpeed   (6225.541, PE::SBasicSensor(20.0709,1.0));
   fusion.AddAngSpeed(6225.543, PE::SBasicSensor(-3.57822987081526,1.0));
   fusion.AddSpeed   (6225.591, PE::SBasicSensor(19.62488,1.0));
   fusion.AddAngSpeed(6225.603, PE::SBasicSensor(-3.58851214055898,1.0));
   fusion.AddSpeed   (6225.641, PE::SBasicSensor(20.0709,1.0));
   fusion.AddAngSpeed(6225.663, PE::SBasicSensor(-3.52681852209665,1.0));
   fusion.AddSpeed   (6225.691, PE::SBasicSensor(19.62488,1.0));
   fusion.AddAngSpeed(6225.723, PE::SBasicSensor(-3.55766533132781,1.0));
   fusion.AddSpeed   (6225.742, PE::SBasicSensor(19.6773529411765,1.0));
   fusion.AddAngSpeed(6225.783, PE::SBasicSensor(-3.6913348379962,1.0));
   fusion.AddSpeed   (6225.791, PE::SBasicSensor(20.4805102040816,1.0));
   fusion.AddSpeed   (6225.841, PE::SBasicSensor(20.0709,1.0));
   fusion.AddAngSpeed(6225.843, PE::SBasicSensor(-3.87641569338319,1.0));
   fusion.AddSpeed   (6225.891, PE::SBasicSensor(19.62488,1.0));
   fusion.AddAngSpeed(6225.903, PE::SBasicSensor(-3.99980293030786,1.0));
   fusion.AddSpeed   (6225.942, PE::SBasicSensor(19.6773529411765,1.0));
   fusion.AddAngSpeed(6225.963, PE::SBasicSensor(-4.15403697646369,1.0));
   fusion.AddSpeed   (6225.991, PE::SBasicSensor(20.4805102040816,1.0));
   fusion.AddAngSpeed(6226.023, PE::SBasicSensor(-4.22601286466974,1.0));
   fusion.AddSpeed   (6226.041, PE::SBasicSensor(20.0709,1.0));
   fusion.AddAngSpeed(6226.083, PE::SBasicSensor(-4.17460151595113,1.0));
   fusion.AddSpeed   (6226.092, PE::SBasicSensor(19.6773529411765,1.0));
   fusion.AddSpeed   (6226.141, PE::SBasicSensor(20.4805102040816,1.0));
   fusion.AddAngSpeed(6226.143, PE::SBasicSensor(-4.09234335800136,1.0));
   fusion.AddSpeed   (6226.191, PE::SBasicSensor(20.0709,1.0));
   fusion.AddAngSpeed(6226.203, PE::SBasicSensor(-4.06149654877019,1.0));
   fusion.AddSpeed   (6226.241, PE::SBasicSensor(19.62488,1.0));
   fusion.AddAngSpeed(6226.263, PE::SBasicSensor(-4.03064973953902,1.0));
   fusion.AddSpeed   (6226.292, PE::SBasicSensor(19.6773529411765,1.0));
   fusion.AddAngSpeed(6226.323, PE::SBasicSensor(-3.98952066056414,1.0));
   fusion.DoFusion();
   //UTC: 2016-12-16T09:27:54  53,639611   10,0071    123,199997    6226.332
   EXPECT_NEAR( 6226.323000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63961331, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00709996, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(123.56189252, fusion.GetHeading().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  2.23340316, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  1.20434883, fusion.GetHeading().Accuracy, 0.00000001);

   fusion.AddSpeed   (6226.341, PE::SBasicSensor(20.9356326530612,1.0));
   fusion.AddAngSpeed(6226.383, PE::SBasicSensor(-3.9278270421018,1.0));
   fusion.AddSpeed   (6226.391, PE::SBasicSensor(20.0709,1.0));
   fusion.AddSpeed   (6226.442, PE::SBasicSensor(19.6773529411765,1.0));
   fusion.AddAngSpeed(6226.443, PE::SBasicSensor(-4.01008520005158,1.0));
   fusion.AddSpeed   (6226.491, PE::SBasicSensor(20.4805102040816,1.0));
   fusion.AddAngSpeed(6226.503, PE::SBasicSensor(-3.83528661440831,1.0));
   fusion.AddSpeed   (6226.541, PE::SBasicSensor(20.0709,1.0));
   fusion.AddAngSpeed(6226.563, PE::SBasicSensor(-3.96895612107669,1.0));
   fusion.AddSpeed   (6226.591, PE::SBasicSensor(19.62488,1.0));
   fusion.AddAngSpeed(6226.623, PE::SBasicSensor(-3.89698023287064,1.0));
   fusion.AddSpeed   (6226.642, PE::SBasicSensor(19.6773529411765,1.0));
   fusion.AddAngSpeed(6226.683, PE::SBasicSensor(-3.79415753543342,1.0));
   fusion.AddSpeed   (6226.691, PE::SBasicSensor(20.4805102040816,1.0));
   fusion.AddSpeed   (6226.742, PE::SBasicSensor(19.6773529411765,1.0));
   fusion.AddAngSpeed(6226.743, PE::SBasicSensor(-3.75302845645853,1.0));
   fusion.AddSpeed   (6226.791, PE::SBasicSensor(20.4805102040816,1.0));
   fusion.AddAngSpeed(6226.803, PE::SBasicSensor(-3.73246391697109,1.0));
   fusion.AddSpeed   (6226.841, PE::SBasicSensor(20.0709,1.0));
   fusion.AddAngSpeed(6226.863, PE::SBasicSensor(-3.67077029850875,1.0));
   fusion.AddSpeed   (6226.891, PE::SBasicSensor(20.0709,1.0));
   fusion.AddAngSpeed(6226.923, PE::SBasicSensor(-3.60907668004642,1.0));
   fusion.AddSpeed   (6226.941, PE::SBasicSensor(19.62488,1.0));
   fusion.AddAngSpeed(6226.983, PE::SBasicSensor(-3.52681852209665,1.0));
   fusion.AddSpeed   (6226.991, PE::SBasicSensor(20.0709,1.0));
   fusion.AddSpeed   (6227.041, PE::SBasicSensor(20.0709,1.0));
   fusion.AddAngSpeed(6227.043, PE::SBasicSensor(-3.32117312722221,1.0));
   fusion.AddSpeed   (6227.092, PE::SBasicSensor(19.2400784313725,1.0));
   fusion.AddAngSpeed(6227.103, PE::SBasicSensor(-3.24919723901615,1.0));
   fusion.AddSpeed   (6227.141, PE::SBasicSensor(20.4805102040816,1.0));
   fusion.AddAngSpeed(6227.163, PE::SBasicSensor(-3.1772213508101,1.0));
   fusion.AddSpeed   (6227.191, PE::SBasicSensor(20.0709,1.0));
   fusion.AddAngSpeed(6227.223, PE::SBasicSensor(-3.05383411388543,1.0));
   fusion.AddSpeed   (6227.241, PE::SBasicSensor(19.62488,1.0));
   fusion.AddAngSpeed(6227.283, PE::SBasicSensor(-3.0846809231166,1.0));
   fusion.AddSpeed   (6227.292, PE::SBasicSensor(19.6773529411765,1.0));
   fusion.AddSpeed   (6227.342, PE::SBasicSensor(20.0709,1.0));
   fusion.AddAngSpeed(6227.343, PE::SBasicSensor(-2.98185822567938,1.0));
   fusion.DoFusion();
   //UTC: 2016-12-16T09:27:55  53,639508   10,007345   127,099998   6227.332
   EXPECT_NEAR( 6227.343000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63950681, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00735191, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(127.22295796, fusion.GetHeading().Value, 0.00000001);
   //Accuracy check
   EXPECT_NEAR(  3.36555648, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR(  2.30536556, fusion.GetHeading().Accuracy, 0.00000001);
}

TEST_F(PECFusionSensorTest, test_start_37_sec_dr_or_only_hamburg_airport_b433_rigth_then_left)
{
   //6245.332  2016-12-16T09:28:13  53,636716   10,009376  179,300003(0,15)
   PE::CFusionSensor fusion = PE::CFusionSensor(6245.332,PE::SPosition(53.636716, 10.009376, 1.0), PE::SBasicSensor(179.300003, 0.15), PE::SBasicSensor(), PE::SBasicSensor());
   PE::TValue bias = -0.24552;
   fusion.AddSpeed   (6245.341, PE::SBasicSensor(20.193730606878,1.0));
   fusion.AddAngSpeed(6245.344, PE::SBasicSensor(-0.524395756929822+0.24552,0.01));
   fusion.AddSpeed   (6245.391, PE::SBasicSensor(20.5976052190156,1.0));
   fusion.AddAngSpeed(6245.404, PE::SBasicSensor(-0.832863849241482+0.24552,0.01));
   fusion.AddSpeed   (6245.441, PE::SBasicSensor(21.0553297794381,1.0));
   fusion.AddAngSpeed(6245.464, PE::SBasicSensor(-1.14133194155314+0.24552,0.01));
   fusion.AddSpeed   (6245.490, PE::SBasicSensor(21.0179645091995,1.0));
   fusion.AddAngSpeed(6245.524, PE::SBasicSensor(-1.19274329027175+0.24552,0.01));
   fusion.AddSpeed   (6245.540, PE::SBasicSensor(21.0553297794381,1.0));
   fusion.AddAngSpeed(6245.584, PE::SBasicSensor(-1.18246102052803+0.24552,0.01));
   fusion.AddSpeed   (6245.591, PE::SBasicSensor(20.193730606878,1.0));
   fusion.AddSpeed   (6245.640, PE::SBasicSensor(21.0179645091995,1.0));
   fusion.AddAngSpeed(6245.644, PE::SBasicSensor(-1.04879151385964+0.24552,0.01));
   fusion.AddSpeed   (6245.690, PE::SBasicSensor(21.0553297794381,1.0));
   fusion.AddAngSpeed(6245.704, PE::SBasicSensor(-0.935686546678702+0.24552,0.01));
   fusion.AddSpeed   (6245.740, PE::SBasicSensor(20.5976052190156,1.0));
   fusion.AddAngSpeed(6245.764, PE::SBasicSensor(-1.01794470462848+0.24552,0.01));
   fusion.AddSpeed   (6245.791, PE::SBasicSensor(20.6424801759197,1.0));
   fusion.AddAngSpeed(6245.824, PE::SBasicSensor(-0.853428388728926+0.24552,0.01));
   fusion.AddSpeed   (6245.840, PE::SBasicSensor(21.0179645091995,1.0));
   fusion.AddAngSpeed(6245.884, PE::SBasicSensor(-0.87399292821637+0.24552,0.01));
   fusion.AddSpeed   (6245.890, PE::SBasicSensor(21.0553297794381,1.0));
   fusion.AddSpeed   (6245.940, PE::SBasicSensor(21.0553297794381,1.0));
   fusion.AddAngSpeed(6245.944, PE::SBasicSensor(-0.71975888206054+0.24552,0.01));
   fusion.AddSpeed   (6245.991, PE::SBasicSensor(20.193730606878,1.0));
   fusion.DoFusion();
//    EXPECT_NEAR( 6245.991000, fusion.GetTimestamp(), 0.00000001);
//    EXPECT_NEAR( 53.63659561, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00937737, fusion.GetPosition().Longitude, 0.00000001);
   fusion.AddAngSpeed(6246.004, PE::SBasicSensor(-0.699194342573096+0.24552,0.01));
   fusion.AddSpeed   (6246.040, PE::SBasicSensor(21.0179645091995,1.0));
   fusion.AddAngSpeed(6246.064, PE::SBasicSensor(-0.709476612316818+0.24552,0.01));
   fusion.AddSpeed   (6246.090, PE::SBasicSensor(21.0553297794381,1.0));
   fusion.AddAngSpeed(6246.124, PE::SBasicSensor(-0.71975888206054+0.24552,0.01));
   fusion.AddSpeed   (6246.140, PE::SBasicSensor(20.5976052190156,1.0));
   fusion.AddAngSpeed(6246.184, PE::SBasicSensor(-0.730041151804262+0.24552,0.01));
   fusion.AddSpeed   (6246.191, PE::SBasicSensor(20.6424801759197,1.0));
   fusion.AddSpeed   (6246.240, PE::SBasicSensor(21.0179645091995,1.0));
   fusion.AddAngSpeed(6246.244, PE::SBasicSensor(-0.832863849241482+0.24552,0.01));
   fusion.AddSpeed   (6246.290, PE::SBasicSensor(20.5976052190156,1.0));
   fusion.AddAngSpeed(6246.304, PE::SBasicSensor(-0.863710658472648+0.24552,0.01));
   fusion.AddSpeed   (6246.341, PE::SBasicSensor(20.193730606878,1.0));
   fusion.AddAngSpeed(6246.364, PE::SBasicSensor(-0.863710658472648+0.24552,0.01));
   fusion.AddSpeed   (6246.391, PE::SBasicSensor(21.0553297794381,1.0));
   fusion.AddAngSpeed(6246.424, PE::SBasicSensor(-0.915122007191258+0.24552,0.01));
   fusion.AddSpeed   (6246.441, PE::SBasicSensor(20.5976052190156,1.0));
   fusion.AddAngSpeed(6246.484, PE::SBasicSensor(-0.894557467703814+0.24552,0.01));
   fusion.AddSpeed   (6246.490, PE::SBasicSensor(21.0179645091995,1.0));
   fusion.AddSpeed   (6246.540, PE::SBasicSensor(20.5976052190156,1.0));
   fusion.AddAngSpeed(6246.544, PE::SBasicSensor(-0.802017040010316+0.24552,0.01));
   fusion.AddSpeed   (6246.591, PE::SBasicSensor(20.193730606878,1.0));
   fusion.AddAngSpeed(6246.604, PE::SBasicSensor(-0.791734770266594+0.24552,0.01));
   fusion.AddSpeed   (6246.640, PE::SBasicSensor(21.0179645091995,1.0));
   fusion.AddAngSpeed(6246.664, PE::SBasicSensor(-0.637500724110764+0.24552,0.01));
   fusion.AddSpeed   (6246.690, PE::SBasicSensor(20.5976052190156,1.0));
   fusion.AddAngSpeed(6246.724, PE::SBasicSensor(-0.71975888206054+0.24552,0.01));
   fusion.AddSpeed   (6246.740, PE::SBasicSensor(20.5976052190156,1.0));
   fusion.AddAngSpeed(6246.784, PE::SBasicSensor(-0.77117023077915+0.24552,0.01));
   fusion.AddSpeed   (6246.791, PE::SBasicSensor(20.193730606878,1.0));
   fusion.AddSpeed   (6246.840, PE::SBasicSensor(21.0179645091995,1.0));
   fusion.AddAngSpeed(6246.844, PE::SBasicSensor(-0.740323421547984+0.24552,0.01));
   fusion.AddSpeed   (6246.890, PE::SBasicSensor(20.139880658593,1.0));
   fusion.AddAngSpeed(6246.904, PE::SBasicSensor(-0.586089375392154+0.24552,0.01));
   fusion.AddSpeed   (6246.940, PE::SBasicSensor(20.5976052190156,1.0));
   fusion.AddAngSpeed(6246.964, PE::SBasicSensor(-0.71975888206054+0.24552,0.01));
   fusion.AddSpeed   (6246.991, PE::SBasicSensor(20.193730606878,1.0));
   fusion.DoFusion();
//    EXPECT_NEAR( 6246.991000, fusion.GetTimestamp(), 0.00000001);
//    EXPECT_NEAR( 53.63640882, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00937575, fusion.GetPosition().Longitude, 0.00000001);
   fusion.AddAngSpeed(6247.024, PE::SBasicSensor(-0.41129078974888+0.24552,0.01));
   fusion.AddSpeed   (6247.041, PE::SBasicSensor(20.5976052190156,1.0));
   fusion.AddAngSpeed(6247.084, PE::SBasicSensor(-0.349597171286548+0.24552,0.01));
   fusion.AddSpeed   (6247.090, PE::SBasicSensor(20.5508986312173,1.0));
   fusion.AddSpeed   (6247.140, PE::SBasicSensor(20.5976052190156,1.0));
   fusion.AddAngSpeed(6247.144, PE::SBasicSensor(-0.442137598980046+0.24552,0.01));
   fusion.AddSpeed   (6247.191, PE::SBasicSensor(19.7449810378363,1.0));
   fusion.AddAngSpeed(6247.204, PE::SBasicSensor(-0.431855329236324+0.24552,0.01));
   fusion.AddSpeed   (6247.240, PE::SBasicSensor(21.0179645091995,1.0));
   fusion.AddAngSpeed(6247.264, PE::SBasicSensor(-0.390726250261436+0.24552,0.01));
   fusion.AddSpeed   (6247.290, PE::SBasicSensor(20.139880658593,1.0));
   fusion.AddAngSpeed(6247.324, PE::SBasicSensor(-0.41129078974888+0.24552,0.01));
   fusion.AddSpeed   (6247.340, PE::SBasicSensor(20.5976052190156,1.0));
   fusion.AddAngSpeed(6247.384, PE::SBasicSensor(-0.380443980517714+0.24552,0.01));
   fusion.AddSpeed   (6247.391, PE::SBasicSensor(19.7449810378363,1.0));
   fusion.AddSpeed   (6247.441, PE::SBasicSensor(20.5976052190156,1.0));
   fusion.AddAngSpeed(6247.444, PE::SBasicSensor(-0.431855329236324+0.24552,0.01));
   fusion.AddSpeed   (6247.491, PE::SBasicSensor(20.139880658593,1.0));
   fusion.AddAngSpeed(6247.504, PE::SBasicSensor(-0.339314901542826+0.24552,0.01));
   fusion.AddSpeed   (6247.540, PE::SBasicSensor(20.5508986312173,1.0));
   fusion.AddAngSpeed(6247.564, PE::SBasicSensor(-0.20564539487444+0.24552,0.01));
   fusion.AddSpeed   (6247.591, PE::SBasicSensor(19.7449810378363,1.0));
   fusion.AddAngSpeed(6247.624, PE::SBasicSensor(-0.143951776412108+0.24552,0.01));
   fusion.AddSpeed   (6247.640, PE::SBasicSensor(20.5508986312173,1.0));
   fusion.AddAngSpeed(6247.684, PE::SBasicSensor(0.020564539487444+0.24552,0.01));
   fusion.AddSpeed   (6247.690, PE::SBasicSensor(20.139880658593,1.0));
   fusion.AddSpeed   (6247.740, PE::SBasicSensor(20.139880658593,1.0));
   fusion.AddAngSpeed(6247.744, PE::SBasicSensor(0.071975888206054+0.24552,0.01));
   fusion.AddSpeed   (6247.791, PE::SBasicSensor(19.7449810378363,1.0));
   fusion.AddAngSpeed(6247.804, PE::SBasicSensor(0.082258157949776+0.24552,0.01));
   fusion.AddSpeed   (6247.840, PE::SBasicSensor(20.5508986312173,1.0));
   fusion.AddAngSpeed(6247.864, PE::SBasicSensor(0.123387236924664+0.24552,0.01));
   fusion.AddSpeed   (6247.890, PE::SBasicSensor(20.139880658593,1.0));
   fusion.AddAngSpeed(6247.924, PE::SBasicSensor(0.195363125130718+0.24552,0.01));
   fusion.AddSpeed   (6247.940, PE::SBasicSensor(20.139880658593,1.0));
   fusion.AddAngSpeed(6247.984, PE::SBasicSensor(0.226209934361884+0.24552,0.01));
   fusion.AddSpeed   (6247.991, PE::SBasicSensor(19.2962314687945,1.0));
   fusion.DoFusion();
//    EXPECT_NEAR( 6247.991000, fusion.GetTimestamp(), 0.00000001);
//    EXPECT_NEAR( 53.63623004, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00937132, fusion.GetPosition().Longitude, 0.00000001);

   fusion.AddSpeed   (6248.040, PE::SBasicSensor(20.5508986312173,1.0));
   fusion.AddAngSpeed(6248.044, PE::SBasicSensor(0.174798585643274+0.24552,0.01));
   fusion.AddSpeed   (6248.090, PE::SBasicSensor(19.6821560981704,1.0));
   fusion.AddAngSpeed(6248.104, PE::SBasicSensor(0.20564539487444+0.24552,0.01));
   fusion.AddSpeed   (6248.140, PE::SBasicSensor(19.6821560981704,1.0));
   fusion.AddAngSpeed(6248.164, PE::SBasicSensor(0.113104967180942+0.24552,0.01));
   fusion.AddSpeed   (6248.191, PE::SBasicSensor(19.2962314687945,1.0));
   fusion.AddAngSpeed(6248.224, PE::SBasicSensor(0.174798585643274+0.24552,0.01));
   fusion.AddSpeed   (6248.240, PE::SBasicSensor(20.0838327532351,1.0));
   fusion.AddAngSpeed(6248.284, PE::SBasicSensor(0.113104967180942+0.24552,0.01));
   fusion.AddSpeed   (6248.290, PE::SBasicSensor(19.6821560981704,1.0));
   fusion.AddSpeed   (6248.340, PE::SBasicSensor(19.2244315377479,1.0));
   fusion.AddAngSpeed(6248.344, PE::SBasicSensor(0.143951776412108+0.24552,0.01));
   fusion.AddSpeed   (6248.391, PE::SBasicSensor(18.8474818997528,1.0));
   fusion.AddAngSpeed(6248.404, PE::SBasicSensor(0.15423404615583+0.24552,0.01));
   fusion.AddSpeed   (6248.441, PE::SBasicSensor(19.6821560981704,1.0));
   fusion.AddAngSpeed(6248.464, PE::SBasicSensor(0.082258157949776+0.24552,0.01));
   fusion.AddSpeed   (6248.490, PE::SBasicSensor(19.6167668752529,1.0));
   fusion.AddAngSpeed(6248.524, PE::SBasicSensor(0.15423404615583+0.24552,0.01));
   fusion.AddSpeed   (6248.541, PE::SBasicSensor(18.3987323307111,1.0));
   fusion.AddAngSpeed(6248.584, PE::SBasicSensor(0.10282269743722+0.24552,0.01));
   fusion.AddSpeed   (6248.591, PE::SBasicSensor(19.2244315377479,1.0));
   fusion.AddSpeed   (6248.640, PE::SBasicSensor(19.6167668752529,1.0));
   fusion.AddAngSpeed(6248.644, PE::SBasicSensor(0.082258157949776+0.24552,0.01));
   fusion.AddSpeed   (6248.690, PE::SBasicSensor(18.7667069773253,1.0));
   fusion.AddAngSpeed(6248.704, PE::SBasicSensor(0.020564539487444+0.24552,0.01));
   fusion.AddSpeed   (6248.740, PE::SBasicSensor(18.7667069773253,1.0));
   fusion.AddAngSpeed(6248.764, PE::SBasicSensor(-0.030846809231166+0.24552,0.01));
   fusion.AddSpeed   (6248.791, PE::SBasicSensor(18.3987323307111,1.0));
   fusion.AddAngSpeed(6248.824, PE::SBasicSensor(-0.020564539487444+0.24552,0.01));
   fusion.AddSpeed   (6248.841, PE::SBasicSensor(18.7667069773253,1.0));
   fusion.AddAngSpeed(6248.884, PE::SBasicSensor(0+0.24552,0.01));
   fusion.AddSpeed   (6248.890, PE::SBasicSensor(19.1497009972707,1.0));
   fusion.AddSpeed   (6248.940, PE::SBasicSensor(18.3089824169027,1.0));
   fusion.AddAngSpeed(6248.944, PE::SBasicSensor(-0.082258157949776+0.24552,0.01));
   fusion.AddSpeed   (6248.991, PE::SBasicSensor(18.3987323307111,1.0));
   fusion.AddAngSpeed(6249.004, PE::SBasicSensor(-0.185080855386996+0.24552,0.01));
   fusion.AddSpeed   (6249.041, PE::SBasicSensor(18.3089824169027,1.0));
   fusion.AddAngSpeed(6249.064, PE::SBasicSensor(-0.41129078974888+0.24552,0.01));
   fusion.AddSpeed   (6249.090, PE::SBasicSensor(18.6826351192885,1.0));
   fusion.AddAngSpeed(6249.124, PE::SBasicSensor(-0.524395756929822+0.24552,0.01));
   fusion.AddSpeed   (6249.141, PE::SBasicSensor(17.9499827616693,1.0));
   fusion.AddAngSpeed(6249.184, PE::SBasicSensor(-0.524395756929822+0.24552,0.01));
   fusion.AddSpeed   (6249.191, PE::SBasicSensor(17.8512578564802,1.0));
   fusion.AddSpeed   (6249.241, PE::SBasicSensor(18.3089824169027,1.0));
   fusion.AddAngSpeed(6249.244, PE::SBasicSensor(-0.750605691291706+0.24552,0.01));
   fusion.AddSpeed   (6249.290, PE::SBasicSensor(18.2155692413063,1.0));
   fusion.AddAngSpeed(6249.304, PE::SBasicSensor(-0.812299309754038+0.24552,0.01));
   fusion.AddSpeed   (6249.340, PE::SBasicSensor(17.8512578564802,1.0));
   fusion.AddAngSpeed(6249.364, PE::SBasicSensor(-1.14133194155314+0.24552,0.01));
   fusion.AddSpeed   (6249.391, PE::SBasicSensor(17.9499827616693,1.0));
   fusion.AddAngSpeed(6249.424, PE::SBasicSensor(-1.58346954053319+0.24552,0.01));
   fusion.AddSpeed   (6249.441, PE::SBasicSensor(17.8512578564802,1.0));
   fusion.AddAngSpeed(6249.484, PE::SBasicSensor(-1.92278444207601+0.24552,0.01));
   fusion.AddSpeed   (6249.490, PE::SBasicSensor(17.7485033633241,1.0));
   fusion.AddSpeed   (6249.540, PE::SBasicSensor(17.8512578564802,1.0));
   fusion.AddAngSpeed(6249.544, PE::SBasicSensor(-2.25181707387512+0.24552,0.01));
   fusion.AddSpeed   (6249.591, PE::SBasicSensor(17.0524836235859,1.0));
   fusion.AddAngSpeed(6249.604, PE::SBasicSensor(-2.50887381746817+0.24552,0.01));
   fusion.AddSpeed   (6249.641, PE::SBasicSensor(17.8512578564802,1.0));
   fusion.AddAngSpeed(6249.664, PE::SBasicSensor(-2.87903552824216+0.24552,0.01));
   fusion.AddSpeed   (6249.690, PE::SBasicSensor(17.7485033633241,1.0));
   fusion.AddAngSpeed(6249.724, PE::SBasicSensor(-3.36230220619709+0.24552,0.01));
   fusion.AddSpeed   (6249.740, PE::SBasicSensor(16.935808735635,1.0));
   fusion.AddAngSpeed(6249.784, PE::SBasicSensor(-4.01008520005158+0.24552,0.01));
   fusion.AddSpeed   (6249.791, PE::SBasicSensor(17.0524836235859,1.0));
   fusion.AddSpeed   (6249.841, PE::SBasicSensor(17.3935332960576,1.0));
   fusion.AddAngSpeed(6249.844, PE::SBasicSensor(-4.69899727288095+0.24552,0.01));
   fusion.AddSpeed   (6249.890, PE::SBasicSensor(17.2814374853419,1.0));
   fusion.AddAngSpeed(6249.904, PE::SBasicSensor(-5.19254622057961+0.24552,0.01));
   fusion.AddSpeed   (6249.940, PE::SBasicSensor(17.3935332960576,1.0));
   fusion.AddAngSpeed(6249.964, PE::SBasicSensor(-5.59355474058477+0.24552,0.01));
   fusion.AddSpeed   (6249.990, PE::SBasicSensor(16.935808735635,1.0));
   fusion.DoFusion();
//    EXPECT_NEAR( 6249.990000, fusion.GetTimestamp(), 0.00000001);
//    EXPECT_NEAR( 53.63589705, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00936012, fusion.GetPosition().Longitude, 0.00000001);
   fusion.AddAngSpeed(6250.024, PE::SBasicSensor(-5.96371645135876+0.24552,0.01));
   fusion.AddSpeed   (6250.040, PE::SBasicSensor(16.935808735635,1.0));
   fusion.AddAngSpeed(6250.084, PE::SBasicSensor(-6.45726539905742+0.24552,0.01));
   fusion.AddSpeed   (6250.090, PE::SBasicSensor(16.935808735635,1.0));
   fusion.AddSpeed   (6250.140, PE::SBasicSensor(16.935808735635,1.0));
   fusion.AddAngSpeed(6250.144, PE::SBasicSensor(-6.57037036623836+0.24552,0.01));
   fusion.AddSpeed   (6250.190, PE::SBasicSensor(16.935808735635,1.0));
   fusion.AddAngSpeed(6250.204, PE::SBasicSensor(-6.90968526778118+0.24552,0.01));
   fusion.AddSpeed   (6250.240, PE::SBasicSensor(16.4780841752124,1.0));
   fusion.AddAngSpeed(6250.264, PE::SBasicSensor(-6.7760157611128+0.24552,0.01));
   fusion.AddSpeed   (6250.290, PE::SBasicSensor(16.935808735635,1.0));
   fusion.AddAngSpeed(6250.324, PE::SBasicSensor(-6.9610966164998+0.24552,0.01));
   fusion.AddSpeed   (6250.340, PE::SBasicSensor(16.4780841752124,1.0));
   fusion.AddAngSpeed(6250.384, PE::SBasicSensor(-7.11533066265563+0.24552,0.01));
   fusion.AddSpeed   (6250.390, PE::SBasicSensor(16.935808735635,1.0));
   fusion.AddSpeed   (6250.441, PE::SBasicSensor(16.1549844855024,1.0));
   fusion.AddAngSpeed(6250.444, PE::SBasicSensor(-7.46492783394217+0.24552,0.01));
   fusion.AddSpeed   (6250.490, PE::SBasicSensor(16.8143716073596,1.0));
   fusion.AddAngSpeed(6250.504, PE::SBasicSensor(-7.87621862369105+0.24552,0.01));
   fusion.AddSpeed   (6250.540, PE::SBasicSensor(16.4780841752124,1.0));
   fusion.AddAngSpeed(6250.564, PE::SBasicSensor(-8.32863849241482+0.24552,0.01));
   fusion.AddSpeed   (6250.590, PE::SBasicSensor(16.4780841752124,1.0));
   fusion.AddAngSpeed(6250.624, PE::SBasicSensor(-8.6473888544702+0.24552,0.01));
   fusion.AddSpeed   (6250.640, PE::SBasicSensor(16.4780841752124,1.0));
   fusion.AddAngSpeed(6250.684, PE::SBasicSensor(-8.94557467703814+0.24552,0.01));
   fusion.AddSpeed   (6250.690, PE::SBasicSensor(16.4780841752124,1.0));
   fusion.AddSpeed   (6250.740, PE::SBasicSensor(16.4780841752124,1.0));
   fusion.AddAngSpeed(6250.744, PE::SBasicSensor(-9.04839737447536+0.24552,0.01));
   fusion.AddSpeed   (6250.791, PE::SBasicSensor(15.7062349164607,1.0));
   fusion.AddAngSpeed(6250.804, PE::SBasicSensor(-9.1615023416563+0.24552,0.01));
   fusion.AddSpeed   (6250.840, PE::SBasicSensor(16.8143716073596,1.0));
   fusion.AddAngSpeed(6250.864, PE::SBasicSensor(-9.09980872319397+0.24552,0.01));
   fusion.AddSpeed   (6250.890, PE::SBasicSensor(16.4780841752124,1.0));
   fusion.AddAngSpeed(6250.924, PE::SBasicSensor(-9.33630092729958+0.24552,0.01));
   fusion.AddSpeed   (6250.940, PE::SBasicSensor(16.0203596147899,1.0));
   fusion.AddAngSpeed(6250.984, PE::SBasicSensor(-9.35686546678702+0.24552,0.01));
   fusion.AddSpeed   (6250.991, PE::SBasicSensor(15.7062349164607,1.0));
   fusion.DoFusion();
//    EXPECT_NEAR( 6250.991000, fusion.GetTimestamp(), 0.00000001);
//    EXPECT_NEAR( 53.63574806, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00933038, fusion.GetPosition().Longitude, 0.00000001);
// 
   fusion.AddSpeed   (6251.040, PE::SBasicSensor(16.8143716073596,1.0));
   fusion.AddAngSpeed(6251.044, PE::SBasicSensor(-9.48025270371168+0.24552,0.01));
   fusion.AddSpeed   (6251.090, PE::SBasicSensor(16.0203596147899,1.0));
   fusion.AddAngSpeed(6251.104, PE::SBasicSensor(-9.70646263807357+0.24552,0.01));
   fusion.AddSpeed   (6251.140, PE::SBasicSensor(16.0203596147899,1.0));
   fusion.AddAngSpeed(6251.164, PE::SBasicSensor(-10.0149307303852+0.24552,0.01));
   fusion.AddSpeed   (6251.191, PE::SBasicSensor(16.1549844855024,1.0));
   fusion.AddAngSpeed(6251.224, PE::SBasicSensor(-10.1691647765411+0.24552,0.01));
   fusion.AddSpeed   (6251.240, PE::SBasicSensor(16.3473057293774,1.0));
   fusion.AddAngSpeed(6251.284, PE::SBasicSensor(-10.2925520134657+0.24552,0.01));
   fusion.AddSpeed   (6251.290, PE::SBasicSensor(15.5626350543673,1.0));
   fusion.AddSpeed   (6251.340, PE::SBasicSensor(16.4780841752124,1.0));
   fusion.AddAngSpeed(6251.344, PE::SBasicSensor(-10.3336810924406+0.24552,0.01));
   fusion.AddSpeed   (6251.391, PE::SBasicSensor(15.7062349164607,1.0));
   fusion.AddAngSpeed(6251.404, PE::SBasicSensor(-10.4159392503904+0.24552,0.01));
   fusion.AddSpeed   (6251.441, PE::SBasicSensor(15.5626350543673,1.0));
   fusion.AddAngSpeed(6251.464, PE::SBasicSensor(-10.5701732965462+0.24552,0.01));
   fusion.AddSpeed   (6251.490, PE::SBasicSensor(16.3473057293774,1.0));
   fusion.AddAngSpeed(6251.524, PE::SBasicSensor(-10.652431454496+0.24552,0.01));
   fusion.AddSpeed   (6251.540, PE::SBasicSensor(16.0203596147899,1.0));
   fusion.AddAngSpeed(6251.584, PE::SBasicSensor(-10.6729959939834+0.24552,0.01));
   fusion.AddSpeed   (6251.591, PE::SBasicSensor(15.2574853474189,1.0));
   fusion.AddSpeed   (6251.640, PE::SBasicSensor(16.3473057293774,1.0));
   fusion.AddAngSpeed(6251.644, PE::SBasicSensor(-10.6729959939834+0.24552,0.01));
   fusion.AddSpeed   (6251.690, PE::SBasicSensor(16.0203596147899,1.0));
   fusion.AddAngSpeed(6251.704, PE::SBasicSensor(-10.6421491847523+0.24552,0.01));
   fusion.AddSpeed   (6251.740, PE::SBasicSensor(15.5626350543673,1.0));
   fusion.AddAngSpeed(6251.764, PE::SBasicSensor(-10.6010201057774+0.24552,0.01));
   fusion.AddSpeed   (6251.791, PE::SBasicSensor(15.7062349164607,1.0));
   fusion.AddAngSpeed(6251.824, PE::SBasicSensor(-10.7038428032146+0.24552,0.01));
   fusion.AddSpeed   (6251.840, PE::SBasicSensor(15.8802398513952,1.0));
   fusion.AddAngSpeed(6251.884, PE::SBasicSensor(-10.6729959939834+0.24552,0.01));
   fusion.AddSpeed   (6251.890, PE::SBasicSensor(15.5626350543673,1.0));
   fusion.AddSpeed   (6251.940, PE::SBasicSensor(15.5626350543673,1.0));
   fusion.AddAngSpeed(6251.944, PE::SBasicSensor(-10.7963832309081+0.24552,0.01));
   fusion.AddSpeed   (6251.990, PE::SBasicSensor(15.5626350543673,1.0));
   fusion.DoFusion();
//    EXPECT_NEAR( 6251.990000, fusion.GetTimestamp(), 0.00000001);
//    EXPECT_NEAR( 53.63561255, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00926449, fusion.GetPosition().Longitude, 0.00000001);
   fusion.AddAngSpeed(6252.004, PE::SBasicSensor(-10.8889236586016+0.24552,0.01));
   fusion.AddSpeed   (6252.040, PE::SBasicSensor(15.5626350543673,1.0));
   fusion.AddAngSpeed(6252.064, PE::SBasicSensor(-10.8889236586016+0.24552,0.01));
   fusion.AddSpeed   (6252.090, PE::SBasicSensor(15.5626350543673,1.0));
   fusion.AddAngSpeed(6252.124, PE::SBasicSensor(-10.8889236586016+0.24552,0.01));
   fusion.AddSpeed   (6252.140, PE::SBasicSensor(15.5626350543673,1.0));
   fusion.AddAngSpeed(6252.184, PE::SBasicSensor(-11.0328754350137+0.24552,0.01));
   fusion.AddSpeed   (6252.191, PE::SBasicSensor(15.2574853474189,1.0));
   fusion.AddSpeed   (6252.240, PE::SBasicSensor(15.8802398513952,1.0));
   fusion.AddAngSpeed(6252.244, PE::SBasicSensor(-11.1151335929635+0.24552,0.01));
   fusion.AddSpeed   (6252.290, PE::SBasicSensor(15.1049104939447,1.0));
   fusion.AddAngSpeed(6252.304, PE::SBasicSensor(-11.1562626719384+0.24552,0.01));
   fusion.AddSpeed   (6252.340, PE::SBasicSensor(15.5626350543673,1.0));
   fusion.AddAngSpeed(6252.364, PE::SBasicSensor(-11.3104967180942+0.24552,0.01));
   fusion.AddSpeed   (6252.390, PE::SBasicSensor(15.5626350543673,1.0));
   fusion.AddAngSpeed(6252.424, PE::SBasicSensor(-11.5572711919435+0.24552,0.01));
   fusion.AddSpeed   (6252.441, PE::SBasicSensor(14.8087357783772,1.0));
   fusion.AddAngSpeed(6252.484, PE::SBasicSensor(-11.6806584288682+0.24552,0.01));
   fusion.AddSpeed   (6252.490, PE::SBasicSensor(15.8802398513952,1.0));
   fusion.AddSpeed   (6252.540, PE::SBasicSensor(15.1049104939447,1.0));
   fusion.AddAngSpeed(6252.544, PE::SBasicSensor(-11.8965860934864+0.24552,0.01));
   fusion.AddSpeed   (6252.591, PE::SBasicSensor(15.2574853474189,1.0));
   fusion.AddAngSpeed(6252.604, PE::SBasicSensor(-12.0508201396422+0.24552,0.01));
   fusion.AddSpeed   (6252.640, PE::SBasicSensor(15.413173973413,1.0));
   fusion.AddAngSpeed(6252.664, PE::SBasicSensor(-12.2564655345166+0.24552,0.01));
   fusion.AddSpeed   (6252.690, PE::SBasicSensor(15.1049104939447,1.0));
   fusion.AddAngSpeed(6252.725, PE::SBasicSensor(-12.3490059622101+0.24552,0.01));
   fusion.AddSpeed   (6252.740, PE::SBasicSensor(15.1049104939447,1.0));
   fusion.AddAngSpeed(6252.784, PE::SBasicSensor(-12.3284414227227+0.24552,0.01));
   fusion.AddSpeed   (6252.790, PE::SBasicSensor(15.1049104939447,1.0));
   fusion.AddSpeed   (6252.840, PE::SBasicSensor(15.1049104939447,1.0));
   fusion.AddAngSpeed(6252.844, PE::SBasicSensor(-12.3387236924664+0.24552,0.01));
   fusion.AddSpeed   (6252.890, PE::SBasicSensor(15.1049104939447,1.0));
   fusion.AddAngSpeed(6252.904, PE::SBasicSensor(-12.390135041185+0.24552,0.01));
   fusion.AddSpeed   (6252.940, PE::SBasicSensor(15.1049104939447,1.0));
   fusion.AddAngSpeed(6252.964, PE::SBasicSensor(-12.4004173109287+0.24552,0.01));
   fusion.AddSpeed   (6252.990, PE::SBasicSensor(15.1049104939447,1.0));
   fusion.AddAngSpeed(6253.024, PE::SBasicSensor(-12.1536428370794+0.24552,0.01));
   fusion.AddSpeed   (6253.040, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6253.084, PE::SBasicSensor(-11.8863038237426+0.24552,0.01));
   fusion.AddSpeed   (6253.090, PE::SBasicSensor(15.1049104939447,1.0));
   fusion.AddSpeed   (6253.140, PE::SBasicSensor(15.1049104939447,1.0));
   fusion.AddAngSpeed(6253.144, PE::SBasicSensor(-11.6600938893808+0.24552,0.01));
   fusion.AddSpeed   (6253.190, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6253.204, PE::SBasicSensor(-11.4236016852751+0.24552,0.01));
   fusion.AddSpeed   (6253.240, PE::SBasicSensor(15.1049104939447,1.0));
   fusion.AddAngSpeed(6253.264, PE::SBasicSensor(-11.4750130339938+0.24552,0.01));
   fusion.AddSpeed   (6253.290, PE::SBasicSensor(15.1049104939447,1.0));
   fusion.AddAngSpeed(6253.325, PE::SBasicSensor(-11.6086825406621+0.24552,0.01));
   fusion.AddSpeed   (6253.340, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6253.384, PE::SBasicSensor(-11.4955775734812+0.24552,0.01));
   fusion.AddSpeed   (6253.390, PE::SBasicSensor(15.1049104939447,1.0));
   fusion.AddSpeed   (6253.441, PE::SBasicSensor(14.3599862093355,1.0));
   fusion.AddAngSpeed(6253.444, PE::SBasicSensor(-11.3207789878379+0.24552,0.01));
   fusion.AddSpeed   (6253.490, PE::SBasicSensor(14.9461080954308,1.0));
   fusion.AddAngSpeed(6253.504, PE::SBasicSensor(-11.1254158627072+0.24552,0.01));
   fusion.AddSpeed   (6253.540, PE::SBasicSensor(15.1049104939447,1.0));
   fusion.AddAngSpeed(6253.564, PE::SBasicSensor(-10.7449718821895+0.24552,0.01));
   fusion.AddSpeed   (6253.590, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6253.624, PE::SBasicSensor(-10.1588825067973+0.24552,0.01));
   fusion.AddSpeed   (6253.640, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6253.684, PE::SBasicSensor(-9.61392221038007+0.24552,0.01));
   fusion.AddSpeed   (6253.690, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddSpeed   (6253.740, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6253.744, PE::SBasicSensor(-9.12037326268142+0.24552,0.01));
   fusion.AddSpeed   (6253.790, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6253.804, PE::SBasicSensor(-8.9250101375507+0.24552,0.01));
   fusion.AddSpeed   (6253.840, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6253.864, PE::SBasicSensor(-8.7399292821637+0.24552,0.01));
   fusion.AddSpeed   (6253.890, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6253.924, PE::SBasicSensor(-8.69880020318881+0.24552,0.01));
   fusion.AddSpeed   (6253.940, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6253.984, PE::SBasicSensor(-8.63710658472648+0.24552,0.01));
   fusion.AddSpeed   (6253.990, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.DoFusion();
//    EXPECT_NEAR( 6253.990000, fusion.GetTimestamp(), 0.00000001);
//    EXPECT_NEAR( 53.63538354, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00901594, fusion.GetPosition().Longitude, 0.00000001);

   fusion.AddSpeed   (6254.040, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6254.044, PE::SBasicSensor(-8.79134063088231+0.24552,0.01));
   fusion.AddSpeed   (6254.090, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6254.104, PE::SBasicSensor(-8.62682431498276+0.24552,0.01));
   fusion.AddSpeed   (6254.140, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6254.164, PE::SBasicSensor(-8.59597750575159+0.24552,0.01));
   fusion.AddSpeed   (6254.190, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6254.224, PE::SBasicSensor(-8.46230799908321+0.24552,0.01));
   fusion.AddSpeed   (6254.240, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6254.284, PE::SBasicSensor(-8.5034370780581+0.24552,0.01));
   fusion.AddSpeed   (6254.290, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddSpeed   (6254.340, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6254.344, PE::SBasicSensor(-8.53428388728926+0.24552,0.01));
   fusion.AddSpeed   (6254.390, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6254.405, PE::SBasicSensor(-8.72964701241998+0.24552,0.01));
   fusion.AddSpeed   (6254.440, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6254.464, PE::SBasicSensor(-8.66795339395765+0.24552,0.01));
   fusion.AddSpeed   (6254.490, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6254.525, PE::SBasicSensor(-8.72964701241998+0.24552,0.01));
   fusion.AddSpeed   (6254.540, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6254.585, PE::SBasicSensor(-8.8324697098572+0.24552,0.01));
   fusion.AddSpeed   (6254.590, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddSpeed   (6254.640, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6254.645, PE::SBasicSensor(-8.69880020318881+0.24552,0.01));
   fusion.AddSpeed   (6254.690, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6254.705, PE::SBasicSensor(-9.04839737447536+0.24552,0.01));
   fusion.AddSpeed   (6254.740, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6254.765, PE::SBasicSensor(-8.90444559806325+0.24552,0.01));
   fusion.AddSpeed   (6254.790, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6254.825, PE::SBasicSensor(-9.32601865755585+0.24552,0.01));
   fusion.AddSpeed   (6254.840, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6254.885, PE::SBasicSensor(-9.52138178268657+0.24552,0.01));
   fusion.AddSpeed   (6254.890, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddSpeed   (6254.940, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6254.945, PE::SBasicSensor(-9.93267257243545+0.24552,0.01));
   fusion.AddSpeed   (6254.990, PE::SBasicSensor(13.731736812677,1.0));
   fusion.DoFusion();
//    EXPECT_NEAR( 6254.990000, fusion.GetTimestamp(), 0.00000001);
//    EXPECT_NEAR( 53.63529764, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00885536, fusion.GetPosition().Longitude, 0.00000001);
   fusion.AddAngSpeed(6255.005, PE::SBasicSensor(-9.91210803294801+0.24552,0.01));
   fusion.AddSpeed   (6255.040, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6255.065, PE::SBasicSensor(-9.71674490781729+0.24552,0.01));
   fusion.AddSpeed   (6255.090, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6255.125, PE::SBasicSensor(-9.80928533551079+0.24552,0.01));
   fusion.AddSpeed   (6255.140, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6255.185, PE::SBasicSensor(-9.94295484217918+0.24552,0.01));
   fusion.AddSpeed   (6255.190, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddSpeed   (6255.240, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6255.245, PE::SBasicSensor(-10.3028342832094+0.24552,0.01));
   fusion.AddSpeed   (6255.290, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6255.305, PE::SBasicSensor(-10.6113023755211+0.24552,0.01));
   fusion.AddSpeed   (6255.340, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6255.365, PE::SBasicSensor(-10.7655364216769+0.24552,0.01));
   fusion.AddSpeed   (6255.390, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6255.425, PE::SBasicSensor(-10.724407342702+0.24552,0.01));
   fusion.AddSpeed   (6255.440, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6255.485, PE::SBasicSensor(-10.6318669150086+0.24552,0.01));
   fusion.AddSpeed   (6255.490, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddSpeed   (6255.540, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6255.545, PE::SBasicSensor(-10.7552541519332+0.24552,0.01));
   fusion.AddSpeed   (6255.590, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6255.605, PE::SBasicSensor(-10.9506172770639+0.24552,0.01));
   fusion.AddSpeed   (6255.640, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6255.665, PE::SBasicSensor(-11.207674020657+0.24552,0.01));
   fusion.AddSpeed   (6255.690, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6255.725, PE::SBasicSensor(-11.4030371457877+0.24552,0.01));
   fusion.AddSpeed   (6255.740, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6255.785, PE::SBasicSensor(-11.4955775734812+0.24552,0.01));
   fusion.AddSpeed   (6255.790, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddSpeed   (6255.840, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6255.845, PE::SBasicSensor(-11.5367066524561+0.24552,0.01));
   fusion.AddSpeed   (6255.890, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6255.905, PE::SBasicSensor(-11.7937633960491+0.24552,0.01));
   fusion.AddSpeed   (6255.940, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6255.965, PE::SBasicSensor(-11.6086825406621+0.24552,0.01));
   fusion.AddSpeed   (6255.990, PE::SBasicSensor(13.731736812677,1.0));
   fusion.DoFusion();
   EXPECT_NEAR( 6255.990000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63521543, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00872298, fusion.GetPosition().Longitude, 0.00000001);

   fusion.AddAngSpeed(6256.025, PE::SBasicSensor(-11.6189648104059+0.24552,0.01));
   fusion.AddSpeed   (6256.040, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6256.085, PE::SBasicSensor(-11.4236016852751+0.24552,0.01));
   fusion.AddSpeed   (6256.090, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddSpeed   (6256.140, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6256.145, PE::SBasicSensor(-11.4030371457877+0.24552,0.01));
   fusion.AddSpeed   (6256.190, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6256.205, PE::SBasicSensor(-11.2693676391193+0.24552,0.01));
   fusion.AddSpeed   (6256.240, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6256.265, PE::SBasicSensor(-11.094569053476+0.24552,0.01));
   fusion.AddSpeed   (6256.290, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6256.325, PE::SBasicSensor(-11.0328754350137+0.24552,0.01));
   fusion.AddSpeed   (6256.340, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6256.385, PE::SBasicSensor(-10.8169477703955+0.24552,0.01));
   fusion.AddSpeed   (6256.390, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddSpeed   (6256.441, PE::SBasicSensor(13.0137375022103,1.0));
   fusion.AddAngSpeed(6256.445, PE::SBasicSensor(-10.4981974083402+0.24552,0.01));
   fusion.AddSpeed   (6256.490, PE::SBasicSensor(13.5449104614842,1.0));
   fusion.AddAngSpeed(6256.505, PE::SBasicSensor(-10.3131165529532+0.24552,0.01));
   fusion.AddSpeed   (6256.540, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6256.565, PE::SBasicSensor(-9.88126122371684+0.24552,0.01));
   fusion.AddSpeed   (6256.590, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6256.625, PE::SBasicSensor(-9.73730944730474+0.24552,0.01));
   fusion.AddSpeed   (6256.640, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6256.685, PE::SBasicSensor(-9.69618036832985+0.24552,0.01));
   fusion.AddSpeed   (6256.690, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddSpeed   (6256.740, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6256.745, PE::SBasicSensor(-9.62420448012379+0.24552,0.01));
   fusion.AddSpeed   (6256.790, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6256.805, PE::SBasicSensor(-9.77843852627962+0.24552,0.01));
   fusion.AddSpeed   (6256.840, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6256.865, PE::SBasicSensor(-9.56251086166146+0.24552,0.01));
   fusion.AddSpeed   (6256.890, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6256.925, PE::SBasicSensor(-9.53166405243029+0.24552,0.01));
   fusion.AddSpeed   (6256.940, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6256.985, PE::SBasicSensor(-9.45968816422424+0.24552,0.01));
   fusion.AddSpeed   (6256.990, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddSpeed   (6257.040, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6257.045, PE::SBasicSensor(-9.57279313140518+0.24552,0.01));
   fusion.AddSpeed   (6257.090, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6257.105, PE::SBasicSensor(-9.48025270371168+0.24552,0.01));
   fusion.AddSpeed   (6257.140, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6257.165, PE::SBasicSensor(-9.54194632217402+0.24552,0.01));
   fusion.AddSpeed   (6257.190, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6257.225, PE::SBasicSensor(-9.52138178268657+0.24552,0.01));
   fusion.AddSpeed   (6257.240, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6257.285, PE::SBasicSensor(-9.48025270371168+0.24552,0.01));
   fusion.AddSpeed   (6257.290, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddSpeed   (6257.340, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6257.345, PE::SBasicSensor(-9.35686546678702+0.24552,0.01));
   fusion.AddSpeed   (6257.390, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6257.405, PE::SBasicSensor(-9.39799454576191+0.24552,0.01));
   fusion.AddSpeed   (6257.440, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6257.465, PE::SBasicSensor(-9.57279313140518+0.24552,0.01));
   fusion.AddSpeed   (6257.490, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6257.525, PE::SBasicSensor(-9.35686546678702+0.24552,0.01));
   fusion.AddSpeed   (6257.540, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6257.585, PE::SBasicSensor(-9.50081724319913+0.24552,0.01));
   fusion.AddSpeed   (6257.590, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddSpeed   (6257.640, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6257.645, PE::SBasicSensor(-9.30545411806841+0.24552,0.01));
   fusion.AddSpeed   (6257.690, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6257.705, PE::SBasicSensor(-9.32601865755585+0.24552,0.01));
   fusion.AddSpeed   (6257.740, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6257.765, PE::SBasicSensor(-9.22319596011863+0.24552,0.01));
   fusion.AddSpeed   (6257.790, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6257.825, PE::SBasicSensor(-9.05867964421908+0.24552,0.01));
   fusion.AddSpeed   (6257.840, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6257.885, PE::SBasicSensor(-8.78105836113859+0.24552,0.01));
   fusion.AddSpeed   (6257.890, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddSpeed   (6257.940, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6257.945, PE::SBasicSensor(-8.39033211087715+0.24552,0.01));
   fusion.AddSpeed   (6257.990, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6258.005, PE::SBasicSensor(-7.92762997240966+0.24552,0.01));
   fusion.AddSpeed   (6258.040, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6258.065, PE::SBasicSensor(-7.31069378778634+0.24552,0.01));
   fusion.AddSpeed   (6258.090, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6258.125, PE::SBasicSensor(-6.73488668213791+0.24552,0.01));
   fusion.AddSpeed   (6258.140, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6258.185, PE::SBasicSensor(-6.24133773443926+0.24552,0.01));
   fusion.AddSpeed   (6258.190, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddSpeed   (6258.240, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6258.245, PE::SBasicSensor(-5.76835332622804+0.24552,0.01));
   fusion.AddSpeed   (6258.290, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6258.305, PE::SBasicSensor(-5.26452210878566+0.24552,0.01));
   fusion.AddSpeed   (6258.340, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6258.365, PE::SBasicSensor(-4.82238450980562+0.24552,0.01));
   fusion.AddSpeed   (6258.390, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6258.425, PE::SBasicSensor(-4.10262562774508+0.24552,0.01));
   fusion.AddSpeed   (6258.440, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6258.485, PE::SBasicSensor(-3.04355184414171+0.24552,0.01));
   fusion.AddSpeed   (6258.490, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddSpeed   (6258.540, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6258.545, PE::SBasicSensor(-2.28266388310628+0.24552,0.01));
   fusion.AddSpeed   (6258.590, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6258.605, PE::SBasicSensor(-1.60403408002063+0.24552,0.01));
   fusion.AddSpeed   (6258.640, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6258.665, PE::SBasicSensor(-1.36754187591503+0.24552,0.01));
   fusion.AddSpeed   (6258.690, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6258.725, PE::SBasicSensor(-0.966533355909868+0.24552,0.01));
   fusion.AddSpeed   (6258.740, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6258.785, PE::SBasicSensor(-0.61693618462332+0.24552,0.01));
   fusion.AddSpeed   (6258.790, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddSpeed   (6258.840, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6258.845, PE::SBasicSensor(-0.277621283080494+0.24552,0.01));
   fusion.AddSpeed   (6258.890, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6258.905, PE::SBasicSensor(0.05141134871861+0.24552,0.01));
   fusion.AddSpeed   (6258.940, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6258.965, PE::SBasicSensor(0.030846809231166+0.24552,0.01));
   fusion.AddSpeed   (6258.990, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6259.025, PE::SBasicSensor(0.041129078974888+0.24552,0.01));
   fusion.AddSpeed   (6259.040, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6259.085, PE::SBasicSensor(0.30846809231166+0.24552,0.01));
   fusion.AddSpeed   (6259.090, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddSpeed   (6259.140, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6259.145, PE::SBasicSensor(0.544960296417266+0.24552,0.01));
   fusion.AddSpeed   (6259.190, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6259.205, PE::SBasicSensor(0.97681562565359+0.24552,0.01));
   fusion.AddSpeed   (6259.240, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6259.265, PE::SBasicSensor(1.14133194155314+0.24552,0.01));
   fusion.AddSpeed   (6259.290, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6259.325, PE::SBasicSensor(1.28528371796525+0.24552,0.01));
   fusion.AddSpeed   (6259.340, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6259.385, PE::SBasicSensor(1.30584825745269+0.24552,0.01));
   fusion.AddSpeed   (6259.390, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddSpeed   (6259.441, PE::SBasicSensor(13.462487071252,1.0));
   fusion.AddAngSpeed(6259.445, PE::SBasicSensor(1.29556598770897+0.24552,0.01));
   fusion.AddSpeed   (6259.490, PE::SBasicSensor(14.0119763394664,1.0));
   fusion.AddAngSpeed(6259.505, PE::SBasicSensor(1.37782414565875+0.24552,0.01));
   fusion.AddSpeed   (6259.540, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6259.565, PE::SBasicSensor(1.26471917847781+0.24552,0.01));
   fusion.AddSpeed   (6259.590, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6259.625, PE::SBasicSensor(1.23387236924664+0.24552,0.01));
   fusion.AddSpeed   (6259.640, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6259.685, PE::SBasicSensor(1.24415463899036+0.24552,0.01));
   fusion.AddSpeed   (6259.690, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddSpeed   (6259.740, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6259.745, PE::SBasicSensor(1.30584825745269+0.24552,0.01));
   fusion.AddSpeed   (6259.790, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6259.805, PE::SBasicSensor(1.29556598770897+0.24552,0.01));
   fusion.AddSpeed   (6259.840, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6259.865, PE::SBasicSensor(1.24415463899036+0.24552,0.01));
   fusion.AddSpeed   (6259.890, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6259.925, PE::SBasicSensor(1.26471917847781+0.24552,0.01));
   fusion.AddSpeed   (6259.940, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6259.985, PE::SBasicSensor(1.31613052719642+0.24552,0.01));
   fusion.AddSpeed   (6259.990, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddSpeed   (6260.040, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6260.045, PE::SBasicSensor(1.33669506668386+0.24552,0.01));
   fusion.AddSpeed   (6260.090, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6260.105, PE::SBasicSensor(1.5423404615583+0.24552,0.01));
   fusion.AddSpeed   (6260.140, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6260.165, PE::SBasicSensor(1.62459861950808+0.24552,0.01));
   fusion.AddSpeed   (6260.190, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6260.225, PE::SBasicSensor(1.7274213169453+0.24552,0.01));
   fusion.AddSpeed   (6260.240, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6260.285, PE::SBasicSensor(1.95363125130718+0.24552,0.01));
   fusion.AddSpeed   (6260.290, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddSpeed   (6260.340, PE::SBasicSensor(14.6471859335222,1.0));

   fusion.DoFusion();
   //6260.332  2016-12-16T09:28:28  53,6351     10,007929  264,600006(0,3)
   EXPECT_NEAR( 6260.340000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63509979, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00788434, fusion.GetPosition().Longitude, 0.00000001);
   //EXPECT_NEAR(260.99439273, fusion.GetHeading().Value, 0.00000001);
   //Accuracy check TO BAD ACCURACY estimate
   //EXPECT_NEAR(  0.00000000, fusion.GetPosition().HorizontalAcc, 0.00000001);
   //EXPECT_NEAR( 16.29013439, fusion.GetHeading().Accuracy, 0.00000001);
   fusion.AddAngSpeed(6260.345, PE::SBasicSensor(2.03588940925696+0.24552,0.01));
   fusion.AddSpeed   (6260.390, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6260.405, PE::SBasicSensor(1.95363125130718+0.24552,0.01));
   fusion.AddSpeed   (6260.441, PE::SBasicSensor(13.9112366402937,1.0));
   fusion.AddAngSpeed(6260.465, PE::SBasicSensor(1.91250217233229+0.24552,0.01));
   fusion.AddSpeed   (6260.490, PE::SBasicSensor(14.9461080954308,1.0));
   fusion.AddAngSpeed(6260.525, PE::SBasicSensor(1.95363125130718+0.24552,0.01));
   fusion.AddSpeed   (6260.540, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6260.585, PE::SBasicSensor(1.80967947489507+0.24552,0.01));
   fusion.AddSpeed   (6260.590, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddSpeed   (6260.640, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6260.645, PE::SBasicSensor(1.83024401438252+0.24552,0.01));
   fusion.AddSpeed   (6260.690, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6260.705, PE::SBasicSensor(1.6348808892518+0.24552,0.01));
   fusion.AddSpeed   (6260.740, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6260.765, PE::SBasicSensor(1.73770358668902+0.24552,0.01));
   fusion.AddSpeed   (6260.790, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6260.825, PE::SBasicSensor(1.7274213169453+0.24552,0.01));
   fusion.AddSpeed   (6260.840, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6260.885, PE::SBasicSensor(1.7274213169453+0.24552,0.01));
   fusion.AddSpeed   (6260.890, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddSpeed   (6260.940, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6260.945, PE::SBasicSensor(1.80967947489507+0.24552,0.01));
   fusion.AddSpeed   (6260.990, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6261.005, PE::SBasicSensor(1.92278444207601+0.24552,0.01));
   fusion.AddSpeed   (6261.040, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6261.065, PE::SBasicSensor(2.06673621848812+0.24552,0.01));
   fusion.AddSpeed   (6261.090, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6261.125, PE::SBasicSensor(2.15927664618162+0.24552,0.01));
   fusion.AddSpeed   (6261.140, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6261.185, PE::SBasicSensor(2.29294615285001+0.24552,0.01));
   fusion.AddSpeed   (6261.190, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddSpeed   (6261.240, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6261.245, PE::SBasicSensor(2.478027008237+0.24552,0.01));
   fusion.AddSpeed   (6261.290, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6261.305, PE::SBasicSensor(2.5705674359305+0.24552,0.01));
   fusion.AddSpeed   (6261.340, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6261.365, PE::SBasicSensor(2.65282559388028+0.24552,0.01));
   fusion.AddSpeed   (6261.390, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6261.425, PE::SBasicSensor(2.49859154772445+0.24552,0.01));
   fusion.AddSpeed   (6261.440, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6261.485, PE::SBasicSensor(2.59113197541794+0.24552,0.01));
   fusion.AddSpeed   (6261.490, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddSpeed   (6261.540, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6261.545, PE::SBasicSensor(2.46774473849328+0.24552,0.01));
   fusion.AddSpeed   (6261.590, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6261.605, PE::SBasicSensor(2.53972062669933+0.24552,0.01));
   fusion.AddSpeed   (6261.640, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6261.665, PE::SBasicSensor(2.60141424516167+0.24552,0.01));
   fusion.AddSpeed   (6261.690, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6261.725, PE::SBasicSensor(2.72480148208633+0.24552,0.01));
   fusion.AddSpeed   (6261.740, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6261.785, PE::SBasicSensor(2.81734190977983+0.24552,0.01));
   fusion.AddSpeed   (6261.790, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddSpeed   (6261.840, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6261.845, PE::SBasicSensor(3.05383411388543+0.24552,0.01));
   fusion.AddSpeed   (6261.890, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6261.905, PE::SBasicSensor(3.39314901542826+0.24552,0.01));
   fusion.AddSpeed   (6261.940, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6261.965, PE::SBasicSensor(3.75302845645853+0.24552,0.01));
   fusion.AddSpeed   (6261.990, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.DoFusion();
//    EXPECT_NEAR( 6261.990000, fusion.GetTimestamp(), 0.00000001);
//    EXPECT_NEAR( 53.63511530, fusion.GetPosition().Latitude , 0.00000001);
//    EXPECT_NEAR( 10.00746291, fusion.GetPosition().Longitude, 0.00000001);
   fusion.AddAngSpeed(6262.025, PE::SBasicSensor(4.13347243697624+0.24552,0.01));
   fusion.AddSpeed   (6262.040, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6262.085, PE::SBasicSensor(4.19516605543858+0.24552,0.01));
   fusion.AddSpeed   (6262.090, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddSpeed   (6262.140, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6262.145, PE::SBasicSensor(4.21573059492602+0.24552,0.01));
   fusion.AddSpeed   (6262.190, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6262.205, PE::SBasicSensor(4.34940010159441+0.24552,0.01));
   fusion.AddSpeed   (6262.240, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6262.265, PE::SBasicSensor(4.30827102261952+0.24552,0.01));
   fusion.AddSpeed   (6262.290, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6262.325, PE::SBasicSensor(4.39052918056929+0.24552,0.01));
   fusion.AddSpeed   (6262.340, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6262.385, PE::SBasicSensor(4.40081145031302+0.24552,0.01));
   fusion.AddSpeed   (6262.390, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddSpeed   (6262.440, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6262.445, PE::SBasicSensor(4.50363414775024+0.24552,0.01));
   fusion.AddSpeed   (6262.490, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6262.505, PE::SBasicSensor(4.77097316108701+0.24552,0.01));
   fusion.AddSpeed   (6262.540, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6262.565, PE::SBasicSensor(5.0485944441675+0.24552,0.01));
   fusion.AddSpeed   (6262.590, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6262.625, PE::SBasicSensor(5.22339302981078+0.24552,0.01));
   fusion.AddSpeed   (6262.640, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6262.685, PE::SBasicSensor(5.27480437852939+0.24552,0.01));
   fusion.AddSpeed   (6262.690, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddSpeed   (6262.740, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6262.745, PE::SBasicSensor(5.17198168109217+0.24552,0.01));
   fusion.AddSpeed   (6262.790, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6262.805, PE::SBasicSensor(5.28508664827311+0.24552,0.01));
   fusion.AddSpeed   (6262.840, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6262.865, PE::SBasicSensor(5.12057033237356+0.24552,0.01));
   fusion.AddSpeed   (6262.890, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6262.925, PE::SBasicSensor(5.52157885237871+0.24552,0.01));
   fusion.AddSpeed   (6262.940, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6262.985, PE::SBasicSensor(5.72722424725315+0.24552,0.01));
   fusion.AddSpeed   (6262.990, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddSpeed   (6263.040, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6263.045, PE::SBasicSensor(6.11795049751459+0.24552,0.01));
   fusion.AddSpeed   (6263.090, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6263.105, PE::SBasicSensor(6.38528951085136+0.24552,0.01));
   fusion.AddSpeed   (6263.140, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6263.165, PE::SBasicSensor(6.58065263598208+0.24552,0.01));
   fusion.AddSpeed   (6263.190, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6263.225, PE::SBasicSensor(6.69375760316302+0.24552,0.01));
   fusion.AddSpeed   (6263.240, PE::SBasicSensor(14.6471859335222,1.0));
   fusion.AddAngSpeed(6263.285, PE::SBasicSensor(6.63206398470069+0.24552,0.01));
   fusion.AddSpeed   (6263.290, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddSpeed   (6263.340, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6263.345, PE::SBasicSensor(6.70403987290674+0.24552,0.01));
   fusion.AddSpeed   (6263.390, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6263.405, PE::SBasicSensor(6.87883845855002+0.24552,0.01));
   fusion.AddSpeed   (6263.440, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6263.465, PE::SBasicSensor(7.05363704419329+0.24552,0.01));
   fusion.AddSpeed   (6263.490, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6263.525, PE::SBasicSensor(7.22843562983657+0.24552,0.01));
   fusion.AddSpeed   (6263.540, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6263.585, PE::SBasicSensor(7.41351648522356+0.24552,0.01));
   fusion.AddSpeed   (6263.590, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddSpeed   (6263.640, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6263.645, PE::SBasicSensor(7.57803280112312+0.24552,0.01));
   fusion.AddSpeed   (6263.690, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6263.705, PE::SBasicSensor(7.43408102471101+0.24552,0.01));
   fusion.AddSpeed   (6263.740, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6263.765, PE::SBasicSensor(7.51633918266078+0.24552,0.01));
   fusion.AddSpeed   (6263.790, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6263.825, PE::SBasicSensor(7.50605691291706+0.24552,0.01));
   fusion.AddSpeed   (6263.840, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6263.885, PE::SBasicSensor(7.53690372214823+0.24552,0.01));
   fusion.AddSpeed   (6263.890, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddSpeed   (6263.940, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6263.945, PE::SBasicSensor(7.34154059701751+0.24552,0.01));
   fusion.AddSpeed   (6263.990, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.DoFusion();
   EXPECT_NEAR( 6263.990000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63497638, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00711243, fusion.GetPosition().Longitude, 0.00000001);

   fusion.AddAngSpeed(6264.005, PE::SBasicSensor(7.25928243906773+0.24552,0.01));
   fusion.AddSpeed   (6264.040, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6264.065, PE::SBasicSensor(7.1975888206054+0.24552,0.01));
   fusion.AddSpeed   (6264.090, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6264.125, PE::SBasicSensor(7.20787109034912+0.24552,0.01));
   fusion.AddSpeed   (6264.140, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6264.185, PE::SBasicSensor(7.23871789958029+0.24552,0.01));
   fusion.AddSpeed   (6264.190, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddSpeed   (6264.240, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6264.245, PE::SBasicSensor(7.26956470881146+0.24552,0.01));
   fusion.AddSpeed   (6264.290, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6264.305, PE::SBasicSensor(7.24900016932401+0.24552,0.01));
   fusion.AddSpeed   (6264.340, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6264.365, PE::SBasicSensor(7.34154059701751+0.24552,0.01));
   fusion.AddSpeed   (6264.390, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6264.425, PE::SBasicSensor(7.39295194573612+0.24552,0.01));
   fusion.AddSpeed   (6264.440, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6264.485, PE::SBasicSensor(7.58831507086684+0.24552,0.01));
   fusion.AddSpeed   (6264.490, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddSpeed   (6264.540, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6264.545, PE::SBasicSensor(7.77339592625383+0.24552,0.01));
   fusion.AddSpeed   (6264.590, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6264.605, PE::SBasicSensor(7.85565408420361+0.24552,0.01));
   fusion.AddSpeed   (6264.640, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6264.665, PE::SBasicSensor(7.8967831631785+0.24552,0.01));
   fusion.AddSpeed   (6264.690, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6264.725, PE::SBasicSensor(7.85565408420361+0.24552,0.01));
   fusion.AddSpeed   (6264.740, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6264.785, PE::SBasicSensor(7.73226684727895+0.24552,0.01));
   fusion.AddSpeed   (6264.790, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddSpeed   (6264.840, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6264.845, PE::SBasicSensor(7.72198457753522+0.24552,0.01));
   fusion.AddSpeed   (6264.890, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6264.905, PE::SBasicSensor(7.77339592625383+0.24552,0.01));
   fusion.AddSpeed   (6264.940, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6264.965, PE::SBasicSensor(7.78367819599755+0.24552,0.01));
   fusion.AddSpeed   (6264.989, PE::SBasicSensor(14.0119763394664,1.0));
   fusion.AddAngSpeed(6265.025, PE::SBasicSensor(8.00988813035944+0.24552,0.01));
   fusion.AddSpeed   (6265.040, PE::SBasicSensor(13.9112366402937,1.0));
   fusion.AddAngSpeed(6265.085, PE::SBasicSensor(8.16412217651527+0.24552,0.01));
   fusion.AddSpeed   (6265.090, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddSpeed   (6265.140, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6265.145, PE::SBasicSensor(8.1332753672841+0.24552,0.01));
   fusion.AddSpeed   (6265.190, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6265.205, PE::SBasicSensor(8.18468671600271+0.24552,0.01));
   fusion.AddSpeed   (6265.240, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6265.265, PE::SBasicSensor(8.09214628830921+0.24552,0.01));
   fusion.AddSpeed   (6265.290, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6265.325, PE::SBasicSensor(8.20525125549016+0.24552,0.01));
   fusion.AddSpeed   (6265.340, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6265.385, PE::SBasicSensor(8.30807395292738+0.24552,0.01));
   fusion.AddSpeed   (6265.390, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddSpeed   (6265.440, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6265.445, PE::SBasicSensor(8.60625977549531+0.24552,0.01));
   fusion.AddSpeed   (6265.490, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6265.505, PE::SBasicSensor(8.76049382165114+0.24552,0.01));
   fusion.AddSpeed   (6265.540, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6265.565, PE::SBasicSensor(8.85303424934464+0.24552,0.01));
   fusion.AddSpeed   (6265.589, PE::SBasicSensor(14.0119763394664,1.0));
   fusion.AddAngSpeed(6265.625, PE::SBasicSensor(8.84275197960092+0.24552,0.01));
   fusion.AddSpeed   (6265.640, PE::SBasicSensor(13.9112366402937,1.0));
   fusion.AddAngSpeed(6265.685, PE::SBasicSensor(8.71936474267626+0.24552,0.01));
   fusion.AddSpeed   (6265.689, PE::SBasicSensor(13.5449104614842,1.0));
   fusion.AddSpeed   (6265.740, PE::SBasicSensor(13.462487071252,1.0));
   fusion.AddAngSpeed(6265.745, PE::SBasicSensor(8.69880020318881+0.24552,0.01));
   fusion.AddSpeed   (6265.790, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6265.805, PE::SBasicSensor(8.71936474267626+0.24552,0.01));
   fusion.AddSpeed   (6265.840, PE::SBasicSensor(14.1894613730996,1.0));
   fusion.AddAngSpeed(6265.865, PE::SBasicSensor(8.75021155190742+0.24552,0.01));
   fusion.AddSpeed   (6265.889, PE::SBasicSensor(13.5449104614842,1.0));
   fusion.AddAngSpeed(6265.925, PE::SBasicSensor(8.80162290062603+0.24552,0.01));
   fusion.AddSpeed   (6265.939, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6265.985, PE::SBasicSensor(8.87359878883209+0.24552,0.01));
   fusion.AddSpeed   (6265.989, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddSpeed   (6266.039, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6266.045, PE::SBasicSensor(8.66795339395765+0.24552,0.01));
   fusion.AddSpeed   (6266.089, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6266.105, PE::SBasicSensor(8.33892076215854+0.24552,0.01));
   fusion.AddSpeed   (6266.140, PE::SBasicSensor(13.0137375022103,1.0));
   fusion.AddAngSpeed(6266.165, PE::SBasicSensor(8.12299309754038+0.24552,0.01));
   fusion.AddSpeed   (6266.189, PE::SBasicSensor(14.0119763394664,1.0));
   fusion.AddAngSpeed(6266.225, PE::SBasicSensor(7.72198457753522+0.24552,0.01));
   fusion.AddSpeed   (6266.240, PE::SBasicSensor(13.462487071252,1.0));
   fusion.AddAngSpeed(6266.285, PE::SBasicSensor(7.58831507086684+0.24552,0.01));
   fusion.AddSpeed   (6266.289, PE::SBasicSensor(13.5449104614842,1.0));
   fusion.AddSpeed   (6266.340, PE::SBasicSensor(13.462487071252,1.0));
   fusion.AddAngSpeed(6266.345, PE::SBasicSensor(7.46492783394217+0.24552,0.01));
   fusion.AddSpeed   (6266.389, PE::SBasicSensor(13.5449104614842,1.0));
   fusion.AddAngSpeed(6266.405, PE::SBasicSensor(7.36210513650495+0.24552,0.01));
   fusion.AddSpeed   (6266.440, PE::SBasicSensor(13.462487071252,1.0));
   fusion.AddAngSpeed(6266.465, PE::SBasicSensor(7.49577464317334+0.24552,0.01));
   fusion.AddSpeed   (6266.489, PE::SBasicSensor(13.5449104614842,1.0));
   fusion.AddAngSpeed(6266.525, PE::SBasicSensor(7.7117023077915+0.24552,0.01));
   fusion.AddSpeed   (6266.540, PE::SBasicSensor(13.462487071252,1.0));
   fusion.AddAngSpeed(6266.585, PE::SBasicSensor(8.17440444625899+0.24552,0.01));
   fusion.AddSpeed   (6266.589, PE::SBasicSensor(13.5449104614842,1.0));
   fusion.AddSpeed   (6266.639, PE::SBasicSensor(13.731736812677,1.0));
   fusion.AddAngSpeed(6266.645, PE::SBasicSensor(8.18468671600271+0.24552,0.01));
   fusion.AddSpeed   (6266.689, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6266.705, PE::SBasicSensor(8.03045266984688+0.24552,0.01));
   fusion.AddSpeed   (6266.740, PE::SBasicSensor(13.0137375022103,1.0));
   fusion.AddAngSpeed(6266.765, PE::SBasicSensor(8.02017040010316+0.24552,0.01));
   fusion.AddSpeed   (6266.789, PE::SBasicSensor(13.5449104614842,1.0));
   fusion.AddAngSpeed(6266.825, PE::SBasicSensor(7.84537181445989+0.24552,0.01));
   fusion.AddSpeed   (6266.840, PE::SBasicSensor(13.462487071252,1.0));
   fusion.AddAngSpeed(6266.885, PE::SBasicSensor(7.83508954471616+0.24552,0.01));
   fusion.AddSpeed   (6266.889, PE::SBasicSensor(13.5449104614842,1.0));
   fusion.AddSpeed   (6266.939, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6266.945, PE::SBasicSensor(7.94819451189711+0.24552,0.01));
   fusion.AddSpeed   (6266.989, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6267.005, PE::SBasicSensor(7.94819451189711+0.24552,0.01));
   fusion.AddSpeed   (6267.039, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6267.065, PE::SBasicSensor(8.05101720933433+0.24552,0.01));
   fusion.AddSpeed   (6267.089, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6267.125, PE::SBasicSensor(8.03045266984688+0.24552,0.01));
   fusion.AddSpeed   (6267.139, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6267.185, PE::SBasicSensor(7.86593635394733+0.24552,0.01));
   fusion.AddSpeed   (6267.189, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddSpeed   (6267.239, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6267.245, PE::SBasicSensor(7.69113776830406+0.24552,0.01));
   fusion.AddSpeed   (6267.289, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6267.305, PE::SBasicSensor(7.51633918266078+0.24552,0.01));
   fusion.AddSpeed   (6267.339, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6267.365, PE::SBasicSensor(7.45464556419845+0.24552,0.01));
   fusion.AddSpeed   (6267.389, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6267.425, PE::SBasicSensor(7.47521010368589+0.24552,0.01));
   fusion.AddSpeed   (6267.440, PE::SBasicSensor(13.0137375022103,1.0));
   fusion.AddAngSpeed(6267.485, PE::SBasicSensor(7.51633918266078+0.24552,0.01));
   fusion.AddSpeed   (6267.489, PE::SBasicSensor(13.5449104614842,1.0));
   fusion.AddSpeed   (6267.539, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6267.545, PE::SBasicSensor(7.39295194573612+0.24552,0.01));
   fusion.AddSpeed   (6267.589, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6267.605, PE::SBasicSensor(7.12561293239935+0.24552,0.01));
   fusion.AddSpeed   (6267.639, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6267.665, PE::SBasicSensor(6.95081434675607+0.24552,0.01));
   fusion.AddSpeed   (6267.689, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6267.725, PE::SBasicSensor(6.83770937957513+0.24552,0.01));
   fusion.AddSpeed   (6267.739, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6267.785, PE::SBasicSensor(6.89940299803746+0.24552,0.01));
   fusion.AddSpeed   (6267.789, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddSpeed   (6267.839, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6267.845, PE::SBasicSensor(6.9610966164998+0.24552,0.01));
   fusion.AddSpeed   (6267.889, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6267.905, PE::SBasicSensor(7.09476612316818+0.24552,0.01));
   fusion.AddSpeed   (6267.939, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6267.965, PE::SBasicSensor(7.13589520214307+0.24552,0.01));
   fusion.AddSpeed   (6267.989, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6268.025, PE::SBasicSensor(7.27984697855518+0.24552,0.01));
   fusion.AddSpeed   (6268.039, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6268.085, PE::SBasicSensor(7.36210513650495+0.24552,0.01));
   fusion.AddSpeed   (6268.089, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddSpeed   (6268.139, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6268.145, PE::SBasicSensor(7.36210513650495+0.24552,0.01));
   fusion.AddSpeed   (6268.189, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6268.205, PE::SBasicSensor(7.44436329445473+0.24552,0.01));
   fusion.AddSpeed   (6268.239, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6268.265, PE::SBasicSensor(7.35182286676123+0.24552,0.01));
   fusion.AddSpeed   (6268.289, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6268.325, PE::SBasicSensor(7.30041151804262+0.24552,0.01));
   fusion.AddSpeed   (6268.339, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6268.385, PE::SBasicSensor(7.34154059701751+0.24552,0.01));
   fusion.AddSpeed   (6268.390, PE::SBasicSensor(13.0137375022103,1.0));
   fusion.AddSpeed   (6268.440, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6268.445, PE::SBasicSensor(7.18730655086168+0.24552,0.01));
   fusion.AddSpeed   (6268.489, PE::SBasicSensor(13.0778445835019,1.0));
   fusion.AddAngSpeed(6268.505, PE::SBasicSensor(7.0125079652184+0.24552,0.01));
   fusion.AddSpeed   (6268.539, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6268.565, PE::SBasicSensor(6.74516895188163+0.24552,0.01));
   fusion.AddSpeed   (6268.589, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6268.625, PE::SBasicSensor(6.53952355700719+0.24552,0.01));
   fusion.AddSpeed   (6268.639, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6268.685, PE::SBasicSensor(6.32359589238903+0.24552,0.01));
   fusion.AddSpeed   (6268.689, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddSpeed   (6268.739, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6268.745, PE::SBasicSensor(6.43670085956997+0.24552,0.01));
   fusion.AddSpeed   (6268.789, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6268.805, PE::SBasicSensor(6.42641858982625+0.24552,0.01));
   fusion.AddSpeed   (6268.839, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6268.865, PE::SBasicSensor(6.62178171495697+0.24552,0.01));
   fusion.AddSpeed   (6268.889, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6268.925, PE::SBasicSensor(6.56008809649464+0.24552,0.01));
   fusion.AddSpeed   (6268.939, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6268.985, PE::SBasicSensor(6.61149944521325+0.24552,0.01));
   fusion.AddSpeed   (6268.989, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddSpeed   (6269.039, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6269.045, PE::SBasicSensor(6.54980582675092+0.24552,0.01));
   fusion.AddSpeed   (6269.089, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6269.105, PE::SBasicSensor(6.6834753334193+0.24552,0.01));
   fusion.AddSpeed   (6269.139, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6269.165, PE::SBasicSensor(6.74516895188163+0.24552,0.01));
   fusion.AddSpeed   (6269.189, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6269.225, PE::SBasicSensor(6.7760157611128+0.24552,0.01));
   fusion.AddSpeed   (6269.239, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6269.285, PE::SBasicSensor(6.70403987290674+0.24552,0.01));
   fusion.AddSpeed   (6269.289, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddSpeed   (6269.339, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6269.345, PE::SBasicSensor(6.78629803085652+0.24552,0.01));
   fusion.AddSpeed   (6269.389, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6269.405, PE::SBasicSensor(6.76573349136908+0.24552,0.01));
   fusion.AddSpeed   (6269.440, PE::SBasicSensor(12.5649879331685,1.0));
   fusion.AddAngSpeed(6269.465, PE::SBasicSensor(6.79658030060024+0.24552,0.01));
   fusion.AddSpeed   (6269.489, PE::SBasicSensor(13.5449104614842,1.0));
   fusion.AddAngSpeed(6269.525, PE::SBasicSensor(6.93024980726863+0.24552,0.01));
   fusion.AddSpeed   (6269.539, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6269.585, PE::SBasicSensor(6.90968526778118+0.24552,0.01));
   fusion.AddSpeed   (6269.589, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddSpeed   (6269.639, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6269.645, PE::SBasicSensor(7.11533066265563+0.24552,0.01));
   fusion.AddSpeed   (6269.689, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6269.705, PE::SBasicSensor(7.08448385342446+0.24552,0.01));
   fusion.AddSpeed   (6269.739, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6269.765, PE::SBasicSensor(7.04335477444957+0.24552,0.01));
   fusion.AddSpeed   (6269.789, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6269.825, PE::SBasicSensor(7.1050483929119+0.24552,0.01));
   fusion.AddSpeed   (6269.839, PE::SBasicSensor(13.2740122522545,1.0));
   fusion.AddAngSpeed(6269.885, PE::SBasicSensor(6.98166115598724+0.24552,0.01));
   fusion.AddSpeed   (6269.889, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddSpeed   (6269.939, PE::SBasicSensor(12.3585631314093,1.0));
   fusion.AddAngSpeed(6269.945, PE::SBasicSensor(6.88912072829374+0.24552,0.01));
   fusion.AddSpeed   (6269.989, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6270.005, PE::SBasicSensor(6.85827391906257+0.24552,0.01));
   fusion.AddSpeed   (6270.039, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6270.065, PE::SBasicSensor(6.70403987290674+0.24552,0.01));
   fusion.AddSpeed   (6270.089, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6270.125, PE::SBasicSensor(6.75545122162535+0.24552,0.01));
   fusion.AddSpeed   (6270.139, PE::SBasicSensor(12.3585631314093,1.0));
   fusion.AddAngSpeed(6270.185, PE::SBasicSensor(6.73488668213791+0.24552,0.01));
   fusion.AddSpeed   (6270.189, PE::SBasicSensor(12.3585631314093,1.0));
   fusion.AddSpeed   (6270.239, PE::SBasicSensor(12.3585631314093,1.0));
   fusion.AddAngSpeed(6270.245, PE::SBasicSensor(6.70403987290674+0.24552,0.01));
   fusion.AddSpeed   (6270.289, PE::SBasicSensor(12.8162876918319,1.0));
   fusion.AddAngSpeed(6270.305, PE::SBasicSensor(6.60121717546952+0.24552,0.01));
   fusion.AddSpeed   (6270.339, PE::SBasicSensor(12.3585631314093,1.0));
   fusion.AddAngSpeed(6270.365, PE::SBasicSensor(6.56008809649464+0.24552,0.01));
   fusion.AddSpeed   (6270.389, PE::SBasicSensor(12.3585631314093,1.0));
   fusion.AddAngSpeed(6270.425, PE::SBasicSensor(6.38528951085136+0.24552,0.01));
   fusion.AddSpeed   (6270.440, PE::SBasicSensor(11.6674887950851,1.0));
   fusion.AddAngSpeed(6270.485, PE::SBasicSensor(6.27218454367042+0.24552,0.01));
   fusion.AddSpeed   (6270.489, PE::SBasicSensor(12.6107787055197,1.0));
   fusion.AddSpeed   (6270.539, PE::SBasicSensor(11.9008385709868,1.0));
   fusion.AddAngSpeed(6270.545, PE::SBasicSensor(6.42641858982625+0.24552,0.01));
   fusion.AddSpeed   (6270.589, PE::SBasicSensor(11.9008385709868,1.0));
   fusion.AddAngSpeed(6270.605, PE::SBasicSensor(6.2619022739267+0.24552,0.01));
   fusion.AddSpeed   (6270.639, PE::SBasicSensor(12.3585631314093,1.0));
   fusion.AddAngSpeed(6270.665, PE::SBasicSensor(6.28246681341414+0.24552,0.01));
   fusion.AddSpeed   (6270.689, PE::SBasicSensor(11.9008385709868,1.0));
   fusion.AddAngSpeed(6270.725, PE::SBasicSensor(6.22077319495181+0.24552,0.01));
   fusion.AddSpeed   (6270.739, PE::SBasicSensor(11.9008385709868,1.0));
   fusion.AddAngSpeed(6270.785, PE::SBasicSensor(6.1693618462332+0.24552,0.01));
   fusion.AddSpeed   (6270.789, PE::SBasicSensor(11.4431140105642,1.0));
   fusion.AddSpeed   (6270.839, PE::SBasicSensor(11.4431140105642,1.0));
   fusion.AddAngSpeed(6270.845, PE::SBasicSensor(6.1693618462332+0.24552,0.01));
   fusion.AddSpeed   (6270.889, PE::SBasicSensor(11.9008385709868,1.0));
   fusion.AddAngSpeed(6270.905, PE::SBasicSensor(6.08710368828343+0.24552,0.01));
   fusion.AddSpeed   (6270.939, PE::SBasicSensor(11.4431140105642,1.0));
   fusion.AddAngSpeed(6270.965, PE::SBasicSensor(6.06653914879598+0.24552,0.01));
   fusion.AddSpeed   (6270.989, PE::SBasicSensor(11.4431140105642,1.0));
   fusion.AddAngSpeed(6271.025, PE::SBasicSensor(6.04597460930854+0.24552,0.01));
   fusion.AddSpeed   (6271.039, PE::SBasicSensor(11.4431140105642,1.0));
   fusion.AddAngSpeed(6271.085, PE::SBasicSensor(6.08710368828343+0.24552,0.01));
   fusion.AddSpeed   (6271.089, PE::SBasicSensor(11.4431140105642,1.0));
   fusion.AddSpeed   (6271.139, PE::SBasicSensor(11.4431140105642,1.0));
   fusion.AddAngSpeed(6271.145, PE::SBasicSensor(6.22077319495181+0.24552,0.01));
   fusion.AddSpeed   (6271.189, PE::SBasicSensor(10.9853894501416,1.0));
   fusion.AddAngSpeed(6271.205, PE::SBasicSensor(6.44698312931369+0.24552,0.01));
   fusion.AddSpeed   (6271.239, PE::SBasicSensor(11.4431140105642,1.0));
   fusion.AddAngSpeed(6271.265, PE::SBasicSensor(6.40585405033881+0.24552,0.01));
   fusion.AddSpeed   (6271.289, PE::SBasicSensor(10.9853894501416,1.0));
   fusion.AddAngSpeed(6271.325, PE::SBasicSensor(6.45726539905742+0.24552,0.01));
   fusion.AddSpeed   (6271.339, PE::SBasicSensor(10.9853894501416,1.0));
   fusion.AddAngSpeed(6271.385, PE::SBasicSensor(6.56008809649464+0.24552,0.01));
   fusion.AddSpeed   (6271.389, PE::SBasicSensor(10.9853894501416,1.0));
   fusion.AddSpeed   (6271.440, PE::SBasicSensor(10.7699896570016,1.0));
   fusion.AddAngSpeed(6271.445, PE::SBasicSensor(6.51895901751975+0.24552,0.01));
   fusion.AddSpeed   (6271.489, PE::SBasicSensor(10.7425151935909,1.0));
   fusion.AddAngSpeed(6271.505, PE::SBasicSensor(6.60121717546952+0.24552,0.01));
   fusion.AddSpeed   (6271.539, PE::SBasicSensor(10.9853894501416,1.0));
   fusion.AddAngSpeed(6271.565, PE::SBasicSensor(6.51895901751975+0.24552,0.01));
   fusion.AddSpeed   (6271.589, PE::SBasicSensor(10.9853894501416,1.0));
   fusion.AddAngSpeed(6271.625, PE::SBasicSensor(6.71432214265047+0.24552,0.01));
   fusion.AddSpeed   (6271.639, PE::SBasicSensor(10.5276648897191,1.0));
   fusion.AddAngSpeed(6271.685, PE::SBasicSensor(6.62178171495697+0.24552,0.01));
   fusion.AddSpeed   (6271.689, PE::SBasicSensor(10.5276648897191,1.0));
   fusion.AddSpeed   (6271.739, PE::SBasicSensor(10.5276648897191,1.0));
   fusion.AddAngSpeed(6271.745, PE::SBasicSensor(6.65262852418813+0.24552,0.01));
   fusion.AddSpeed   (6271.789, PE::SBasicSensor(10.9853894501416,1.0));
   fusion.AddAngSpeed(6271.805, PE::SBasicSensor(6.62178171495697+0.24552,0.01));
   fusion.AddSpeed   (6271.839, PE::SBasicSensor(10.5276648897191,1.0));
   fusion.AddAngSpeed(6271.865, PE::SBasicSensor(6.43670085956997+0.24552,0.01));
   fusion.AddSpeed   (6271.889, PE::SBasicSensor(10.5276648897191,1.0));
   fusion.AddAngSpeed(6271.925, PE::SBasicSensor(6.48811220828858+0.24552,0.01));
   fusion.AddSpeed   (6271.939, PE::SBasicSensor(10.0699403292965,1.0));
   fusion.AddAngSpeed(6271.985, PE::SBasicSensor(6.45726539905742+0.24552,0.01));
   fusion.AddSpeed   (6271.989, PE::SBasicSensor(10.5276648897191,1.0));
   fusion.AddSpeed   (6272.039, PE::SBasicSensor(10.5276648897191,1.0));
   fusion.AddAngSpeed(6272.045, PE::SBasicSensor(6.34416043187647+0.24552,0.01));
   fusion.AddSpeed   (6272.089, PE::SBasicSensor(10.0699403292965,1.0));
   fusion.AddAngSpeed(6272.105, PE::SBasicSensor(6.14879730674576+0.24552,0.01));
   fusion.AddSpeed   (6272.139, PE::SBasicSensor(10.5276648897191,1.0));
   fusion.AddAngSpeed(6272.165, PE::SBasicSensor(5.94315191187132+0.24552,0.01));
   fusion.AddSpeed   (6272.189, PE::SBasicSensor(10.0699403292965,1.0));
   fusion.AddAngSpeed(6272.225, PE::SBasicSensor(5.88145829340898+0.24552,0.01));
   fusion.AddSpeed   (6272.239, PE::SBasicSensor(10.0699403292965,1.0));
   fusion.AddAngSpeed(6272.285, PE::SBasicSensor(5.80948240520293+0.24552,0.01));
   fusion.AddSpeed   (6272.289, PE::SBasicSensor(10.0699403292965,1.0));
   fusion.AddSpeed   (6272.339, PE::SBasicSensor(10.0699403292965,1.0));
   fusion.AddAngSpeed(6272.346, PE::SBasicSensor(5.78891786571549+0.24552,0.01));
   fusion.AddSpeed   (6272.389, PE::SBasicSensor(10.0699403292965,1.0));
   fusion.AddAngSpeed(6272.406, PE::SBasicSensor(5.71694197750943+0.24552,0.01));
   fusion.AddSpeed   (6272.440, PE::SBasicSensor(9.87249051891813,1.0));
   fusion.AddAngSpeed(6272.466, PE::SBasicSensor(5.7477887867406+0.24552,0.01));
   fusion.AddSpeed   (6272.489, PE::SBasicSensor(10.2754493156087,1.0));
   fusion.AddAngSpeed(6272.526, PE::SBasicSensor(5.58327247084105+0.24552,0.01));
   fusion.AddSpeed   (6272.539, PE::SBasicSensor(9.61221576887393,1.0));
   fusion.AddAngSpeed(6272.586, PE::SBasicSensor(5.36734480622288+0.24552,0.01));
   fusion.AddSpeed   (6272.589, PE::SBasicSensor(10.0699403292965,1.0));
   fusion.AddSpeed   (6272.639, PE::SBasicSensor(9.61221576887393,1.0));
   fusion.AddAngSpeed(6272.646, PE::SBasicSensor(5.39819161545405+0.24552,0.01));
   fusion.AddSpeed   (6272.689, PE::SBasicSensor(10.0699403292965,1.0));
   fusion.AddAngSpeed(6272.706, PE::SBasicSensor(5.30565118776055+0.24552,0.01));
   fusion.AddSpeed   (6272.739, PE::SBasicSensor(9.61221576887393,1.0));
   fusion.AddAngSpeed(6272.766, PE::SBasicSensor(5.41875615494149+0.24552,0.01));
   fusion.AddSpeed   (6272.789, PE::SBasicSensor(10.0699403292965,1.0));
   fusion.AddAngSpeed(6272.826, PE::SBasicSensor(5.53186112212244+0.24552,0.01));
   fusion.AddSpeed   (6272.839, PE::SBasicSensor(9.61221576887393,1.0));
   fusion.AddAngSpeed(6272.886, PE::SBasicSensor(5.37762707596661+0.24552,0.01));
   fusion.AddSpeed   (6272.889, PE::SBasicSensor(9.61221576887393,1.0));
   fusion.AddSpeed   (6272.939, PE::SBasicSensor(9.61221576887393,1.0));
   fusion.AddAngSpeed(6272.946, PE::SBasicSensor(5.16169941134845+0.24552,0.01));
   fusion.AddSpeed   (6272.989, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6273.006, PE::SBasicSensor(5.0485944441675+0.24552,0.01));
   fusion.AddSpeed   (6273.039, PE::SBasicSensor(9.61221576887393,1.0));
   fusion.AddAngSpeed(6273.066, PE::SBasicSensor(5.08972352314239+0.24552,0.01));
   fusion.AddSpeed   (6273.089, PE::SBasicSensor(9.61221576887393,1.0));
   fusion.AddAngSpeed(6273.126, PE::SBasicSensor(5.20282849032333+0.24552,0.01));
   fusion.AddSpeed   (6273.139, PE::SBasicSensor(9.61221576887393,1.0));
   fusion.AddAngSpeed(6273.186, PE::SBasicSensor(5.10000579288611+0.24552,0.01));
   fusion.AddSpeed   (6273.189, PE::SBasicSensor(9.61221576887393,1.0));
   fusion.AddSpeed   (6273.239, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6273.246, PE::SBasicSensor(5.19254622057961+0.24552,0.01));
   fusion.AddSpeed   (6273.289, PE::SBasicSensor(9.61221576887393,1.0));
   fusion.AddAngSpeed(6273.306, PE::SBasicSensor(5.11028806262983+0.24552,0.01));
   fusion.AddSpeed   (6273.339, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6273.366, PE::SBasicSensor(4.93548947698656+0.24552,0.01));
   fusion.AddSpeed   (6273.389, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6273.426, PE::SBasicSensor(4.98690082570517+0.24552,0.01));
   fusion.AddSpeed   (6273.439, PE::SBasicSensor(9.61221576887393,1.0));
   fusion.AddAngSpeed(6273.486, PE::SBasicSensor(4.70927954262468+0.24552,0.01));
   fusion.AddSpeed   (6273.489, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddSpeed   (6273.539, PE::SBasicSensor(9.61221576887393,1.0));
   fusion.AddAngSpeed(6273.546, PE::SBasicSensor(4.91492493749912+0.24552,0.01));
   fusion.AddSpeed   (6273.589, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6273.606, PE::SBasicSensor(4.79153770057445+0.24552,0.01));
   fusion.AddSpeed   (6273.639, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6273.666, PE::SBasicSensor(4.41109372005674+0.24552,0.01));
   fusion.AddSpeed   (6273.689, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6273.726, PE::SBasicSensor(4.31855329236324+0.24552,0.01));
   fusion.AddSpeed   (6273.739, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6273.786, PE::SBasicSensor(4.40081145031302+0.24552,0.01));
   fusion.AddSpeed   (6273.789, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddSpeed   (6273.839, PE::SBasicSensor(9.61221576887393,1.0));
   fusion.AddAngSpeed(6273.846, PE::SBasicSensor(4.40081145031302+0.24552,0.01));
   fusion.AddSpeed   (6273.889, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6273.906, PE::SBasicSensor(4.41109372005674+0.24552,0.01));
   fusion.AddSpeed   (6273.939, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6273.966, PE::SBasicSensor(4.27742421338835+0.24552,0.01));
   fusion.AddSpeed   (6273.989, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6274.026, PE::SBasicSensor(3.73246391697109+0.24552,0.01));
   fusion.AddSpeed   (6274.039, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6274.086, PE::SBasicSensor(3.21835042978499+0.24552,0.01));
   fusion.AddSpeed   (6274.089, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddSpeed   (6274.139, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6274.146, PE::SBasicSensor(2.79677737029238+0.24552,0.01));
   fusion.AddSpeed   (6274.189, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6274.206, PE::SBasicSensor(2.69395467285516+0.24552,0.01));
   fusion.AddSpeed   (6274.239, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6274.266, PE::SBasicSensor(2.48830927798072+0.24552,0.01));
   fusion.AddSpeed   (6274.289, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6274.326, PE::SBasicSensor(2.08730075797557+0.24552,0.01));
   fusion.AddSpeed   (6274.339, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6274.386, PE::SBasicSensor(1.51149365232713+0.24552,0.01));
   fusion.AddSpeed   (6274.389, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddSpeed   (6274.439, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6274.446, PE::SBasicSensor(0.791734770266594+0.24552,0.01));
   fusion.AddSpeed   (6274.489, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6274.506, PE::SBasicSensor(0.246774473849328+0.24552,0.01));
   fusion.AddSpeed   (6274.539, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6274.566, PE::SBasicSensor(0+0.24552,0.01));
   fusion.AddSpeed   (6274.589, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6274.626, PE::SBasicSensor(0+0.24552,0.01));
   fusion.AddSpeed   (6274.639, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6274.686, PE::SBasicSensor(-0.246774473849328+0.24552,0.01));
   fusion.AddSpeed   (6274.689, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddSpeed   (6274.739, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6274.746, PE::SBasicSensor(-0.709476612316818+0.24552,0.01));
   fusion.AddSpeed   (6274.789, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6274.806, PE::SBasicSensor(-0.791734770266594+0.24552,0.01));
   fusion.AddSpeed   (6274.839, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6274.866, PE::SBasicSensor(-1.18246102052803+0.24552,0.01));
   fusion.AddSpeed   (6274.889, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6274.926, PE::SBasicSensor(-1.15161421129686+0.24552,0.01));
   fusion.AddSpeed   (6274.939, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6274.986, PE::SBasicSensor(-1.64516315899552+0.24552,0.01));
   fusion.AddSpeed   (6274.989, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddSpeed   (6275.039, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6275.046, PE::SBasicSensor(-1.50121138258341+0.24552,0.01));
   fusion.AddSpeed   (6275.089, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6275.106, PE::SBasicSensor(-1.29556598770897+0.24552,0.01));
   fusion.AddSpeed   (6275.139, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6275.166, PE::SBasicSensor(-1.03850924411592+0.24552,0.01));
   fusion.AddSpeed   (6275.189, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6275.226, PE::SBasicSensor(-0.802017040010316+0.24552,0.01));
   fusion.AddSpeed   (6275.239, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6275.286, PE::SBasicSensor(-1.05907378360337+0.24552,0.01));
   fusion.AddSpeed   (6275.289, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddSpeed   (6275.339, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6275.346, PE::SBasicSensor(-1.1207674020657+0.24552,0.01));
   fusion.AddSpeed   (6275.389, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6275.406, PE::SBasicSensor(-0.77117023077915+0.24552,0.01));
   fusion.AddSpeed   (6275.439, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6275.466, PE::SBasicSensor(-0.61693618462332+0.24552,0.01));
   fusion.AddSpeed   (6275.489, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6275.526, PE::SBasicSensor(-0.390726250261436+0.24552,0.01));
   fusion.AddSpeed   (6275.539, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6275.586, PE::SBasicSensor(-0.442137598980046+0.24552,0.01));
   fusion.AddSpeed   (6275.589, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddSpeed   (6275.639, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6275.646, PE::SBasicSensor(-0.349597171286548+0.24552,0.01));
   fusion.AddSpeed   (6275.689, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6275.706, PE::SBasicSensor(-0.226209934361884+0.24552,0.01));
   fusion.AddSpeed   (6275.739, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6275.766, PE::SBasicSensor(-0.133669506668386+0.24552,0.01));
   fusion.AddSpeed   (6275.789, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6275.826, PE::SBasicSensor(0+0.24552,0.01));
   fusion.AddSpeed   (6275.839, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6275.886, PE::SBasicSensor(0.143951776412108+0.24552,0.01));
   fusion.AddSpeed   (6275.889, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddSpeed   (6275.939, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6275.946, PE::SBasicSensor(0.226209934361884+0.24552,0.01));
   fusion.AddSpeed   (6275.989, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6276.006, PE::SBasicSensor(0.483266677954934+0.24552,0.01));
   fusion.AddSpeed   (6276.039, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6276.066, PE::SBasicSensor(0.884275197960092+0.24552,0.01));
   fusion.AddSpeed   (6276.089, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6276.126, PE::SBasicSensor(1.14133194155314+0.24552,0.01));
   fusion.AddSpeed   (6276.139, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6276.186, PE::SBasicSensor(1.39838868514619+0.24552,0.01));
   fusion.AddSpeed   (6276.189, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddSpeed   (6276.239, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6276.246, PE::SBasicSensor(1.59375181027691+0.24552,0.01));
   fusion.AddSpeed   (6276.289, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6276.306, PE::SBasicSensor(1.76855039592018+0.24552,0.01));
   fusion.AddSpeed   (6276.339, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6276.366, PE::SBasicSensor(1.99476033028207+0.24552,0.01));
   fusion.AddSpeed   (6276.389, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6276.426, PE::SBasicSensor(1.99476033028207+0.24552,0.01));
   fusion.AddSpeed   (6276.439, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6276.486, PE::SBasicSensor(1.92278444207601+0.24552,0.01));
   fusion.AddSpeed   (6276.489, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddSpeed   (6276.539, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6276.546, PE::SBasicSensor(1.98447806053835+0.24552,0.01));
   fusion.AddSpeed   (6276.589, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6276.606, PE::SBasicSensor(1.95363125130718+0.24552,0.01));
   fusion.AddSpeed   (6276.639, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6276.666, PE::SBasicSensor(1.84052628412624+0.24552,0.01));
   fusion.AddSpeed   (6276.689, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6276.726, PE::SBasicSensor(1.61431634976435+0.24552,0.01));
   fusion.AddSpeed   (6276.739, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6276.786, PE::SBasicSensor(1.43951776412108+0.24552,0.01));
   fusion.AddSpeed   (6276.789, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddSpeed   (6276.839, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6276.846, PE::SBasicSensor(1.4498000338648+0.24552,0.01));
   fusion.AddSpeed   (6276.889, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6276.906, PE::SBasicSensor(1.56290500104574+0.24552,0.01));
   fusion.AddSpeed   (6276.939, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6276.966, PE::SBasicSensor(1.67600996822669+0.24552,0.01));
   fusion.AddSpeed   (6276.989, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.026, PE::SBasicSensor(1.71713904720157+0.24552,0.01));
   fusion.AddSpeed   (6277.039, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.086, PE::SBasicSensor(1.61431634976435+0.24552,0.01));
   fusion.AddSpeed   (6277.089, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddSpeed   (6277.139, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.146, PE::SBasicSensor(1.47036457335225+0.24552,0.01));
   fusion.AddSpeed   (6277.189, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.206, PE::SBasicSensor(1.57318727078947+0.24552,0.01));
   fusion.AddSpeed   (6277.239, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.266, PE::SBasicSensor(1.52177592207086+0.24552,0.01));
   fusion.AddSpeed   (6277.289, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.326, PE::SBasicSensor(1.51149365232713+0.24552,0.01));
   fusion.AddSpeed   (6277.339, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.386, PE::SBasicSensor(1.61431634976435+0.24552,0.01));
   fusion.AddSpeed   (6277.389, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddSpeed   (6277.439, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.446, PE::SBasicSensor(1.8713730933574+0.24552,0.01));
   fusion.AddSpeed   (6277.489, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.506, PE::SBasicSensor(1.78911493540763+0.24552,0.01));
   fusion.AddSpeed   (6277.539, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.566, PE::SBasicSensor(1.79939720515135+0.24552,0.01));
   fusion.AddSpeed   (6277.589, PE::SBasicSensor(9.15449120845136,1.0));
   fusion.AddAngSpeed(6277.626, PE::SBasicSensor(1.61431634976435+0.24552,0.01));
   fusion.AddSpeed   (6277.639, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.686, PE::SBasicSensor(1.52177592207086+0.24552,0.01));
   fusion.AddSpeed   (6277.689, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddSpeed   (6277.739, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.746, PE::SBasicSensor(1.22359009950292+0.24552,0.01));
   fusion.AddSpeed   (6277.789, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.806, PE::SBasicSensor(0.884275197960092+0.24552,0.01));
   fusion.AddSpeed   (6277.839, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.866, PE::SBasicSensor(0.442137598980046+0.24552,0.01));
   fusion.AddSpeed   (6277.889, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.926, PE::SBasicSensor(0.329032631799104+0.24552,0.01));
   fusion.AddSpeed   (6277.939, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6277.986, PE::SBasicSensor(0.5141134871861+0.24552,0.01));
   fusion.AddSpeed   (6277.989, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddSpeed   (6278.039, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6278.046, PE::SBasicSensor(0.61693618462332+0.24552,0.01));
   fusion.AddSpeed   (6278.089, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6278.106, PE::SBasicSensor(0.678629803085652+0.24552,0.01));
   fusion.AddSpeed   (6278.139, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6278.166, PE::SBasicSensor(0.534678026673544+0.24552,0.01));
   fusion.AddSpeed   (6278.189, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6278.226, PE::SBasicSensor(0.442137598980046+0.24552,0.01));
   fusion.AddSpeed   (6278.239, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6278.286, PE::SBasicSensor(0.380443980517714+0.24552,0.01));
   fusion.AddSpeed   (6278.289, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddSpeed   (6278.339, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.DoFusion();
   EXPECT_NEAR( 6278.339000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63369979, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00635405, fusion.GetPosition().Longitude, 0.00000001);

   fusion.AddAngSpeed(6278.346, PE::SBasicSensor(0.472984408211212+0.24552,0.01));
   fusion.AddSpeed   (6278.389, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6278.406, PE::SBasicSensor(0.5141134871861+0.24552,0.01));
   fusion.AddSpeed   (6278.439, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6278.466, PE::SBasicSensor(0.606653914879598+0.24552,0.01));
   fusion.AddSpeed   (6278.489, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6278.526, PE::SBasicSensor(0.61693618462332+0.24552,0.01));
   fusion.AddSpeed   (6278.539, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6278.586, PE::SBasicSensor(0.5141134871861+0.24552,0.01));
   fusion.AddSpeed   (6278.589, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddSpeed   (6278.639, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6278.646, PE::SBasicSensor(0.493548947698656+0.24552,0.01));
   fusion.AddSpeed   (6278.689, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6278.706, PE::SBasicSensor(0.46270213846749+0.24552,0.01));
   fusion.AddSpeed   (6278.739, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6278.766, PE::SBasicSensor(0.534678026673544+0.24552,0.01));
   fusion.AddSpeed   (6278.789, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6278.826, PE::SBasicSensor(0.596371645135876+0.24552,0.01));
   fusion.AddSpeed   (6278.839, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6278.886, PE::SBasicSensor(0.472984408211212+0.24552,0.01));
   fusion.AddSpeed   (6278.889, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddSpeed   (6278.939, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6278.946, PE::SBasicSensor(0.544960296417266+0.24552,0.01));
   fusion.AddSpeed   (6278.989, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6279.006, PE::SBasicSensor(0.61693618462332+0.24552,0.01));
   fusion.AddSpeed   (6279.039, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6279.066, PE::SBasicSensor(0.688912072829374+0.24552,0.01));
   fusion.AddSpeed   (6279.089, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6279.126, PE::SBasicSensor(0.596371645135876+0.24552,0.01));
   fusion.AddSpeed   (6279.139, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddAngSpeed(6279.186, PE::SBasicSensor(0.596371645135876+0.24552,0.01));
   fusion.AddSpeed   (6279.189, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddSpeed   (6279.239, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddAngSpeed(6279.246, PE::SBasicSensor(0.586089375392154+0.24552,0.01));
   fusion.AddSpeed   (6279.289, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6279.306, PE::SBasicSensor(0.56552483590471+0.24552,0.01));
   fusion.AddSpeed   (6279.339, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddAngSpeed(6279.366, PE::SBasicSensor(0.401008520005158+0.24552,0.01));
   fusion.AddSpeed   (6279.389, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6279.426, PE::SBasicSensor(0.56552483590471+0.24552,0.01));
   fusion.AddSpeed   (6279.439, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddAngSpeed(6279.486, PE::SBasicSensor(0.586089375392154+0.24552,0.01));
   fusion.AddSpeed   (6279.489, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddSpeed   (6279.539, PE::SBasicSensor(8.69676664802879,1.0));
   fusion.AddAngSpeed(6279.546, PE::SBasicSensor(0.46270213846749+0.24552,0.01));
   fusion.AddSpeed   (6279.589, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddAngSpeed(6279.606, PE::SBasicSensor(0.483266677954934+0.24552,0.01));
   fusion.AddSpeed   (6279.639, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddAngSpeed(6279.666, PE::SBasicSensor(0.658065263598208+0.24552,0.01));
   fusion.AddSpeed   (6279.689, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddAngSpeed(6279.726, PE::SBasicSensor(0.555242566160988+0.24552,0.01));
   fusion.AddSpeed   (6279.739, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddAngSpeed(6279.786, PE::SBasicSensor(0.606653914879598+0.24552,0.01));
   fusion.AddSpeed   (6279.789, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddSpeed   (6279.839, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddAngSpeed(6279.846, PE::SBasicSensor(0.35987944103027+0.24552,0.01));
   fusion.AddSpeed   (6279.889, PE::SBasicSensor(7.78131752718366,1.0));
   fusion.AddAngSpeed(6279.906, PE::SBasicSensor(0.472984408211212+0.24552,0.01));
   fusion.AddSpeed   (6279.939, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddAngSpeed(6279.966, PE::SBasicSensor(0.431855329236324+0.24552,0.01));
   fusion.AddSpeed   (6279.989, PE::SBasicSensor(7.78131752718366,1.0));
   fusion.AddAngSpeed(6280.026, PE::SBasicSensor(0.596371645135876+0.24552,0.01));
   fusion.AddSpeed   (6280.039, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddAngSpeed(6280.086, PE::SBasicSensor(0.61693618462332+0.24552,0.01));
   fusion.AddSpeed   (6280.089, PE::SBasicSensor(7.78131752718366,1.0));
   fusion.AddSpeed   (6280.139, PE::SBasicSensor(8.23904208760622,1.0));
   fusion.AddAngSpeed(6280.146, PE::SBasicSensor(0.472984408211212+0.24552,0.01));
   fusion.AddSpeed   (6280.189, PE::SBasicSensor(7.78131752718366,1.0));
   fusion.AddAngSpeed(6280.206, PE::SBasicSensor(0.185080855386996+0.24552,0.01));
   fusion.AddSpeed   (6280.239, PE::SBasicSensor(7.78131752718366,1.0));
   fusion.AddAngSpeed(6280.266, PE::SBasicSensor(0.277621283080494+0.24552,0.01));
   fusion.AddSpeed   (6280.289, PE::SBasicSensor(7.78131752718366,1.0));
   fusion.AddAngSpeed(6280.326, PE::SBasicSensor(0.431855329236324+0.24552,0.01));
   fusion.AddSpeed   (6280.339, PE::SBasicSensor(7.78131752718366,1.0));
   fusion.DoFusion();
   EXPECT_NEAR( 6280.339000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63355319, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00640553, fusion.GetPosition().Longitude, 0.00000001);

   fusion.AddAngSpeed(6280.386, PE::SBasicSensor(0.647782993854486+0.24552,0.01));
   fusion.AddSpeed   (6280.389, PE::SBasicSensor(7.32359296676109,1.0));
   fusion.AddSpeed   (6280.439, PE::SBasicSensor(7.78131752718366,1.0));
   fusion.AddAngSpeed(6280.446, PE::SBasicSensor(0.46270213846749+0.24552,0.01));
   fusion.AddSpeed   (6280.489, PE::SBasicSensor(7.32359296676109,1.0));
   fusion.AddAngSpeed(6280.506, PE::SBasicSensor(0.390726250261436+0.24552,0.01));
   fusion.AddSpeed   (6280.539, PE::SBasicSensor(7.78131752718366,1.0));
   fusion.AddAngSpeed(6280.566, PE::SBasicSensor(0.442137598980046+0.24552,0.01));
   fusion.AddSpeed   (6280.589, PE::SBasicSensor(7.32359296676109,1.0));
   fusion.AddAngSpeed(6280.626, PE::SBasicSensor(0.452419868723768+0.24552,0.01));
   fusion.AddSpeed   (6280.639, PE::SBasicSensor(7.78131752718366,1.0));
   fusion.AddAngSpeed(6280.686, PE::SBasicSensor(0.35987944103027+0.24552,0.01));
   fusion.AddSpeed   (6280.689, PE::SBasicSensor(7.32359296676109,1.0));
   fusion.AddSpeed   (6280.739, PE::SBasicSensor(7.32359296676109,1.0));
   fusion.AddAngSpeed(6280.746, PE::SBasicSensor(0.329032631799104+0.24552,0.01));
   fusion.AddSpeed   (6280.789, PE::SBasicSensor(7.32359296676109,1.0));
   fusion.AddAngSpeed(6280.806, PE::SBasicSensor(0.298185822567938+0.24552,0.01));
   fusion.AddSpeed   (6280.839, PE::SBasicSensor(6.86586840633852,1.0));
   fusion.AddAngSpeed(6280.866, PE::SBasicSensor(0.25705674359305+0.24552,0.01));
   fusion.AddSpeed   (6280.889, PE::SBasicSensor(7.32359296676109,1.0));
   fusion.AddAngSpeed(6280.926, PE::SBasicSensor(0.401008520005158+0.24552,0.01));
   fusion.AddSpeed   (6280.939, PE::SBasicSensor(7.32359296676109,1.0));
   fusion.AddAngSpeed(6280.986, PE::SBasicSensor(0.246774473849328+0.24552,0.01));
   fusion.AddSpeed   (6280.989, PE::SBasicSensor(6.86586840633852,1.0));
   fusion.AddSpeed   (6281.039, PE::SBasicSensor(7.32359296676109,1.0));
   fusion.AddAngSpeed(6281.046, PE::SBasicSensor(0.277621283080494+0.24552,0.01));
   fusion.AddSpeed   (6281.089, PE::SBasicSensor(6.86586840633852,1.0));
   fusion.AddAngSpeed(6281.106, PE::SBasicSensor(0.329032631799104+0.24552,0.01));
   fusion.AddSpeed   (6281.139, PE::SBasicSensor(6.86586840633852,1.0));
   fusion.AddAngSpeed(6281.166, PE::SBasicSensor(0.164516315899552+0.24552,0.01));
   fusion.AddSpeed   (6281.189, PE::SBasicSensor(7.32359296676109,1.0));
   fusion.AddAngSpeed(6281.226, PE::SBasicSensor(0.071975888206054+0.24552,0.01));
   fusion.AddSpeed   (6281.239, PE::SBasicSensor(6.86586840633852,1.0));
   fusion.AddAngSpeed(6281.286, PE::SBasicSensor(0.215927664618162+0.24552,0.01));
   fusion.AddSpeed   (6281.289, PE::SBasicSensor(6.86586840633852,1.0));
   fusion.AddSpeed   (6281.339, PE::SBasicSensor(6.86586840633852,1.0));
   fusion.DoFusion();
   EXPECT_NEAR( 6281.339000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63349326, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00642853, fusion.GetPosition().Longitude, 0.00000001);

   fusion.AddAngSpeed(6281.346, PE::SBasicSensor(0.195363125130718+0.24552,0.01));
   fusion.AddSpeed   (6281.389, PE::SBasicSensor(6.86586840633852,1.0));
   fusion.AddAngSpeed(6281.406, PE::SBasicSensor(0.329032631799104+0.24552,0.01));
   fusion.AddSpeed   (6281.439, PE::SBasicSensor(6.86586840633852,1.0));
   fusion.AddAngSpeed(6281.466, PE::SBasicSensor(0.329032631799104+0.24552,0.01));
   fusion.AddSpeed   (6281.489, PE::SBasicSensor(6.86586840633852,1.0));
   fusion.AddAngSpeed(6281.526, PE::SBasicSensor(0.349597171286548+0.24552,0.01));
   fusion.AddSpeed   (6281.539, PE::SBasicSensor(6.86586840633852,1.0));
   fusion.AddAngSpeed(6281.586, PE::SBasicSensor(0.277621283080494+0.24552,0.01));
   fusion.AddSpeed   (6281.589, PE::SBasicSensor(6.86586840633852,1.0));
   fusion.AddSpeed   (6281.639, PE::SBasicSensor(6.86586840633852,1.0));
   fusion.AddAngSpeed(6281.646, PE::SBasicSensor(0.349597171286548+0.24552,0.01));
   fusion.AddSpeed   (6281.689, PE::SBasicSensor(6.40814384591595,1.0));
   fusion.AddAngSpeed(6281.706, PE::SBasicSensor(0.25705674359305+0.24552,0.01));
   fusion.AddSpeed   (6281.739, PE::SBasicSensor(6.86586840633852,1.0));
   fusion.AddAngSpeed(6281.766, PE::SBasicSensor(0.215927664618162+0.24552,0.01));
   fusion.AddSpeed   (6281.789, PE::SBasicSensor(6.40814384591595,1.0));
   fusion.AddAngSpeed(6281.826, PE::SBasicSensor(0.287903552824216+0.24552,0.01));
   fusion.AddSpeed   (6281.839, PE::SBasicSensor(6.86586840633852,1.0));
   fusion.AddAngSpeed(6281.886, PE::SBasicSensor(0.287903552824216+0.24552,0.01));
   fusion.AddSpeed   (6281.889, PE::SBasicSensor(6.40814384591595,1.0));
   fusion.AddSpeed   (6281.939, PE::SBasicSensor(6.40814384591595,1.0));
   fusion.AddAngSpeed(6281.946, PE::SBasicSensor(0.215927664618162+0.24552,0.01));
   fusion.AddSpeed   (6281.989, PE::SBasicSensor(6.40814384591595,1.0));
   fusion.AddAngSpeed(6282.006, PE::SBasicSensor(0.30846809231166+0.24552,0.01));
   fusion.AddSpeed   (6282.039, PE::SBasicSensor(6.40814384591595,1.0));
   fusion.AddAngSpeed(6282.066, PE::SBasicSensor(0.15423404615583+0.24552,0.01));
   fusion.AddSpeed   (6282.089, PE::SBasicSensor(6.40814384591595,1.0));
   fusion.AddAngSpeed(6282.126, PE::SBasicSensor(0.30846809231166+0.24552,0.01));
   fusion.AddSpeed   (6282.139, PE::SBasicSensor(6.40814384591595,1.0));
   fusion.AddAngSpeed(6282.186, PE::SBasicSensor(0.215927664618162+0.24552,0.01));
   fusion.AddSpeed   (6282.189, PE::SBasicSensor(6.40814384591595,1.0));
   fusion.AddSpeed   (6282.239, PE::SBasicSensor(6.40814384591595,1.0));
   fusion.AddAngSpeed(6282.246, PE::SBasicSensor(0.267339013336772+0.24552,0.01));
   fusion.AddSpeed   (6282.289, PE::SBasicSensor(5.95041928549338,1.0));
   fusion.AddAngSpeed(6282.306, PE::SBasicSensor(0.226209934361884+0.24552,0.01));
   fusion.AddSpeed   (6282.339, PE::SBasicSensor(6.40814384591595,1.0));
   fusion.DoFusion();
   //6282.332  2016-12-16T09:28:50  53,633582   10,006176  175,300003(0,9)
   EXPECT_NEAR( 6282.339000, fusion.GetTimestamp(), 0.00000001);
   EXPECT_NEAR( 53.63343818, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00645053, fusion.GetPosition().Longitude, 0.00000001);
}

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

