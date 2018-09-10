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


TEST_F(PECFusionSensorTest, test_predict_speed )
{
   PE::SPosition pos = PE::SPosition(50.0,10.0,0.1);//lat=50 lon=10
   PE::SBasicSensor heading = PE::SBasicSensor(90.0,5.0);//90deg
   PE::SBasicSensor speed = PE::SBasicSensor(5.0,0.1); //5m/s
   PE::SBasicSensor angSpeed = PE::SBasicSensor(10.0,0.1);//10deg/s
   PE::CFusionSensor fusion = PE::CFusionSensor(1000.0, pos, heading, angSpeed, speed);

   fusion.AddSpeed(1000.5, PE::SBasicSensor(7.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(1000.5000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(   6.3846153, fusion.GetSpeed().Value,0.0000001);
   EXPECT_NEAR(   0.1153846, fusion.GetSpeed().Accuracy,0.0000001);
   EXPECT_NEAR(   6.3846153, fusion.GetSpeed(1001.0).Value,0.0000001);
   EXPECT_NEAR(   0.1730769, fusion.GetSpeed(1001.0).Accuracy,0.0000001);
   EXPECT_NEAR(   6.3846153, fusion.GetSpeed(1001.5).Value,0.0000001);
   EXPECT_NEAR(   0.2307692, fusion.GetSpeed(1001.5).Accuracy,0.0000001);
}


TEST_F(PECFusionSensorTest, test_predict_speed_old_and_same_timestamp )
{
   PE::SPosition pos = PE::SPosition(50.0,10.0,0.1);//lat=50 lon=10
   PE::SBasicSensor heading = PE::SBasicSensor(90.0,5.0);//90deg
   PE::SBasicSensor speed = PE::SBasicSensor(5.0,0.1); //5m/s
   PE::SBasicSensor angSpeed = PE::SBasicSensor(10.0,0.1);//10deg/s
   PE::CFusionSensor fusion = PE::CFusionSensor(1000.0, pos, heading, angSpeed, speed);

   fusion.AddSpeed(1000.5, PE::SBasicSensor(7.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(1000.5000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(   6.3846153, fusion.GetSpeed().Value,0.0000001);
   EXPECT_NEAR(   0.1153846, fusion.GetSpeed().Accuracy,0.0000001);
   //same timestamp
   EXPECT_NEAR(   6.3846153, fusion.GetSpeed(1000.5).Value,0.0000001);
   EXPECT_NEAR(   0.1153846, fusion.GetSpeed(1000.5).Accuracy,0.0000001);
   //old timestamp
   EXPECT_FALSE(fusion.GetSpeed(1000.0).IsValid());
}


TEST_F(PECFusionSensorTest, test_predict_angular_speed )
{
   PE::SPosition pos = PE::SPosition(50.0,10.0,0.1);//lat=50 lon=10
   PE::SBasicSensor heading = PE::SBasicSensor(90.0,5.0);//90deg
   PE::SBasicSensor speed = PE::SBasicSensor(5.0,0.1); //5m/s
   PE::SBasicSensor angSpeed = PE::SBasicSensor(10.0,0.1);//10deg/s
   PE::CFusionSensor fusion = PE::CFusionSensor(1000.0, pos, heading, angSpeed, speed);

   fusion.AddAngSpeed(1000.5, PE::SBasicSensor(12.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(1000.5000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(  11.3846153, fusion.GetAngSpeed().Value,0.0000001);
   EXPECT_NEAR(   0.1153846, fusion.GetAngSpeed().Accuracy,0.0000001);
   EXPECT_NEAR(  11.3846153, fusion.GetAngSpeed(1001.0).Value,0.0000001);
   EXPECT_NEAR(   0.1730769, fusion.GetAngSpeed(1001.0).Accuracy,0.0000001);
   EXPECT_NEAR(  11.3846153, fusion.GetAngSpeed(1001.5).Value,0.0000001);
   EXPECT_NEAR(   0.2307692, fusion.GetAngSpeed(1001.5).Accuracy,0.0000001);
}


TEST_F(PECFusionSensorTest, test_predict_angular_speed_old_and_same_timestamp )
{
   PE::SPosition pos = PE::SPosition(50.0,10.0,0.1);//lat=50 lon=10
   PE::SBasicSensor heading = PE::SBasicSensor(90.0,5.0);//90deg
   PE::SBasicSensor speed = PE::SBasicSensor(5.0,0.1); //5m/s
   PE::SBasicSensor angSpeed = PE::SBasicSensor(10.0,0.1);//10deg/s
   PE::CFusionSensor fusion = PE::CFusionSensor(1000.0, pos, heading, angSpeed, speed);

   fusion.AddAngSpeed(1000.5, PE::SBasicSensor(12.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(1000.5000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(  11.3846153, fusion.GetAngSpeed().Value,0.0000001);
   EXPECT_NEAR(   0.1153846, fusion.GetAngSpeed().Accuracy,0.0000001);
   //same timestamp
   EXPECT_NEAR(  11.3846153, fusion.GetAngSpeed(1000.5).Value,0.0000001);
   EXPECT_NEAR(   0.1153846, fusion.GetAngSpeed(1000.5).Accuracy,0.0000001);
   //old timestamp
   EXPECT_FALSE(fusion.GetAngSpeed(1000.0).IsValid());
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


TEST_F(PECFusionSensorTest, test_do_fusion_on_empty_sensors_queue )
{
   PE::SBasicSensor  heading  ( 90.0,0.1);
   PE::SBasicSensor angSpeed  ( 18.0,0.1); //turn left 18deg/s
   PE::SBasicSensor    speed  ( 10.0,0.1); //10 m/s
   PE::SPosition         pos  ( 50.0,10.0, 0.1);
   PE::CFusionSensor fusion  = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);

   fusion.AddPosition(1.0, PE::SPosition(50.00001401, 10.00013761, 0.1));
   fusion.AddHeading (1.0, PE::SBasicSensor(72.0, 0.1));
   fusion.AddSpeed   (1.0, speed);
   fusion.AddAngSpeed(1.0, angSpeed);
   fusion.DoFusion();
   EXPECT_NEAR( 1.00000000, fusion.GetTimestamp()         , 0.00000001);
   EXPECT_NEAR(50.00001401, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(71.99998642, fusion.GetHeading().Value     , 0.00000001); //72deg
   EXPECT_NEAR(18.00001028, fusion.GetAngSpeed().Value    , 0.00000001); //18 deg/s
   EXPECT_NEAR( 9.99981838, fusion.GetSpeed().Value       , 0.00000001); //10 m/s
   EXPECT_NEAR( 0.12054797, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.13768000, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 0.13533165, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.14117846, fusion.GetSpeed().Accuracy        , 0.00000001);
   fusion.DoFusion();
   fusion.DoFusion();
   fusion.DoFusion();
   fusion.DoFusion();
   EXPECT_NEAR( 1.00000000, fusion.GetTimestamp()         , 0.00000001);
   EXPECT_NEAR(50.00001401, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00013761, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(71.99998642, fusion.GetHeading().Value     , 0.00000001); //72deg
   EXPECT_NEAR(18.00001028, fusion.GetAngSpeed().Value    , 0.00000001); //18 deg/s
   EXPECT_NEAR( 9.99981838, fusion.GetSpeed().Value       , 0.00000001); //10 m/s
   EXPECT_NEAR( 0.12054797, fusion.GetPosition().HorizontalAcc, 0.00000001);
   EXPECT_NEAR( 0.13768000, fusion.GetHeading().Accuracy      , 0.00000001);
   EXPECT_NEAR( 0.13533165, fusion.GetAngSpeed().Accuracy     , 0.00000001);
   EXPECT_NEAR( 0.14117846, fusion.GetSpeed().Accuracy        , 0.00000001);
}


TEST_F(PECFusionSensorTest, test_adding_invalid_position )
{
   PE::SBasicSensor  heading  ( 90.0,0.1);
   PE::SBasicSensor angSpeed  ( 18.0,0.1); //turn left 18deg/s
   PE::SBasicSensor    speed  ( 10.0,0.1); //10 m/s
   PE::SPosition         pos  ( 50.0,10.0, 0.1);
   PE::CFusionSensor fusion  = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);

   fusion.AddPosition(1.0, PE::SPosition(90.01, 180.01, 0.1)); //invalid lat>90 and invalid lon>180
   fusion.DoFusion();
   EXPECT_EQ( 0.0, fusion.GetTimestamp());
   EXPECT_EQ(50.0, fusion.GetPosition().Latitude);
   EXPECT_EQ(10.0, fusion.GetPosition().Longitude);
   EXPECT_EQ(90.0, fusion.GetHeading().Value);
   EXPECT_EQ(18.0, fusion.GetAngSpeed().Value);
   EXPECT_EQ(10.0, fusion.GetSpeed().Value);
   EXPECT_EQ( 0.1, fusion.GetPosition().HorizontalAcc);
   EXPECT_EQ( 0.1, fusion.GetHeading().Accuracy);
   EXPECT_EQ( 0.1, fusion.GetAngSpeed().Accuracy);
   EXPECT_EQ( 0.1, fusion.GetSpeed().Accuracy);
}


TEST_F(PECFusionSensorTest, test_adding_invalid_heading )
{
   PE::SBasicSensor  heading  ( 90.0,0.1);
   PE::SBasicSensor angSpeed  ( 18.0,0.1); //turn left 18deg/s
   PE::SBasicSensor    speed  ( 10.0,0.1); //10 m/s
   PE::SPosition         pos  ( 50.0,10.0, 0.1);
   PE::CFusionSensor fusion  = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);

   fusion.AddHeading(1.0, PE::SBasicSensor()); //invalid heading
   fusion.DoFusion();
   EXPECT_EQ( 0.0, fusion.GetTimestamp());
   EXPECT_EQ(50.0, fusion.GetPosition().Latitude);
   EXPECT_EQ(10.0, fusion.GetPosition().Longitude);
   EXPECT_EQ(90.0, fusion.GetHeading().Value);
   EXPECT_EQ(18.0, fusion.GetAngSpeed().Value);
   EXPECT_EQ(10.0, fusion.GetSpeed().Value);
   EXPECT_EQ( 0.1, fusion.GetPosition().HorizontalAcc);
   EXPECT_EQ( 0.1, fusion.GetHeading().Accuracy);
   EXPECT_EQ( 0.1, fusion.GetAngSpeed().Accuracy);
   EXPECT_EQ( 0.1, fusion.GetSpeed().Accuracy);
}


TEST_F(PECFusionSensorTest, test_adding_invalid_speed )
{
   PE::SBasicSensor  heading  ( 90.0,0.1);
   PE::SBasicSensor angSpeed  ( 18.0,0.1); //turn left 18deg/s
   PE::SBasicSensor    speed  ( 10.0,0.1); //10 m/s
   PE::SPosition         pos  ( 50.0,10.0, 0.1);
   PE::CFusionSensor fusion  = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);

   fusion.AddSpeed(1.0, PE::SBasicSensor()); //invalid speed
   fusion.DoFusion();
   EXPECT_EQ( 0.0, fusion.GetTimestamp());
   EXPECT_EQ(50.0, fusion.GetPosition().Latitude);
   EXPECT_EQ(10.0, fusion.GetPosition().Longitude);
   EXPECT_EQ(90.0, fusion.GetHeading().Value);
   EXPECT_EQ(18.0, fusion.GetAngSpeed().Value);
   EXPECT_EQ(10.0, fusion.GetSpeed().Value);
   EXPECT_EQ( 0.1, fusion.GetPosition().HorizontalAcc);
   EXPECT_EQ( 0.1, fusion.GetHeading().Accuracy);
   EXPECT_EQ( 0.1, fusion.GetAngSpeed().Accuracy);
   EXPECT_EQ( 0.1, fusion.GetSpeed().Accuracy);
}


TEST_F(PECFusionSensorTest, test_adding_invalid_angular_speed )
{
   PE::SBasicSensor  heading  ( 90.0,0.1);
   PE::SBasicSensor angSpeed  ( 18.0,0.1); //turn left 18deg/s
   PE::SBasicSensor    speed  ( 10.0,0.1); //10 m/s
   PE::SPosition         pos  ( 50.0,10.0, 0.1);
   PE::CFusionSensor fusion  = PE::CFusionSensor(0.0,pos,heading,angSpeed,speed);

   fusion.AddAngSpeed(1.0, PE::SBasicSensor()); //invalid angular speed
   fusion.DoFusion();
   EXPECT_EQ( 0.0, fusion.GetTimestamp());
   EXPECT_EQ(50.0, fusion.GetPosition().Latitude);
   EXPECT_EQ(10.0, fusion.GetPosition().Longitude);
   EXPECT_EQ(90.0, fusion.GetHeading().Value);
   EXPECT_EQ(18.0, fusion.GetAngSpeed().Value);
   EXPECT_EQ(10.0, fusion.GetSpeed().Value);
   EXPECT_EQ( 0.1, fusion.GetPosition().HorizontalAcc);
   EXPECT_EQ( 0.1, fusion.GetHeading().Accuracy);
   EXPECT_EQ( 0.1, fusion.GetAngSpeed().Accuracy);
   EXPECT_EQ( 0.1, fusion.GetSpeed().Accuracy);
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
   EXPECT_NEAR(72.00, fusion.GetHeading().Value      , 0.01); //72deg
   EXPECT_NEAR(18.00, fusion.GetAngSpeed().Value     , 0.01);//18 deg/s
   EXPECT_NEAR(10.00, fusion.GetSpeed().Value        , 0.01);//10 m/s
   EXPECT_NEAR( 0.12, fusion.GetPosition().HorizontalAcc, 0.01);
   EXPECT_NEAR( 0.36, fusion.GetHeading().Accuracy      , 0.01);
   EXPECT_NEAR( 0.24, fusion.GetAngSpeed().Accuracy     , 0.01);
   EXPECT_NEAR( 0.20, fusion.GetSpeed().Accuracy        , 0.01);

   fusion.AddPosition(2.0, PE::SPosition(50.00005467, 10.00026176, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(50.00005467, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00026176, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(54.00, fusion.GetHeading().Value     , 0.01);//54 deg
   EXPECT_NEAR(18.00, fusion.GetAngSpeed().Value    , 0.01);
   EXPECT_NEAR(10.00, fusion.GetSpeed().Value       , 0.01);
   EXPECT_NEAR( 0.12, fusion.GetPosition().HorizontalAcc, 0.01);
   EXPECT_NEAR( 0.90, fusion.GetHeading().Accuracy      , 0.01);
   EXPECT_NEAR( 0.57, fusion.GetAngSpeed().Accuracy     , 0.01);
   EXPECT_NEAR( 0.26, fusion.GetSpeed().Accuracy        , 0.01);

   fusion.AddPosition(3.0, PE::SPosition(50.00011800, 10.00036029, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(50.00011800, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00036029, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(36.00, fusion.GetHeading().Value     , 0.01);//36 deg
   EXPECT_NEAR(18.00, fusion.GetAngSpeed().Value    , 0.01);
   EXPECT_NEAR(10.00, fusion.GetSpeed().Value       , 0.01);
   EXPECT_NEAR( 0.11, fusion.GetPosition().HorizontalAcc, 0.01);
   EXPECT_NEAR( 1.70, fusion.GetHeading().Accuracy      , 0.01);
   EXPECT_NEAR( 1.37, fusion.GetAngSpeed().Accuracy     , 0.01);
   EXPECT_NEAR( 0.26, fusion.GetSpeed().Accuracy        , 0.01);

   fusion.AddPosition(4.0, PE::SPosition(50.00019780, 10.00042354, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(50.00019780, fusion.GetPosition().Latitude,  0.0000001);
   EXPECT_NEAR(10.00042354, fusion.GetPosition().Longitude, 0.0000001);
   EXPECT_NEAR(18.00, fusion.GetHeading().Value     , 0.01);//18 deg
   EXPECT_NEAR(18.00, fusion.GetAngSpeed().Value    , 0.01);
   EXPECT_NEAR(10.00, fusion.GetSpeed().Value       , 0.01);
   EXPECT_NEAR( 0.11, fusion.GetPosition().HorizontalAcc, 0.01);
   EXPECT_NEAR( 2.77, fusion.GetHeading().Accuracy      , 0.01);
   EXPECT_NEAR( 3.14, fusion.GetAngSpeed().Accuracy     , 0.01);
   EXPECT_NEAR( 0.26, fusion.GetSpeed().Accuracy        , 0.01);

   fusion.AddPosition(5.0, PE::SPosition(50.00028626, 10.00044534, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR(50.00028626, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(10.00044534, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 0.00, fusion.GetHeading().Value     , 0.01);//0deg
   EXPECT_NEAR(18.00, fusion.GetAngSpeed().Value    , 0.01);
   EXPECT_NEAR(10.00, fusion.GetSpeed().Value       , 0.01);
   EXPECT_NEAR( 0.11, fusion.GetPosition().HorizontalAcc, 0.01);
   EXPECT_NEAR( 4.08, fusion.GetHeading().Accuracy      , 0.01);
   EXPECT_NEAR( 6.21, fusion.GetAngSpeed().Accuracy     , 0.01);
   EXPECT_NEAR( 0.26, fusion.GetSpeed().Accuracy        , 0.01);

   fusion.AddPosition(10.0, PE::SPosition(50.00057252, 10.00000000, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00057252, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(270.00, fusion.GetHeading().Value     , 0.01);
   EXPECT_NEAR( 18.00, fusion.GetAngSpeed().Value    , 0.01);
   EXPECT_NEAR( 10.00, fusion.GetSpeed().Value       , 0.01);
   EXPECT_NEAR(  0.10, fusion.GetPosition().HorizontalAcc, 0.01);
   EXPECT_NEAR(  4.87, fusion.GetHeading().Accuracy      , 0.01);
   EXPECT_NEAR( 10.34, fusion.GetAngSpeed().Accuracy     , 0.01);
   EXPECT_NEAR(  0.34, fusion.GetSpeed().Accuracy        , 0.01);

   fusion.AddPosition(15.0, PE::SPosition(50.00028626, 9.99955464, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00028626, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(  9.99955464, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(180.00, fusion.GetHeading().Value     , 0.01);
   EXPECT_NEAR( 18.00, fusion.GetAngSpeed().Value    , 0.01);
   EXPECT_NEAR(  9.02, fusion.GetSpeed().Value       , 0.01);
   EXPECT_NEAR(  0.10, fusion.GetPosition().HorizontalAcc, 0.01);
   EXPECT_NEAR(  5.61, fusion.GetHeading().Accuracy      , 0.01);
   EXPECT_NEAR( 11.78, fusion.GetAngSpeed().Accuracy     , 0.01);
   EXPECT_NEAR(  0.32, fusion.GetSpeed().Accuracy        , 0.01);

   fusion.AddPosition(20.0, PE::SPosition(50.00000000, 10.00000000, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 90.00, fusion.GetHeading().Value     , 0.01);
   EXPECT_NEAR( 18.00, fusion.GetAngSpeed().Value    , 0.01);
   EXPECT_NEAR(  9.00, fusion.GetSpeed().Value       , 0.01);
   EXPECT_NEAR(  0.10, fusion.GetPosition().HorizontalAcc, 0.01);
   EXPECT_NEAR(  6.34, fusion.GetHeading().Accuracy      , 0.01);
   EXPECT_NEAR( 13.44, fusion.GetAngSpeed().Accuracy     , 0.01);
   EXPECT_NEAR(  0.32, fusion.GetSpeed().Accuracy        , 0.01);
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
   EXPECT_NEAR(359.99, fusion.GetHeading().Value     , 0.01);
   EXPECT_NEAR( 18.00, fusion.GetAngSpeed().Value    , 0.01);
   EXPECT_NEAR( 10.00, fusion.GetSpeed().Value       , 0.01);
   EXPECT_NEAR(  0.10, fusion.GetPosition().HorizontalAcc, 0.01);
   EXPECT_NEAR(  0.12, fusion.GetHeading().Accuracy      , 0.01);
   EXPECT_NEAR(  0.68, fusion.GetAngSpeed().Accuracy     , 0.01);
   EXPECT_NEAR(  0.24, fusion.GetSpeed().Accuracy        , 0.01);

   fusion.AddPosition(10.0, PE::SPosition(50.00057252, 10.00000000, 0.1));
   fusion.AddHeading(10.0, PE::SBasicSensor(270.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00057252, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(270.00, fusion.GetHeading().Value     , 0.01);
   EXPECT_NEAR( 18.00, fusion.GetAngSpeed().Value    , 0.01);
   EXPECT_NEAR( 10.00, fusion.GetSpeed().Value       , 0.01);
   EXPECT_NEAR(  0.10, fusion.GetPosition().HorizontalAcc, 0.01);
   EXPECT_NEAR(  0.11, fusion.GetHeading().Accuracy      , 0.01);
   EXPECT_NEAR(  1.02, fusion.GetAngSpeed().Accuracy     , 0.01);
   EXPECT_NEAR(  0.23, fusion.GetSpeed().Accuracy        , 0.01);

   fusion.AddHeading(15.0, PE::SBasicSensor(180.0, 0.1));
   fusion.AddPosition(15.0, PE::SPosition(50.00028626, 9.99955464, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00028626, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR(  9.99955464, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR(180.00, fusion.GetHeading().Value     , 0.01);
   EXPECT_NEAR( 18.00, fusion.GetAngSpeed().Value    , 0.01);
   EXPECT_NEAR( 10.00, fusion.GetSpeed().Value       , 0.01);
   EXPECT_NEAR(  0.10, fusion.GetPosition().HorizontalAcc, 0.01);
   EXPECT_NEAR(  0.11, fusion.GetHeading().Accuracy      , 0.01);
   EXPECT_NEAR(  0.97, fusion.GetAngSpeed().Accuracy     , 0.01);
   EXPECT_NEAR(  0.23, fusion.GetSpeed().Accuracy        , 0.01);

   fusion.AddPosition(20.0, PE::SPosition(50.00000000, 10.00000000, 0.1));
   fusion.AddHeading(20.0, PE::SBasicSensor(90.0, 0.1));
   fusion.DoFusion();
   EXPECT_NEAR( 50.00000000, fusion.GetPosition().Latitude , 0.00000001);
   EXPECT_NEAR( 10.00000000, fusion.GetPosition().Longitude, 0.00000001);
   EXPECT_NEAR( 90.00, fusion.GetHeading().Value     , 0.01);
   EXPECT_NEAR( 18.00, fusion.GetAngSpeed().Value    , 0.01);
   EXPECT_NEAR( 10.00, fusion.GetSpeed().Value       , 0.01);
   EXPECT_NEAR(  0.10, fusion.GetPosition().HorizontalAcc, 0.01);
   EXPECT_NEAR(  0.11, fusion.GetHeading().Accuracy      , 0.01);
   EXPECT_NEAR(  0.97, fusion.GetAngSpeed().Accuracy     , 0.01);
   EXPECT_NEAR(  0.23, fusion.GetSpeed().Accuracy        , 0.01);
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
   EXPECT_NEAR( 90.00, fusion.GetHeading().Value     , 0.01);
   EXPECT_NEAR( 18.00, fusion.GetAngSpeed().Value    , 0.01);
   EXPECT_NEAR( 10.00, fusion.GetSpeed().Value       , 0.01);
   EXPECT_NEAR(  0.10, fusion.GetPosition().HorizontalAcc, 0.01);
   EXPECT_NEAR(  0.11, fusion.GetHeading().Accuracy      , 0.01);
   EXPECT_NEAR(  0.97, fusion.GetAngSpeed().Accuracy     , 0.01);
   EXPECT_NEAR(  0.23, fusion.GetSpeed().Accuracy        , 0.01);
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


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

