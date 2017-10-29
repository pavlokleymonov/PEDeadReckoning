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

class PECFusionSensorTest : public ::testing::Test
{
public:
   virtual void SetUp()
   {}
   virtual void TearDown()
   {}

   const PE::SBasicSensor& getAngSpeed(const PE::CFusionSensor& fusion)
   {
      return fusion.m_AngSpeed;
   }
   const PE::SBasicSensor& getSpeed(const PE::CFusionSensor& fusion)
   {
      return fusion.m_Speed;
   }
};


/**
 * create test
 */
TEST_F(PECFusionSensorTest, test_create )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(1000.0, PE::SPosition(50.0,10.0,0.1), PE::SBasicSensor(90.0,5.0));
   EXPECT_EQ(1000.0, fusion.GetTimestamp());

   EXPECT_TRUE(fusion.GetPosition().IsValid());
   EXPECT_EQ(50.0, fusion.GetPosition().Latitude);
   EXPECT_EQ(10.0, fusion.GetPosition().Longitude);
   EXPECT_EQ(0.1, fusion.GetPosition().HorizontalAcc);

   EXPECT_TRUE(fusion.GetHeading().IsValid());
   EXPECT_EQ(90.0, fusion.GetHeading().Value);
   EXPECT_EQ(5.0, fusion.GetHeading().Accuracy);

   EXPECT_FALSE(getAngSpeed(fusion).IsValid());
   EXPECT_FALSE(getSpeed(fusion).IsValid());
}

/**
 * PredictSensorAccuracy test
 */
TEST_F(PECFusionSensorTest, test_PredictSensorAccuracy )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(1000.0, PE::SPosition(), PE::SBasicSensor());
   //test same timestamp
   EXPECT_NEAR(10.1,fusion.PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1000).Value   , 0.00001);
   EXPECT_NEAR(0.1, fusion.PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1000).Accuracy, 0.00001);
   //test outdated timestamp
   EXPECT_NEAR(10.1,fusion.PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),999).Value   , 0.00001);
   EXPECT_NEAR(0.1, fusion.PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),999).Accuracy, 0.00001);
   //test prediction after 1 second
   EXPECT_NEAR(10.1,fusion.PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1001).Value   , 0.00001);
   EXPECT_NEAR(0.2, fusion.PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1001).Accuracy, 0.00001);
   //test prediction after 2 second
   EXPECT_NEAR(10.1,fusion.PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1002).Value   , 0.00001);
   EXPECT_NEAR(0.3, fusion.PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1002).Accuracy, 0.00001);
   //test prediction after 10 second
   EXPECT_NEAR(10.1,fusion.PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1010).Value   , 0.00001);
   EXPECT_NEAR(1.1, fusion.PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1010).Accuracy, 0.00001);
   //test incorrect value
   EXPECT_NEAR(PE::MAX_VALUE, fusion.PredictSensorAccuracy(1000,PE::SBasicSensor(PE::MAX_VALUE,0.1),1010).Value   , 0.00001);
   EXPECT_NEAR(0.1,           fusion.PredictSensorAccuracy(1000,PE::SBasicSensor(PE::MAX_VALUE,0.1),1010).Accuracy, 0.00001);
   //test incorrect accuracy
   EXPECT_NEAR(10.1,             fusion.PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,PE::MAX_ACCURACY),1010).Value   , 0.00001);
   EXPECT_NEAR(PE::MAX_ACCURACY, fusion.PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,PE::MAX_ACCURACY),1010).Accuracy, 0.00001);
}

/**
 * PredictHeading test
 */
TEST_F(PECFusionSensorTest, test_PredictHeading )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(1000.0, PE::SPosition(), PE::SBasicSensor());
   PE::SBasicSensor heading (90.0,0.1); //Heading 90 [deg] accuracy +/- 0.1[deg]
   PE::SBasicSensor angSpeed(10.0,0.2); //10[deg/s], accuracy +/-0.2[deg/s] turning LEFT
   //test same timestamp
   EXPECT_NEAR(heading.Value,    fusion.PredictHeading(1000,heading,1000,angSpeed).Value   , 0.00001);
   EXPECT_NEAR(heading.Accuracy, fusion.PredictHeading(1000,heading,1000,angSpeed).Accuracy, 0.00001);
   //test outdated timestamp
   EXPECT_NEAR(heading.Value,    fusion.PredictHeading(1000,heading,999,angSpeed).Value   , 0.00001);
   EXPECT_NEAR(heading.Accuracy, fusion.PredictHeading(1000,heading,999,angSpeed).Accuracy, 0.00001);
   //test prediction after 1 second
   EXPECT_NEAR(80.0,  fusion.PredictHeading(1000,heading,1001,angSpeed).Value   , 0.00001);
   EXPECT_NEAR(0.3,   fusion.PredictHeading(1000,heading,1001,angSpeed).Accuracy, 0.00001);
   //test prediction after 2 second
   EXPECT_NEAR(70.0,  fusion.PredictHeading(1000,heading,1002,angSpeed).Value   , 0.00001);
   EXPECT_NEAR(0.5,   fusion.PredictHeading(1000,heading,1002,angSpeed).Accuracy, 0.00001);
   //test prediction after 10 second
   EXPECT_NEAR(350.0, fusion.PredictHeading(1000,heading,1010,angSpeed).Value   , 0.00001);
   EXPECT_NEAR(2.1,   fusion.PredictHeading(1000,heading,1010,angSpeed).Accuracy, 0.00001);
   //test incorrect heading
   EXPECT_NEAR(90.0, fusion.PredictHeading(1000,PE::SBasicSensor(90.0,PE::MAX_ACCURACY),1010,angSpeed).Value, 0.00001);
   EXPECT_NEAR(0.1,  fusion.PredictHeading(1000,PE::SBasicSensor(PE::MAX_VALUE,0.1),1010,angSpeed).Accuracy,  0.00001);
   //test incorrect angular velocity
   EXPECT_NEAR(90.0, fusion.PredictHeading(1000,heading,1010,PE::SBasicSensor(10.0,PE::MAX_ACCURACY)).Value, 0.00001);
   EXPECT_NEAR(0.1,  fusion.PredictHeading(1000,heading,1010,PE::SBasicSensor(PE::MAX_VALUE,0.2)).Accuracy,  0.00001);
}

/**
 * PredictPosition test
 */
TEST_F(PECFusionSensorTest, test_PredictPosition )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(1000.0, PE::SPosition(), PE::SBasicSensor());
   PE::SPosition    position   (50.0,120.0,10.0); //Lat=50.0[deg] Lon=120.0[deg] accuracy= 10[m]
   PE::SBasicSensor heading_90 (90.0,5.0);        //Heading=90 [deg] accuracy +/- 5[deg]
   PE::SBasicSensor heading_180(180.0,2.0);       //Heading=180 [deg] accuracy +/- 2[deg]
   PE::SBasicSensor heading_45 (45.0,3.0);        //Heading=45 [deg] accuracy +/- 3[deg]
   PE::SBasicSensor speed      (10.0,0.2);        //Speed=10[m/s], accuracy +/-0.2[m/s]
   //test same timestamp
   EXPECT_EQ(position, fusion.PredictPosition(1000,position,heading_90,1000,speed));
   //test outdated timestamp
   EXPECT_EQ(position, fusion.PredictPosition(1000,position,heading_90,999,speed));
   //incorrect position
   EXPECT_FALSE(fusion.PredictPosition(1000,PE::SPosition(),heading_45,1010,speed).IsValid());
   //incorrect heading
   EXPECT_EQ(position, fusion.PredictPosition(1000,position,PE::SBasicSensor(),1010,speed));
   //incorrect speed
   EXPECT_EQ(position, fusion.PredictPosition(1000,position,heading_180,1010,PE::SBasicSensor()));
   //test prediction after 1 second
   EXPECT_NEAR( 50.00000000, fusion.PredictPosition(1000,position,heading_90,1001,speed).Latitude,     0.00000001);
   EXPECT_NEAR(120.00013990, fusion.PredictPosition(1000,position,heading_90,1001,speed).Longitude,    0.00000001);
   EXPECT_NEAR( 10.20076396, fusion.PredictPosition(1000,position,heading_90,1001,speed).HorizontalAcc,0.00000001);
   //test prediction after 2 second
   EXPECT_NEAR( 49.99982013, fusion.PredictPosition(1000,position,heading_180,1002,speed).Latitude,     0.00000001);
   EXPECT_NEAR(120.00000000, fusion.PredictPosition(1000,position,heading_180,1002,speed).Longitude,    0.00000001);
   EXPECT_NEAR( 10.40024381, fusion.PredictPosition(1000,position,heading_180,1002,speed).HorizontalAcc,0.00000001);
   //test prediction after 10 second
   EXPECT_NEAR( 50.00063591, fusion.PredictPosition(1000,position,heading_45,1010,speed).Latitude,     0.00000001);
   EXPECT_NEAR(120.00098932, fusion.PredictPosition(1000,position,heading_45,1010,speed).Longitude,    0.00000001);
   EXPECT_NEAR( 12.00274469, fusion.PredictPosition(1000,position,heading_45,1010,speed).HorizontalAcc,0.00000001);
}

/**
 * PredictSpeed test
 */
TEST_F(PECFusionSensorTest, test_PredictSpeed )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(1000.0, PE::SPosition(), PE::SBasicSensor());

   PE::SBasicSensor heading_90 (90.0,5.0);        //Heading=90 [deg] accuracy +/- 5[deg]
   PE::SBasicSensor heading_180(180.0,2.0);       //Heading=180 [deg] accuracy +/- 2[deg]
   PE::SBasicSensor heading_45 (45.0,3.0);        //Heading=45 [deg] accuracy +/- 3[deg]
   PE::SBasicSensor speed      (10.0,0.2);        //Speed=10[m/s], accuracy +/-0.2[m/s]

   PE::SPosition position       (50.0,120.0,1.0); //Lat=50.0[deg] Lon=120.0[deg] accuracy= 1[m]
   PE::SPosition position_1sec  = fusion.PredictPosition(1000, position, heading_90,  1001, speed);
   PE::SPosition position_2sec  = fusion.PredictPosition(1000, position, heading_180, 1002, speed);
   PE::SPosition position_10sec = fusion.PredictPosition(1000, position, heading_45,  1010, speed);
   //test same timestamp
   EXPECT_FALSE(fusion.PredictSpeed(1000,position,1000,position_1sec).IsValid());
   //test outdated timestamp
   EXPECT_FALSE(fusion.PredictSpeed(1000,position,999,position_1sec).IsValid());
   //incorrect first position
   EXPECT_FALSE(fusion.PredictSpeed(1000,PE::SPosition(),1001,position_1sec).IsValid());
   //incorrect second position
   EXPECT_FALSE(fusion.PredictSpeed(1000,position,1001,PE::SPosition()).IsValid());
   //test same position
   EXPECT_NEAR(  0.0, fusion.PredictSpeed(1000,position,1010,position).Value,    0.001);
   EXPECT_NEAR(  0.2, fusion.PredictSpeed(1000,position,1010,position).Accuracy, 0.001);
   //test prediction after 1 second
   EXPECT_NEAR( 10.0, fusion.PredictSpeed(1000,position,1001,position_1sec).Value,    0.001);
   EXPECT_NEAR(  2.2, fusion.PredictSpeed(1000,position,1001,position_1sec).Accuracy, 0.001);
   //test prediction after 2 second
   EXPECT_NEAR( 10.0, fusion.PredictSpeed(1000,position,1002,position_2sec).Value,    0.001);
   EXPECT_NEAR(  1.2, fusion.PredictSpeed(1000,position,1002,position_2sec).Accuracy, 0.001);
   //test prediction after 10 second
   EXPECT_NEAR( 10.0, fusion.PredictSpeed(1000,position,1010,position_10sec).Value,    0.001);
   EXPECT_NEAR(  0.4, fusion.PredictSpeed(1000,position,1010,position_10sec).Accuracy, 0.001);
}

/**
 * PredictAngSpeed TEST
 */
TEST_F(PECFusionSensorTest, test_PredictAngSpeed )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(1000.0, PE::SPosition(), PE::SBasicSensor());

   PE::SBasicSensor heading_0  (  0.0,0.1);        //Heading=  0 [deg] accuracy +/- 0.1[deg]
   PE::SBasicSensor heading_10 ( 10.0,0.1);        //Heading= 10 [deg] accuracy +/- 0.1[deg]
   PE::SBasicSensor heading_355(355.0,0.2);        //Heading=355 [deg] accuracy +/- 0.2[deg]
   PE::SBasicSensor heading_180(180.0,0.3);        //Heading=180 [deg] accuracy +/- 0.3[deg]
   PE::SBasicSensor heading_45 ( 45.0,0.4);        //Heading= 45 [deg] accuracy +/- 0.4[deg]
   PE::SBasicSensor heading_270(270.0,0.5);        //Heading=270 [deg] accuracy +/- 0.5[deg]
   PE::SBasicSensor heading_90 ( 90.0,0.6);        //Heading= 90 [deg] accuracy +/- 0.6[deg]
   PE::SBasicSensor heading_91 ( 91.0,0.7);        //Heading= 91 [deg] accuracy +/- 0.7[deg]
   PE::SBasicSensor heading_1  (  1.0,0.8);        //Heading= 1 [deg] accuracy +/- 0.8[deg]
   PE::SBasicSensor heading_181(181.0,0.9);        //Heading= 181 [deg] accuracy +/- 0.9[deg]

   //test same timestamp
   EXPECT_FALSE(fusion.PredictAngSpeed(1000,heading_10,1000,heading_45).IsValid());
   //test outdated timestamp
   EXPECT_FALSE(fusion.PredictAngSpeed(1000,heading_10,999, heading_45).IsValid());
   //incorrect first heading
   EXPECT_FALSE(fusion.PredictAngSpeed(1000,PE::SBasicSensor(),1001,heading_45).IsValid());
   //incorrect second heading
   EXPECT_FALSE(fusion.PredictAngSpeed(1000,heading_10,1001,PE::SBasicSensor()).IsValid());
   //test same heading
   //--- 10 deg
   EXPECT_NEAR(   0.0, fusion.PredictAngSpeed(1000,heading_10,1010,heading_10).Value,    0.001);
   EXPECT_NEAR(  0.02, fusion.PredictAngSpeed(1000,heading_10,1010,heading_10).Accuracy, 0.001);
   //--- 355 deg
   EXPECT_NEAR(   0.0, fusion.PredictAngSpeed(1000,heading_355,1001,heading_355).Value,    0.001);
   EXPECT_NEAR(   0.4, fusion.PredictAngSpeed(1000,heading_355,1001,heading_355).Accuracy, 0.001);
   //--- 90 deg
   EXPECT_NEAR(   0.0, fusion.PredictAngSpeed(1000,heading_90,1002,heading_90).Value,    0.001);
   EXPECT_NEAR(   0.6, fusion.PredictAngSpeed(1000,heading_90,1002,heading_90).Accuracy, 0.001);
   //--- 270 deg
   EXPECT_NEAR(   0.0, fusion.PredictAngSpeed(1000,heading_270,1003,heading_270).Value,    0.001);
   EXPECT_NEAR( 0.333, fusion.PredictAngSpeed(1000,heading_270,1003,heading_270).Accuracy, 0.001);
   //test prediction after 1 second
   //--- 10 -> 45 deg
   EXPECT_NEAR( -35.0, fusion.PredictAngSpeed(1000,heading_10,1001,heading_45).Value,    0.001);
   EXPECT_NEAR(   0.5, fusion.PredictAngSpeed(1000,heading_10,1001,heading_45).Accuracy, 0.001);
   //--- 45 -> 10 deg
   EXPECT_NEAR(  35.0, fusion.PredictAngSpeed(1000,heading_45,1001,heading_10).Value,    0.001);
   EXPECT_NEAR(   0.5, fusion.PredictAngSpeed(1000,heading_45,1001,heading_10).Accuracy, 0.001);
   //--- 10 -> 355 deg
   EXPECT_NEAR(  15.0, fusion.PredictAngSpeed(1000,heading_10,1001,heading_355).Value,    0.001);
   EXPECT_NEAR(   0.3, fusion.PredictAngSpeed(1000,heading_10,1001,heading_355).Accuracy, 0.001);
   //--- 355 -> 10 deg
   EXPECT_NEAR( -15.0, fusion.PredictAngSpeed(1000,heading_355,1001,heading_10).Value,    0.001);
   EXPECT_NEAR(   0.3, fusion.PredictAngSpeed(1000,heading_355,1001,heading_10).Accuracy, 0.001);
   //--- 90 -> 270 deg
   EXPECT_NEAR(-180.0, fusion.PredictAngSpeed(1000,heading_90,1001,heading_270).Value,    0.001);
   EXPECT_NEAR(   1.1, fusion.PredictAngSpeed(1000,heading_90,1001,heading_270).Accuracy, 0.001);
   //--- 270 -> 90 deg
   EXPECT_NEAR( 180.0, fusion.PredictAngSpeed(1000,heading_270,1001,heading_90).Value,    0.001);
   EXPECT_NEAR(   1.1, fusion.PredictAngSpeed(1000,heading_270,1001,heading_90).Accuracy, 0.001);
   //--- 0 -> 180 deg
   EXPECT_NEAR(-180.0, fusion.PredictAngSpeed(1000,heading_0,1001,heading_180).Value,    0.001);
   EXPECT_NEAR(   0.4, fusion.PredictAngSpeed(1000,heading_0,1001,heading_180).Accuracy, 0.001);
   //--- 180 -> 0 deg
   EXPECT_NEAR( 180.0, fusion.PredictAngSpeed(1000,heading_180,1001,heading_0).Value,    0.001);
   EXPECT_NEAR(   0.4, fusion.PredictAngSpeed(1000,heading_180,1001,heading_0).Accuracy, 0.001);
   //--- 180 -> 270 deg
   EXPECT_NEAR( -90.0, fusion.PredictAngSpeed(1000,heading_180,1001,heading_270).Value,    0.001);
   EXPECT_NEAR(   0.8, fusion.PredictAngSpeed(1000,heading_180,1001,heading_270).Accuracy, 0.001);
   //--- 270 -> 180 deg
   EXPECT_NEAR(  90.0, fusion.PredictAngSpeed(1000,heading_270,1001,heading_180).Value,    0.001);
   EXPECT_NEAR(   0.8, fusion.PredictAngSpeed(1000,heading_270,1001,heading_180).Accuracy, 0.001);
   //--- 355 -> 91 deg
   EXPECT_NEAR( -96.0, fusion.PredictAngSpeed(1000,heading_355,1001,heading_91).Value,    0.001);
   EXPECT_NEAR(   0.9, fusion.PredictAngSpeed(1000,heading_355,1001,heading_91).Accuracy, 0.001);
   //--- 91 -> 355 deg
   EXPECT_NEAR(  96.0, fusion.PredictAngSpeed(1000,heading_91,1001,heading_355).Value,    0.001);
   EXPECT_NEAR(   0.9, fusion.PredictAngSpeed(1000,heading_91,1001,heading_355).Accuracy, 0.001);
   //--- 181 -> 1 deg
   EXPECT_NEAR( 180.0, fusion.PredictAngSpeed(1000,heading_181,1001,heading_1).Value,    0.001);
   EXPECT_NEAR(   1.7, fusion.PredictAngSpeed(1000,heading_181,1001,heading_1).Accuracy, 0.001);
   //--- 1 -> 181 deg
   EXPECT_NEAR(-180.0, fusion.PredictAngSpeed(1000,heading_1,1001,heading_181).Value,    0.001);
   EXPECT_NEAR(   1.7, fusion.PredictAngSpeed(1000,heading_1,1001,heading_181).Accuracy, 0.001);
   //--- 91 -> 181 deg
   EXPECT_NEAR( -90.0, fusion.PredictAngSpeed(1000,heading_91,1001,heading_181).Value,    0.001);
   EXPECT_NEAR(   1.6, fusion.PredictAngSpeed(1000,heading_91,1001,heading_181).Accuracy, 0.001);
   //--- 181 -> 91 deg
   EXPECT_NEAR(  90.0, fusion.PredictAngSpeed(1000,heading_181,1001,heading_91).Value,    0.001);
   EXPECT_NEAR(   1.6, fusion.PredictAngSpeed(1000,heading_181,1001,heading_91).Accuracy, 0.001);
}


/**
 * MergeHeading test
 */
TEST_F(PECFusionSensorTest, test_MergeHeading )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(1000.0, PE::SPosition(), PE::SBasicSensor());

   PE::SBasicSensor heading_0  (  0.0,0.1);        //Heading=  0 [deg] accuracy +/- 0.1[deg]
   PE::SBasicSensor heading_10 ( 10.0,0.1);        //Heading= 10 [deg] accuracy +/- 0.1[deg]
   PE::SBasicSensor heading_355(355.0,0.2);        //Heading=355 [deg] accuracy +/- 0.2[deg]
   PE::SBasicSensor heading_180(180.0,0.3);        //Heading=180 [deg] accuracy +/- 0.3[deg]
   PE::SBasicSensor heading_45 ( 45.0,0.4);        //Heading= 45 [deg] accuracy +/- 0.4[deg]
   PE::SBasicSensor heading_270(270.0,0.5);        //Heading=270 [deg] accuracy +/- 0.5[deg]
   PE::SBasicSensor heading_90 ( 90.0,0.6);        //Heading= 90 [deg] accuracy +/- 0.6[deg]
   PE::SBasicSensor heading_91 ( 91.0,0.7);        //Heading= 91 [deg] accuracy +/- 0.7[deg]
   PE::SBasicSensor heading_1  (  1.0,0.8);        //Heading= 1 [deg] accuracy +/- 0.8[deg]
   PE::SBasicSensor heading_181(181.0,0.9);        //Heading= 181 [deg] accuracy +/- 0.9[deg]

   //all sensors are invalid
   EXPECT_FALSE(fusion.MergeHeading(PE::SBasicSensor(), PE::SBasicSensor()).IsValid());
   //one sensor is invalid
   EXPECT_EQ(heading_0, fusion.MergeHeading(PE::SBasicSensor(), heading_0));
   EXPECT_EQ(heading_0, fusion.MergeHeading(heading_0, PE::SBasicSensor()));
   //--- 0 and 10
   EXPECT_NEAR(   5.0, fusion.MergeHeading(heading_0,heading_10).Value,    0.001);
   EXPECT_NEAR(   0.1, fusion.MergeHeading(heading_0,heading_10).Accuracy, 0.001);
   EXPECT_NEAR(   5.0, fusion.MergeHeading(heading_10,heading_0).Value,    0.001);
   EXPECT_NEAR(   0.1, fusion.MergeHeading(heading_10,heading_0).Accuracy, 0.001);
   //--- 90:0.6 and 180:0.3
   EXPECT_NEAR( 150.0, fusion.MergeHeading(heading_90,heading_180).Value,    0.001);
   EXPECT_NEAR(   0.4, fusion.MergeHeading(heading_90,heading_180).Accuracy, 0.001);
   //--- 0:0.1 and 180:0.3
   EXPECT_NEAR(  45.0, fusion.MergeHeading(heading_0,heading_180).Value,    0.001);
   EXPECT_NEAR(  0.15, fusion.MergeHeading(heading_0,heading_180).Accuracy, 0.001);
   //--- 90:0.6 and 270:0.5
   EXPECT_NEAR(188.182, fusion.MergeHeading(heading_90,heading_270).Value,    0.001);
   EXPECT_NEAR(  0.545, fusion.MergeHeading(heading_90,heading_270).Accuracy, 0.001);
   //--- 91:0.7 and 355:0.2
   EXPECT_NEAR(16.333, fusion.MergeHeading(heading_91,heading_355).Value,    0.001);
   EXPECT_NEAR( 0.311, fusion.MergeHeading(heading_91,heading_355).Accuracy, 0.001);
   //--- 181:0.9 and 1:0.8
   EXPECT_NEAR(85.705, fusion.MergeHeading(heading_181,heading_1).Value,    0.001);
   EXPECT_NEAR( 0.847, fusion.MergeHeading(heading_181,heading_1).Accuracy, 0.001);
   //--- 355:0.2 and 1:0.8
   EXPECT_NEAR( 356.199, fusion.MergeHeading(heading_355,heading_1).Value,    0.001);
   EXPECT_NEAR(    0.32, fusion.MergeHeading(heading_355,heading_1).Accuracy, 0.001);
   //--- 355:0.2 and 10:0.1
   EXPECT_NEAR(   5.0, fusion.MergeHeading(heading_355,heading_10).Value,    0.001);
   EXPECT_NEAR( 0.133, fusion.MergeHeading(heading_355,heading_10).Accuracy, 0.001);
   //--- 0:0.1 and 1:0.8
   EXPECT_NEAR( 0.111, fusion.MergeHeading(heading_0,heading_1).Value,    0.001);
   EXPECT_NEAR( 0.177, fusion.MergeHeading(heading_0,heading_1).Accuracy, 0.001);
   //--- 90:0.6 and 91:0.7
   EXPECT_NEAR(90.461, fusion.MergeHeading(heading_90,heading_91).Value,    0.001);
   EXPECT_NEAR( 0.646, fusion.MergeHeading(heading_90,heading_91).Accuracy, 0.001);
   //--- 180:0.3 and 181:0.9
   EXPECT_NEAR( 180.249, fusion.MergeHeading(heading_180,heading_181).Value,    0.001);
   EXPECT_NEAR(   0.449, fusion.MergeHeading(heading_180,heading_181).Accuracy, 0.001);
}


/**
 * Adding speed test
 */
TEST_F(PECFusionSensorTest, test_first_invalid_position )
{
   /*
   PE::CFusionSensor fusion = PE::CFusionSensor(1000.0, PE::SPosition(), PE::SBasicSensor());
   fusion.AddPosition(1000.0, PE::SPosition(50.0,100.0,10.0), PE::SBasicSensor(90.0,5.0)); //same timestamp
   EXPECT_FALSE(fusion.GetPosition().IsValid());
   fusion.AddPosition(1001.0, PE::SPosition(50.0,100.0), PE::SBasicSensor(90.0,5.0)); //invalid position
   EXPECT_FALSE(fusion.GetPosition().IsValid());
   fusion.AddPosition(1001.0, PE::SPosition(50.0,100.0,10.0), PE::SBasicSensor()); //invalid heading
   EXPECT_FALSE(fusion.GetPosition().IsValid());
   fusion.AddPosition(1001.0, PE::SPosition(50.0,100.0,10.0), PE::SBasicSensor(90.0,5.0)); //all valid
   EXPECT_TRUE(fusion.GetPosition().IsValid());
   EXPECT_EQ(1001.0, fusion.GetTimestamp());
   EXPECT_EQ(PE::SPosition(50.0,100.0,10.0), fusion.GetPosition());
   EXPECT_EQ(PE::SBasicSensor(90.0,5.0), fusion.GetHeading());
   EXPECT_FALSE( getSpeed(fusion).IsValid()); //speed invalid
   EXPECT_FALSE( getAngSpeed(fusion).IsValid()); //angular speed invalid
   */
}

/**
 * Adding angular speed test
 */
TEST_F(PECFusionSensorTest, test_add_angular_speed )
{
   //ToDo test send data without valid position
   //test data only angular velosity
   //test first position is valid
}

/**
 * Adding position test
 */
TEST_F(PECFusionSensorTest, test_add_position )
{
/*


   //TS       Lat[deg]    Long[deg]   Head[deg]   hAcc[deg]   horAcc[m]   AngSpeed[deg/s]     Speed km/h
   //6216331  53,640096   10,004298   89,400002   0,3         2           -3,40340540540541   71,5450012809687
   PE::CFusionSensor fusion = PE::CFusionSensor(
                                 6216.331, //TS
                                 PE::SPosition(53.640096, 10.004298, 2), //Position
                                 PE::SBasicSensor(89.400002,0.3) //Heading
                              );
   //TS       Lat[deg]    Long[deg]   Head[deg]   hAcc[deg]   horAcc[m]   AngSpeed[deg/s]     Speed km/h
   //6217332  53,640094   10,0046     93          0,3         2           -3,5964015984016    71,6042112782731
   fusion.AddPosition(
             6217.332, //TS
             PE::SPosition(53.640094, 10.0046, 2), //Position
             PE::SBasicSensor(93.0,0.3) //Heading
          );
   EXPECT_EQ(6217.332, fusion.GetTimestamp());
   EXPECT_TRUE(fusion.GetPosition().IsValid());
   EXPECT_EQ(53.640094, fusion.GetPosition().Latitude);
   EXPECT_EQ(10.0046, fusion.GetPosition().Longitude);
   EXPECT_EQ(2, fusion.GetPosition().HorizontalAcc);
   EXPECT_TRUE(fusion.GetHeading().IsValid());
   EXPECT_EQ(93.0, fusion.GetHeading().Value);
   EXPECT_EQ(0.3, fusion.GetHeading().Accuracy);
   EXPECT_TRUE(getAngSpeed(fusion).IsValid());
   EXPECT_NEAR(-3.596, getAngSpeed(fusion).Value, 0.001);
   EXPECT_NEAR(0.01, getAngSpeed(fusion).Accuracy, 0.01);
   EXPECT_TRUE(getSpeed(fusion).IsValid());
   EXPECT_NEAR(19.89, getSpeed(fusion).Value, 0.001);
   EXPECT_NEAR(0.01, getSpeed(fusion).Accuracy, 0.01);

   //TS       Lat[deg]    Long[deg]   Head[deg]   hAcc[deg]   horAcc[m]   AngSpeed[deg/s]     Speed km/h
   //6218331  53,640079   10,004898   97          0,3         1           -4,004004004004     71,0467784073282
   fusion.AddPosition(
             6218.331, //TS
             PE::SPosition(53.640079, 10.004898, 1), //Position
             PE::SBasicSensor(97.0,0.3) //Heading
          );
   EXPECT_EQ(6218.331, fusion.GetTimestamp());
   EXPECT_TRUE(fusion.GetPosition().IsValid());
   EXPECT_EQ(53.0, fusion.GetPosition().Latitude);
   EXPECT_EQ(10.0, fusion.GetPosition().Longitude);
   EXPECT_EQ(0, fusion.GetPosition().HorizontalAcc);
   EXPECT_TRUE(fusion.GetHeading().IsValid());
   EXPECT_EQ(97.0, fusion.GetHeading().Value);
   EXPECT_EQ(0.3, fusion.GetHeading().Accuracy);
   EXPECT_TRUE(getAngSpeed(fusion).IsValid());
   EXPECT_NEAR(-4.004, getAngSpeed(fusion).Value, 0.001);
   EXPECT_NEAR(0.01, getAngSpeed(fusion).Accuracy, 0.01);
   EXPECT_TRUE(getSpeed(fusion).IsValid());
   EXPECT_NEAR(19.73, getSpeed(fusion).Value, 0.001);
   EXPECT_NEAR(0.01, getSpeed(fusion).Accuracy, 0.01);

*/
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

