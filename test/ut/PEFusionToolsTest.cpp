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
 * Unit test of the methods in PEFusionTools namespace.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include "PEFusionTools.h"
#include "PETools.h"


class PEFusionToolsTest : public ::testing::Test
{
public:
   virtual void SetUp()
   {}
   virtual void TearDown()
   {}
};



/**
 * PredictSensorAccuracy test
 */
TEST_F(PEFusionToolsTest, test_PredictSensorAccuracy )
{
   //test same timestamp
   EXPECT_NEAR(10.1,PE::FUSION::PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1000).Value   , 0.00001);
   EXPECT_NEAR(0.1, PE::FUSION::PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1000).Accuracy, 0.00001);
   //test outdated timestamp
   EXPECT_NEAR(10.1,PE::FUSION::PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),999).Value   , 0.00001);
   EXPECT_NEAR(0.1, PE::FUSION::PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),999).Accuracy, 0.00001);
   //test prediction after 1 second
   EXPECT_NEAR(10.1,PE::FUSION::PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1001).Value   , 0.00001);
   EXPECT_NEAR(0.2, PE::FUSION::PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1001).Accuracy, 0.00001);
   //test prediction after 2 second
   EXPECT_NEAR(10.1,PE::FUSION::PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1002).Value   , 0.00001);
   EXPECT_NEAR(0.3, PE::FUSION::PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1002).Accuracy, 0.00001);
   //test prediction after 10 second
   EXPECT_NEAR(10.1,PE::FUSION::PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1010).Value   , 0.00001);
   EXPECT_NEAR(1.1, PE::FUSION::PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,0.1),1010).Accuracy, 0.00001);
   //test incorrect value
   EXPECT_NEAR(PE::MAX_VALUE, PE::FUSION::PredictSensorAccuracy(1000,PE::SBasicSensor(PE::MAX_VALUE,0.1),1010).Value   , 0.00001);
   EXPECT_NEAR(0.1,           PE::FUSION::PredictSensorAccuracy(1000,PE::SBasicSensor(PE::MAX_VALUE,0.1),1010).Accuracy, 0.00001);
   //test incorrect accuracy
   EXPECT_NEAR(10.1,             PE::FUSION::PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,PE::MAX_ACCURACY),1010).Value   , 0.00001);
   EXPECT_NEAR(PE::MAX_ACCURACY, PE::FUSION::PredictSensorAccuracy(1000,PE::SBasicSensor(10.1,PE::MAX_ACCURACY),1010).Accuracy, 0.00001);
}

/**
 * PredictHeading test by angular speeds
 */
TEST_F(PEFusionToolsTest, test_PredictHeading_by_angular_speed )
{
   PE::SBasicSensor heading (90.0,0.1); //Heading 90 [deg] accuracy +/- 0.1[deg]
   PE::SBasicSensor angSpeed(10.0,0.2); //10[deg/s], accuracy +/-0.2[deg/s] turning LEFT
   //test same timestamp
   EXPECT_NEAR(heading.Value,    PE::FUSION::PredictHeading(1000,heading,1000,angSpeed).Value   , 0.00001);
   EXPECT_NEAR(heading.Accuracy, PE::FUSION::PredictHeading(1000,heading,1000,angSpeed).Accuracy, 0.00001);
   //test outdated timestamp
   EXPECT_NEAR(heading.Value,    PE::FUSION::PredictHeading(1000,heading,999,angSpeed).Value   , 0.00001);
   EXPECT_NEAR(heading.Accuracy, PE::FUSION::PredictHeading(1000,heading,999,angSpeed).Accuracy, 0.00001);
   //test prediction after 1 second
   EXPECT_NEAR(80.0,  PE::FUSION::PredictHeading(1000,heading,1001,angSpeed).Value   , 0.00001);
   EXPECT_NEAR(0.3,   PE::FUSION::PredictHeading(1000,heading,1001,angSpeed).Accuracy, 0.00001);
   //test prediction after 2 second
   EXPECT_NEAR(70.0,  PE::FUSION::PredictHeading(1000,heading,1002,angSpeed).Value   , 0.00001);
   EXPECT_NEAR(0.5,   PE::FUSION::PredictHeading(1000,heading,1002,angSpeed).Accuracy, 0.00001);
   //test prediction after 10 second
   EXPECT_NEAR(350.0, PE::FUSION::PredictHeading(1000,heading,1010,angSpeed).Value   , 0.00001);
   EXPECT_NEAR(2.1,   PE::FUSION::PredictHeading(1000,heading,1010,angSpeed).Accuracy, 0.00001);
   //test incorrect heading
   EXPECT_NEAR(90.0, PE::FUSION::PredictHeading(1000,PE::SBasicSensor(90.0,PE::MAX_ACCURACY),1010,angSpeed).Value, 0.00001);
   EXPECT_NEAR(0.1,  PE::FUSION::PredictHeading(1000,PE::SBasicSensor(PE::MAX_VALUE,0.1),1010,angSpeed).Accuracy,  0.00001);
   //test incorrect angular velocity
   EXPECT_NEAR(90.0, PE::FUSION::PredictHeading(1000,heading,1010,PE::SBasicSensor(10.0,PE::MAX_ACCURACY)).Value, 0.00001);
   EXPECT_NEAR(0.1,  PE::FUSION::PredictHeading(1000,heading,1010,PE::SBasicSensor(PE::MAX_VALUE,0.2)).Accuracy,  0.00001);
}

/**
 * PredictHeading test between two positions
 */
TEST_F(PEFusionToolsTest, test_PredictHeading_between_two_positions )
{
   PE::SPosition pos1 (10.0, 120.0, 10);
   PE::SPosition pos2 (10.01, 120.0, 20); //to pos1 distance is 1111,2m 
   PE::SPosition pos3 (10.001, 120.0, 3); //to pos1 distance is 111,12m 
   PE::SPosition pos4 (10.0001, 120.0, 2); //to pos1 distance is 11,112m 
   PE::SPosition pos5 (10.00001, 120.0, 1); //to pos1 distance is 1,1112m 

   //test invalid positions
   EXPECT_FALSE(PE::FUSION::PredictHeading(PE::SPosition(),PE::SPosition()).IsValid());
   EXPECT_FALSE(PE::FUSION::PredictHeading(pos1,PE::SPosition()).IsValid());
   EXPECT_FALSE(PE::FUSION::PredictHeading(PE::SPosition(),pos1).IsValid());
   //test same positions
   EXPECT_FALSE(PE::FUSION::PredictHeading(pos1,pos1).IsValid());
   //test big distance to Nord
   EXPECT_NEAR(0.0,PE::FUSION::PredictHeading(pos1,pos2).Value   ,0.001);
   EXPECT_NEAR(0.772,PE::FUSION::PredictHeading(pos1,pos2).Accuracy,0.001);
   //test big distance to South
   EXPECT_NEAR(180.0,PE::FUSION::PredictHeading(pos2,pos1).Value,0.001);
   EXPECT_NEAR(0.772,PE::FUSION::PredictHeading(pos2,pos1).Accuracy,0.001);
   //test 111m distance to South
   EXPECT_NEAR(180.0,PE::FUSION::PredictHeading(pos3,pos1).Value,0.001);
   EXPECT_NEAR(3.334,PE::FUSION::PredictHeading(pos3,pos1).Accuracy,0.001);
   //test 10m distance to South
   EXPECT_NEAR(180.0,PE::FUSION::PredictHeading(pos4,pos5).Value,0.001);
   EXPECT_NEAR(8.343,PE::FUSION::PredictHeading(pos4,pos5).Accuracy,0.001);
}


/**
 * PredictPosition turning moving test
 */
TEST_F(PEFusionToolsTest, test_PredictPosition_turning_moving )
{
   PE::SPosition    position   (50.0,120.0,10.0); //Lat=50.0[deg] Lon=120.0[deg] accuracy= 10[m]

   PE::SBasicSensor heading_0  (  0.0,0.1);        //Heading=  0 [deg] accuracy +/- 0.1[deg]
   PE::SBasicSensor heading_10 ( 10.0,0.1);        //Heading= 10 [deg] accuracy +/- 0.1[deg]
   PE::SBasicSensor heading_45 ( 45.0,3.0);        //Heading=45  [deg] accuracy +/- 3[deg]
   PE::SBasicSensor heading_90 ( 90.0,5.0);        //Heading=90  [deg] accuracy +/- 5[deg]
   PE::SBasicSensor heading_180(180.0,2.0);        //Heading=180 [deg] accuracy +/- 2[deg]
   PE::SBasicSensor heading_270(270.0,0.5);        //Heading=270 [deg] accuracy +/- 0.5[deg]
   PE::SBasicSensor heading_355(355.0,0.2);        //Heading=355 [deg] accuracy +/- 0.2[deg]

   PE::SBasicSensor speed_10       (10.0,0.2);        //Speed=10[m/s], accuracy +/-0.2[m/s]
   PE::SBasicSensor speed_100      (100.0,0.2);        //Speed=100[m/s], accuracy +/-0.2[m/s]
   //test same timestamp
   EXPECT_EQ(position, PE::FUSION::PredictPosition(1000, heading_90, 1000, heading_180, position, speed_10));
   //test outdated timestamp
   EXPECT_EQ(position, PE::FUSION::PredictPosition(1000, heading_90, 999, heading_180, position, speed_10));
   //incorrect position
   EXPECT_FALSE(PE::FUSION::PredictPosition(1000, heading_90, 1001, heading_180, PE::SPosition(), speed_10).IsValid());
   //incorrect first heading
   EXPECT_EQ(position, PE::FUSION::PredictPosition(1000, PE::SBasicSensor(), 1001, heading_180, position, speed_10));
   //incorrect last heading
   EXPECT_EQ(position, PE::FUSION::PredictPosition(1000, heading_90, 1001, PE::SBasicSensor(), position, speed_10));
   //incorrect speed
   EXPECT_EQ(position, PE::FUSION::PredictPosition(1000, heading_90, 1001, heading_180, position, PE::SBasicSensor()));
   //test straight moving to the East
   const PE::SPosition pos_1 = PE::FUSION::PredictPosition(1000, heading_90, 1001, heading_90, position, speed_10);
   EXPECT_NEAR( 50.00000000,pos_1.Latitude,0.00000001);
   EXPECT_NEAR(120.00013990,pos_1.Longitude,0.00000001);
   EXPECT_NEAR( 10.20076396,pos_1.HorizontalAcc,0.00000001);
   EXPECT_NEAR(10.0,PE::TOOLS::ToDistancePrecise(pos_1,position),0.00000001);
   //test straight moving to 10 degree
   const PE::SPosition pos_2 = PE::FUSION::PredictPosition(1000, heading_10, 1001, heading_10, position, speed_100);
   EXPECT_NEAR( 50.00088565,pos_2.Latitude,0.00000001);
   EXPECT_NEAR(120.00024295,pos_2.Longitude,0.00000001);
   EXPECT_NEAR( 10.20000030,pos_2.HorizontalAcc,0.00000001);
   EXPECT_NEAR(100.0,PE::TOOLS::ToDistancePrecise(pos_2,position),0.00000001);
   //test turning moving over 0 from 355->10 degree
   const PE::SPosition pos_3 = PE::FUSION::PredictPosition(1000, heading_355, 1001, heading_10, position, speed_100);
   EXPECT_NEAR( 50.00087557,pos_3.Latitude,0.00000001);
   EXPECT_NEAR(120.00024018,pos_3.Longitude,0.00000001);
   EXPECT_NEAR( 10.20000068,pos_3.HorizontalAcc,0.00000001);
   EXPECT_NEAR( 98.86159294,PE::TOOLS::ToDistancePrecise(pos_3,position),0.00000001);
   //test turning moving over 0 from 10->355 degree
   const PE::SPosition pos_4 = PE::FUSION::PredictPosition(1000, heading_10, 1010, heading_355, position, speed_100);
   EXPECT_NEAR( 50.00885699,pos_4.Latitude,0.00000001);
   EXPECT_NEAR(119.99879426,pos_4.Longitude,0.00000001);
   EXPECT_NEAR( 12.00000685,pos_4.HorizontalAcc,0.00000001);
   EXPECT_NEAR(988.61592946,PE::TOOLS::ToDistancePrecise(pos_4,position),0.00000001);
   //test turning moving from 180->90 degree
   const PE::SPosition pos_5 = PE::FUSION::PredictPosition(1000, heading_180, 1001, heading_90, position, speed_100);
   EXPECT_NEAR( 50.00000000,pos_5.Latitude,0.00000001);
   EXPECT_NEAR(120.00089069,pos_5.Longitude,0.00000001);
   EXPECT_NEAR( 10.20037373,pos_5.HorizontalAcc,0.00000001);
   EXPECT_NEAR( 63.66197723,PE::TOOLS::ToDistancePrecise(pos_5,position),0.00000001);
   //test turning moving from 90->180 degree
   const PE::SPosition pos_6 = PE::FUSION::PredictPosition(1000, heading_90, 1001, heading_180, position, speed_100);
   EXPECT_NEAR( 49.99942747,pos_6.Latitude,0.00000001);
   EXPECT_NEAR(120.00000000,pos_6.Longitude,0.00000001);
   EXPECT_NEAR( 10.20037373,pos_6.HorizontalAcc,0.00000001);
   EXPECT_NEAR( 63.66197723,PE::TOOLS::ToDistancePrecise(pos_6,position),0.00000001);
   //test turning moving from 180->270 degree
   const PE::SPosition pos_7 = PE::FUSION::PredictPosition(1000, heading_180, 1001, heading_270, position, speed_100);
   EXPECT_NEAR( 50.00000000,pos_7.Latitude,0.00000001);
   EXPECT_NEAR(119.99910930,pos_7.Longitude,0.00000001);
   EXPECT_NEAR( 10.20004760,pos_7.HorizontalAcc,0.00000001);
   EXPECT_NEAR( 63.66197723,PE::TOOLS::ToDistancePrecise(pos_7,position),0.00000001);
   //test turning moving from 270->180 degree
   const PE::SPosition pos_8 = PE::FUSION::PredictPosition(1000, heading_270, 1001, heading_180, position, speed_100);
   EXPECT_NEAR( 49.99942747,pos_8.Latitude,0.00000001);
   EXPECT_NEAR(120.00000000,pos_8.Longitude,0.00000001);
   EXPECT_NEAR( 10.20004760,pos_8.HorizontalAcc,0.00000001);
   EXPECT_NEAR( 63.66197723,PE::TOOLS::ToDistancePrecise(pos_8,position),0.00000001);
   //test turning moving from 0->270 degree
   const PE::SPosition pos_9 = PE::FUSION::PredictPosition(1000, heading_0, 1001, heading_270, position, speed_100);
   EXPECT_NEAR( 50.00000000,pos_9.Latitude,0.00000001);
   EXPECT_NEAR(119.99910930,pos_9.Longitude,0.00000001);
   EXPECT_NEAR( 10.20000274,pos_9.HorizontalAcc,0.00000001);
   EXPECT_NEAR( 63.66197723,PE::TOOLS::ToDistancePrecise(pos_9,position),0.00000001);
   //test turning moving from 270->0 degree
   const PE::SPosition pos_10 = PE::FUSION::PredictPosition(1000, heading_270, 1001, heading_0, position, speed_100);
   EXPECT_NEAR( 50.00057252,pos_10.Latitude,0.00000001);
   EXPECT_NEAR(120.00000000,pos_10.Longitude,0.00000001);
   EXPECT_NEAR( 10.20000274,pos_10.HorizontalAcc,0.00000001);
   EXPECT_NEAR( 63.66197723,PE::TOOLS::ToDistancePrecise(pos_10,position),0.00000001);
}


/**
 * PredictSpeed test
 */
TEST_F(PEFusionToolsTest, test_PredictSpeed )
{
   PE::SBasicSensor heading_90 (90.0,5.0);        //Heading=90 [deg] accuracy +/- 5[deg]
   PE::SBasicSensor heading_180(180.0,2.0);       //Heading=180 [deg] accuracy +/- 2[deg]
   PE::SBasicSensor heading_45 (45.0,3.0);        //Heading=45 [deg] accuracy +/- 3[deg]
   PE::SBasicSensor speed      (10.0,0.2);        //Speed=10[m/s], accuracy +/-0.2[m/s]

   PE::SPosition position       (50.0,120.0,1.0); //Lat=50.0[deg] Lon=120.0[deg] accuracy= 1[m]
//   PE::SPosition position_1sec  = PE::FUSION::PredictPosition(1000, position, heading_90,  1001, speed);
   PE::SPosition position_1sec  = PE::FUSION::PredictPosition(1000, heading_90, 1001, heading_90, position, speed);
//   PE::SPosition position_2sec  = PE::FUSION::PredictPosition(1000, position, heading_180, 1002, speed);
   PE::SPosition position_2sec  = PE::FUSION::PredictPosition(1000, heading_180, 1002, heading_180, position, speed);
//   PE::SPosition position_10sec = PE::FUSION::PredictPosition(1000, position, heading_45,  1010, speed);
   PE::SPosition position_10sec = PE::FUSION::PredictPosition(1000, heading_45, 1010, heading_45, position, speed);
   //test same timestamp
   EXPECT_FALSE(PE::FUSION::PredictSpeed(1000,position,1000,position_1sec).IsValid());
   //test outdated timestamp
   EXPECT_FALSE(PE::FUSION::PredictSpeed(1000,position,999,position_1sec).IsValid());
   //incorrect first position
   EXPECT_FALSE(PE::FUSION::PredictSpeed(1000,PE::SPosition(),1001,position_1sec).IsValid());
   //incorrect second position
   EXPECT_FALSE(PE::FUSION::PredictSpeed(1000,position,1001,PE::SPosition()).IsValid());
   //test same position
   EXPECT_NEAR(  0.0, PE::FUSION::PredictSpeed(1000,position,1010,position).Value,    0.001);
   EXPECT_NEAR(  0.2, PE::FUSION::PredictSpeed(1000,position,1010,position).Accuracy, 0.001);
   //test prediction after 1 second
   EXPECT_NEAR( 10.0, PE::FUSION::PredictSpeed(1000,position,1001,position_1sec).Value,    0.001);
   EXPECT_NEAR(  2.2, PE::FUSION::PredictSpeed(1000,position,1001,position_1sec).Accuracy, 0.001);
   //test prediction after 2 second
   EXPECT_NEAR( 10.0, PE::FUSION::PredictSpeed(1000,position,1002,position_2sec).Value,    0.001);
   EXPECT_NEAR(  1.2, PE::FUSION::PredictSpeed(1000,position,1002,position_2sec).Accuracy, 0.001);
   //test prediction after 10 second
   EXPECT_NEAR( 10.0, PE::FUSION::PredictSpeed(1000,position,1010,position_10sec).Value,    0.001);
   EXPECT_NEAR(  0.4, PE::FUSION::PredictSpeed(1000,position,1010,position_10sec).Accuracy, 0.001);
}

/**
 * PredictAngSpeed TEST
 */
TEST_F(PEFusionToolsTest, test_PredictAngSpeed )
{
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
   EXPECT_FALSE(PE::FUSION::PredictAngSpeed(1000,heading_10,1000,heading_45).IsValid());
   //test outdated timestamp
   EXPECT_FALSE(PE::FUSION::PredictAngSpeed(1000,heading_10,999, heading_45).IsValid());
   //incorrect first heading
   EXPECT_FALSE(PE::FUSION::PredictAngSpeed(1000,PE::SBasicSensor(),1001,heading_45).IsValid());
   //incorrect second heading
   EXPECT_FALSE(PE::FUSION::PredictAngSpeed(1000,heading_10,1001,PE::SBasicSensor()).IsValid());
   //test same heading
   //--- 10 deg
   EXPECT_NEAR(   0.0, PE::FUSION::PredictAngSpeed(1000,heading_10,1010,heading_10).Value,    0.001);
   EXPECT_NEAR(  0.02, PE::FUSION::PredictAngSpeed(1000,heading_10,1010,heading_10).Accuracy, 0.001);
   //--- 355 deg
   EXPECT_NEAR(   0.0, PE::FUSION::PredictAngSpeed(1000,heading_355,1001,heading_355).Value,    0.001);
   EXPECT_NEAR(   0.4, PE::FUSION::PredictAngSpeed(1000,heading_355,1001,heading_355).Accuracy, 0.001);
   //--- 90 deg
   EXPECT_NEAR(   0.0, PE::FUSION::PredictAngSpeed(1000,heading_90,1002,heading_90).Value,    0.001);
   EXPECT_NEAR(   0.6, PE::FUSION::PredictAngSpeed(1000,heading_90,1002,heading_90).Accuracy, 0.001);
   //--- 270 deg
   EXPECT_NEAR(   0.0, PE::FUSION::PredictAngSpeed(1000,heading_270,1003,heading_270).Value,    0.001);
   EXPECT_NEAR( 0.333, PE::FUSION::PredictAngSpeed(1000,heading_270,1003,heading_270).Accuracy, 0.001);
   //test prediction after 1 second
   //--- 10 -> 45 deg
   EXPECT_NEAR( -35.0, PE::FUSION::PredictAngSpeed(1000,heading_10,1001,heading_45).Value,    0.001);
   EXPECT_NEAR(   0.5, PE::FUSION::PredictAngSpeed(1000,heading_10,1001,heading_45).Accuracy, 0.001);
   //--- 45 -> 10 deg
   EXPECT_NEAR(  35.0, PE::FUSION::PredictAngSpeed(1000,heading_45,1001,heading_10).Value,    0.001);
   EXPECT_NEAR(   0.5, PE::FUSION::PredictAngSpeed(1000,heading_45,1001,heading_10).Accuracy, 0.001);
   //--- 10 -> 355 deg
   EXPECT_NEAR(  15.0, PE::FUSION::PredictAngSpeed(1000,heading_10,1001,heading_355).Value,    0.001);
   EXPECT_NEAR(   0.3, PE::FUSION::PredictAngSpeed(1000,heading_10,1001,heading_355).Accuracy, 0.001);
   //--- 355 -> 10 deg
   EXPECT_NEAR( -15.0, PE::FUSION::PredictAngSpeed(1000,heading_355,1001,heading_10).Value,    0.001);
   EXPECT_NEAR(   0.3, PE::FUSION::PredictAngSpeed(1000,heading_355,1001,heading_10).Accuracy, 0.001);
   //--- 90 -> 270 deg
   EXPECT_NEAR(-180.0, PE::FUSION::PredictAngSpeed(1000,heading_90,1001,heading_270).Value,    0.001);
   EXPECT_NEAR(   1.1, PE::FUSION::PredictAngSpeed(1000,heading_90,1001,heading_270).Accuracy, 0.001);
   //--- 270 -> 90 deg
   EXPECT_NEAR( 180.0, PE::FUSION::PredictAngSpeed(1000,heading_270,1001,heading_90).Value,    0.001);
   EXPECT_NEAR(   1.1, PE::FUSION::PredictAngSpeed(1000,heading_270,1001,heading_90).Accuracy, 0.001);
   //--- 0 -> 180 deg
   EXPECT_NEAR(-180.0, PE::FUSION::PredictAngSpeed(1000,heading_0,1001,heading_180).Value,    0.001);
   EXPECT_NEAR(   0.4, PE::FUSION::PredictAngSpeed(1000,heading_0,1001,heading_180).Accuracy, 0.001);
   //--- 180 -> 0 deg
   EXPECT_NEAR( 180.0, PE::FUSION::PredictAngSpeed(1000,heading_180,1001,heading_0).Value,    0.001);
   EXPECT_NEAR(   0.4, PE::FUSION::PredictAngSpeed(1000,heading_180,1001,heading_0).Accuracy, 0.001);
   //--- 180 -> 270 deg
   EXPECT_NEAR( -90.0, PE::FUSION::PredictAngSpeed(1000,heading_180,1001,heading_270).Value,    0.001);
   EXPECT_NEAR(   0.8, PE::FUSION::PredictAngSpeed(1000,heading_180,1001,heading_270).Accuracy, 0.001);
   //--- 270 -> 180 deg
   EXPECT_NEAR(  90.0, PE::FUSION::PredictAngSpeed(1000,heading_270,1001,heading_180).Value,    0.001);
   EXPECT_NEAR(   0.8, PE::FUSION::PredictAngSpeed(1000,heading_270,1001,heading_180).Accuracy, 0.001);
   //--- 355 -> 91 deg
   EXPECT_NEAR( -96.0, PE::FUSION::PredictAngSpeed(1000,heading_355,1001,heading_91).Value,    0.001);
   EXPECT_NEAR(   0.9, PE::FUSION::PredictAngSpeed(1000,heading_355,1001,heading_91).Accuracy, 0.001);
   //--- 91 -> 355 deg
   EXPECT_NEAR(  96.0, PE::FUSION::PredictAngSpeed(1000,heading_91,1001,heading_355).Value,    0.001);
   EXPECT_NEAR(   0.9, PE::FUSION::PredictAngSpeed(1000,heading_91,1001,heading_355).Accuracy, 0.001);
   //--- 181 -> 1 deg
   EXPECT_NEAR( 180.0, PE::FUSION::PredictAngSpeed(1000,heading_181,1001,heading_1).Value,    0.001);
   EXPECT_NEAR(   1.7, PE::FUSION::PredictAngSpeed(1000,heading_181,1001,heading_1).Accuracy, 0.001);
   //--- 1 -> 181 deg
   EXPECT_NEAR(-180.0, PE::FUSION::PredictAngSpeed(1000,heading_1,1001,heading_181).Value,    0.001);
   EXPECT_NEAR(   1.7, PE::FUSION::PredictAngSpeed(1000,heading_1,1001,heading_181).Accuracy, 0.001);
   //--- 91 -> 181 deg
   EXPECT_NEAR( -90.0, PE::FUSION::PredictAngSpeed(1000,heading_91,1001,heading_181).Value,    0.001);
   EXPECT_NEAR(   1.6, PE::FUSION::PredictAngSpeed(1000,heading_91,1001,heading_181).Accuracy, 0.001);
   //--- 181 -> 91 deg
   EXPECT_NEAR(  90.0, PE::FUSION::PredictAngSpeed(1000,heading_181,1001,heading_91).Value,    0.001);
   EXPECT_NEAR(   1.6, PE::FUSION::PredictAngSpeed(1000,heading_181,1001,heading_91).Accuracy, 0.001);
}


/**
 * MergeHeading test
 */
TEST_F(PEFusionToolsTest, test_MergeHeading )
{
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
   EXPECT_FALSE(PE::FUSION::MergeHeading(PE::SBasicSensor(), PE::SBasicSensor()).IsValid());
   //one sensor is invalid
   EXPECT_EQ(heading_0, PE::FUSION::MergeHeading(PE::SBasicSensor(), heading_0));
   EXPECT_EQ(heading_0, PE::FUSION::MergeHeading(heading_0, PE::SBasicSensor()));
   //--- 0 and 10
   EXPECT_NEAR(   5.0, PE::FUSION::MergeHeading(heading_0,heading_10).Value,    0.001);
   EXPECT_NEAR(   0.1, PE::FUSION::MergeHeading(heading_0,heading_10).Accuracy, 0.001);
   //checking invariant of merging
   EXPECT_NEAR(   5.0, PE::FUSION::MergeHeading(heading_10,heading_0).Value,    0.001);
   EXPECT_NEAR(   0.1, PE::FUSION::MergeHeading(heading_10,heading_0).Accuracy, 0.001);
   //--- 90:0.6 and 180:0.3
   EXPECT_NEAR( 150.0, PE::FUSION::MergeHeading(heading_90,heading_180).Value,    0.001);
   EXPECT_NEAR(   0.4, PE::FUSION::MergeHeading(heading_90,heading_180).Accuracy, 0.001);
   //--- 0:0.1 and 180:0.3
   EXPECT_NEAR(  45.0, PE::FUSION::MergeHeading(heading_0,heading_180).Value,    0.001);
   EXPECT_NEAR(  0.15, PE::FUSION::MergeHeading(heading_0,heading_180).Accuracy, 0.001);
   //--- 90:0.6 and 270:0.5
   EXPECT_NEAR(188.182, PE::FUSION::MergeHeading(heading_90,heading_270).Value,    0.001);
   EXPECT_NEAR(  0.545, PE::FUSION::MergeHeading(heading_90,heading_270).Accuracy, 0.001);
   //--- 91:0.7 and 355:0.2
   EXPECT_NEAR(16.333, PE::FUSION::MergeHeading(heading_91,heading_355).Value,    0.001);
   EXPECT_NEAR( 0.311, PE::FUSION::MergeHeading(heading_91,heading_355).Accuracy, 0.001);
   //--- 181:0.9 and 1:0.8
   EXPECT_NEAR(85.705, PE::FUSION::MergeHeading(heading_181,heading_1).Value,    0.001);
   EXPECT_NEAR( 0.847, PE::FUSION::MergeHeading(heading_181,heading_1).Accuracy, 0.001);
   //--- 355:0.2 and 1:0.8
   EXPECT_NEAR( 356.199, PE::FUSION::MergeHeading(heading_355,heading_1).Value,    0.001);
   EXPECT_NEAR(    0.32, PE::FUSION::MergeHeading(heading_355,heading_1).Accuracy, 0.001);
   //--- 355:0.2 and 10:0.1
   EXPECT_NEAR(   5.0, PE::FUSION::MergeHeading(heading_355,heading_10).Value,    0.001);
   EXPECT_NEAR( 0.133, PE::FUSION::MergeHeading(heading_355,heading_10).Accuracy, 0.001);
   //--- 0:0.1 and 1:0.8
   EXPECT_NEAR( 0.111, PE::FUSION::MergeHeading(heading_0,heading_1).Value,    0.001);
   EXPECT_NEAR( 0.177, PE::FUSION::MergeHeading(heading_0,heading_1).Accuracy, 0.001);
   //--- 90:0.6 and 91:0.7
   EXPECT_NEAR(90.461, PE::FUSION::MergeHeading(heading_90,heading_91).Value,    0.001);
   EXPECT_NEAR( 0.646, PE::FUSION::MergeHeading(heading_90,heading_91).Accuracy, 0.001);
   //--- 180:0.3 and 181:0.9
   EXPECT_NEAR( 180.249, PE::FUSION::MergeHeading(heading_180,heading_181).Value,    0.001);
   EXPECT_NEAR(   0.449, PE::FUSION::MergeHeading(heading_180,heading_181).Accuracy, 0.001);
}


/**
 * MergePosition test
 */
TEST_F(PEFusionToolsTest, test_MergePosition )
{
   PE::SPosition pos_lat0_lon0     = PE::SPosition(    0,    0, 10);
   PE::SPosition pos_lat1N_lon1E   = PE::SPosition(    1,    1,  9);
   PE::SPosition pos_lat1N_lon1W   = PE::SPosition(    1,   -1,  8);
   PE::SPosition pos_lat1S_lon1E   = PE::SPosition(   -1,    1,  7);
   PE::SPosition pos_lat1S_lon1W   = PE::SPosition(   -1,   -1,  6);
   PE::SPosition pos_lat1N_lon179E_acc5 = PE::SPosition(    1,  179,  5);
   PE::SPosition pos_lat1N_lon179W_acc5 = PE::SPosition(    1, -179,  5);
   PE::SPosition pos_lat1S_lon179E_acc4 = PE::SPosition(   -1,  179,  4);
   PE::SPosition pos_lat1S_lon179W_acc3 = PE::SPosition(   -1, -179,  3);
   PE::SPosition pos_lat1S_lon179E_acc2 = PE::SPosition(   -1,  179,  2);

   //all positions are invalid
   EXPECT_FALSE(PE::FUSION::MergePosition(PE::SPosition(), PE::SPosition()).IsValid());
   //one position is invalid
   EXPECT_EQ(pos_lat0_lon0, PE::FUSION::MergePosition(PE::SPosition(), pos_lat0_lon0));
   EXPECT_EQ(pos_lat0_lon0, PE::FUSION::MergePosition(pos_lat0_lon0, PE::SPosition()));
   //merge position around Primemeridian on the North semisphere
   EXPECT_NEAR(   1.000, PE::FUSION::MergePosition(pos_lat1N_lon1E,pos_lat1N_lon1W).Latitude,      0.001);
   EXPECT_NEAR(  -0.059, PE::FUSION::MergePosition(pos_lat1N_lon1E,pos_lat1N_lon1W).Longitude,     0.001);
   EXPECT_NEAR(   8.471, PE::FUSION::MergePosition(pos_lat1N_lon1E,pos_lat1N_lon1W).HorizontalAcc, 0.001);
   //checking invariant of merging
   EXPECT_NEAR(   1.000, PE::FUSION::MergePosition(pos_lat1N_lon1W,pos_lat1N_lon1E).Latitude,      0.001);
   EXPECT_NEAR(  -0.059, PE::FUSION::MergePosition(pos_lat1N_lon1W,pos_lat1N_lon1E).Longitude,     0.001);
   EXPECT_NEAR(   8.471, PE::FUSION::MergePosition(pos_lat1N_lon1W,pos_lat1N_lon1E).HorizontalAcc, 0.001);
   //merge position around Primemeridian on the South semisphere
   EXPECT_NEAR(  -1.000, PE::FUSION::MergePosition(pos_lat1S_lon1W,pos_lat1S_lon1E).Latitude,      0.001);
   EXPECT_NEAR(  -0.077, PE::FUSION::MergePosition(pos_lat1S_lon1W,pos_lat1S_lon1E).Longitude,     0.001);
   EXPECT_NEAR(   6.461, PE::FUSION::MergePosition(pos_lat1S_lon1W,pos_lat1S_lon1E).HorizontalAcc, 0.001);
   //merge position around Ecvator on the East semisphere
   EXPECT_NEAR(  -0.125, PE::FUSION::MergePosition(pos_lat1N_lon1E,pos_lat1S_lon1E).Latitude,      0.001);
   EXPECT_NEAR(   1.000, PE::FUSION::MergePosition(pos_lat1N_lon1E,pos_lat1S_lon1E).Longitude,     0.001);
   EXPECT_NEAR(   7.875, PE::FUSION::MergePosition(pos_lat1N_lon1E,pos_lat1S_lon1E).HorizontalAcc, 0.001);
   //merge position around Ecvator on the West semisphere
   EXPECT_NEAR(  -0.143, PE::FUSION::MergePosition(pos_lat1N_lon1W,pos_lat1S_lon1W).Latitude,      0.001);
   EXPECT_NEAR(  -1.000, PE::FUSION::MergePosition(pos_lat1N_lon1W,pos_lat1S_lon1W).Longitude,     0.001);
   EXPECT_NEAR(   6.857, PE::FUSION::MergePosition(pos_lat1N_lon1W,pos_lat1S_lon1W).HorizontalAcc, 0.001);
   //merge position around Antimeridian on the North semisphere
   EXPECT_NEAR(   1.000, PE::FUSION::MergePosition(pos_lat1N_lon179E_acc5,pos_lat1N_lon179W_acc5).Latitude,      0.001);
   EXPECT_NEAR(-180.000, PE::FUSION::MergePosition(pos_lat1N_lon179E_acc5,pos_lat1N_lon179W_acc5).Longitude,     0.001);
   EXPECT_NEAR(   5.000, PE::FUSION::MergePosition(pos_lat1N_lon179E_acc5,pos_lat1N_lon179W_acc5).HorizontalAcc, 0.001);
   //merge position around Antimeridian on the South semisphere
   EXPECT_NEAR(  -1.000, PE::FUSION::MergePosition(pos_lat1S_lon179E_acc4,pos_lat1S_lon179W_acc3).Latitude,      0.001);
   EXPECT_NEAR(-179.857, PE::FUSION::MergePosition(pos_lat1S_lon179E_acc4,pos_lat1S_lon179W_acc3).Longitude,     0.001);
   EXPECT_NEAR(   3.428, PE::FUSION::MergePosition(pos_lat1S_lon179E_acc4,pos_lat1S_lon179W_acc3).HorizontalAcc, 0.001);
   //checking invariant of merging
   EXPECT_NEAR(  -1.000, PE::FUSION::MergePosition(pos_lat1S_lon179W_acc3,pos_lat1S_lon179E_acc4).Latitude,      0.001);
   EXPECT_NEAR(-179.857, PE::FUSION::MergePosition(pos_lat1S_lon179W_acc3,pos_lat1S_lon179E_acc4).Longitude,     0.001);
   EXPECT_NEAR(   3.428, PE::FUSION::MergePosition(pos_lat1S_lon179W_acc3,pos_lat1S_lon179E_acc4).HorizontalAcc, 0.001);
   //merge position around Antimeridian on the South semisphere
   EXPECT_NEAR(  -1.000, PE::FUSION::MergePosition(pos_lat1S_lon179E_acc2,pos_lat1S_lon179W_acc3).Latitude,      0.001);
   EXPECT_NEAR( 179.800, PE::FUSION::MergePosition(pos_lat1S_lon179E_acc2,pos_lat1S_lon179W_acc3).Longitude,     0.001);
   EXPECT_NEAR(   2.399, PE::FUSION::MergePosition(pos_lat1S_lon179E_acc2,pos_lat1S_lon179W_acc3).HorizontalAcc, 0.001);
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

