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
   EXPECT_NEAR(10.1,PE::FUSION::PredictSensorAccuracy(0,PE::SBasicSensor(10.1,0.1)).Value   , 0.00001);
   EXPECT_NEAR(0.1, PE::FUSION::PredictSensorAccuracy(0,PE::SBasicSensor(10.1,0.1)).Accuracy, 0.00001);
   //test outdated timestamp
   EXPECT_NEAR(10.1,PE::FUSION::PredictSensorAccuracy(-1,PE::SBasicSensor(10.1,0.1)).Value   , 0.00001);
   EXPECT_NEAR(0.1, PE::FUSION::PredictSensorAccuracy(-1,PE::SBasicSensor(10.1,0.1)).Accuracy, 0.00001);
   //test prediction after 1 second
   EXPECT_NEAR(10.1,PE::FUSION::PredictSensorAccuracy(1,PE::SBasicSensor(10.1,0.1)).Value   , 0.00001);
   EXPECT_NEAR(0.2, PE::FUSION::PredictSensorAccuracy(1,PE::SBasicSensor(10.1,0.1)).Accuracy, 0.00001);
   //test prediction after 2 second
   EXPECT_NEAR(10.1,PE::FUSION::PredictSensorAccuracy(2,PE::SBasicSensor(10.1,0.1)).Value   , 0.00001);
   EXPECT_NEAR(0.3, PE::FUSION::PredictSensorAccuracy(2,PE::SBasicSensor(10.1,0.1)).Accuracy, 0.00001);
   //test prediction after 10 second
   EXPECT_NEAR(10.1,PE::FUSION::PredictSensorAccuracy(10,PE::SBasicSensor(10.1,0.1)).Value   , 0.00001);
   EXPECT_NEAR(1.1, PE::FUSION::PredictSensorAccuracy(10,PE::SBasicSensor(10.1,0.1)).Accuracy, 0.00001);
   //test incorrect value
   EXPECT_NEAR(PE::MAX_VALUE, PE::FUSION::PredictSensorAccuracy(10,PE::SBasicSensor(PE::MAX_VALUE,0.1)).Value   , 0.00001);
   EXPECT_NEAR(0.1,           PE::FUSION::PredictSensorAccuracy(10,PE::SBasicSensor(PE::MAX_VALUE,0.1)).Accuracy, 0.00001);
   //test incorrect accuracy
   EXPECT_NEAR(10.1,             PE::FUSION::PredictSensorAccuracy(10,PE::SBasicSensor(10.1,PE::MAX_ACCURACY)).Value   , 0.00001);
   EXPECT_NEAR(PE::MAX_ACCURACY, PE::FUSION::PredictSensorAccuracy(10,PE::SBasicSensor(10.1,PE::MAX_ACCURACY)).Accuracy, 0.00001);
}

/**
 * PredictHeading test by angular speeds
 */
TEST_F(PEFusionToolsTest, test_PredictHeading_by_angular_speed )
{
   PE::SBasicSensor heading (90.0,0.1); //Heading 90 [deg] accuracy +/- 0.1[deg]
   PE::SBasicSensor angSpeed(10.0,0.2); //10[deg/s], accuracy +/-0.2[deg/s] turning LEFT
   //test same timestamp
   EXPECT_NEAR(heading.Value,    PE::FUSION::PredictHeading(0,heading,angSpeed).Value   , 0.00001);
   EXPECT_NEAR(heading.Accuracy, PE::FUSION::PredictHeading(0,heading,angSpeed).Accuracy, 0.00001);
   //test outdated timestamp
   EXPECT_NEAR(heading.Value,    PE::FUSION::PredictHeading(-1,heading,angSpeed).Value   , 0.00001);
   EXPECT_NEAR(heading.Accuracy, PE::FUSION::PredictHeading(-1,heading,angSpeed).Accuracy, 0.00001);
   //test prediction after 1 second
   EXPECT_NEAR(80.0,  PE::FUSION::PredictHeading(1,heading,angSpeed).Value   , 0.00001);
   EXPECT_NEAR(0.3,   PE::FUSION::PredictHeading(1,heading,angSpeed).Accuracy, 0.00001);
   //test prediction after 2 second
   EXPECT_NEAR(70.0,  PE::FUSION::PredictHeading(2,heading,angSpeed).Value   , 0.00001);
   EXPECT_NEAR(0.5,   PE::FUSION::PredictHeading(2,heading,angSpeed).Accuracy, 0.00001);
   //test prediction after 10 second
   EXPECT_NEAR(350.0, PE::FUSION::PredictHeading(10,heading,angSpeed).Value   , 0.00001);
   EXPECT_NEAR(2.1,   PE::FUSION::PredictHeading(10,heading,angSpeed).Accuracy, 0.00001);
   //test incorrect heading
   EXPECT_NEAR(90.0, PE::FUSION::PredictHeading(10,PE::SBasicSensor(90.0,PE::MAX_ACCURACY),angSpeed).Value, 0.00001);
   EXPECT_NEAR(0.1,  PE::FUSION::PredictHeading(10,PE::SBasicSensor(PE::MAX_VALUE,0.1),angSpeed).Accuracy,  0.00001);
   //test incorrect angular velocity
   EXPECT_NEAR(90.0, PE::FUSION::PredictHeading(10,heading,PE::SBasicSensor(10.0,PE::MAX_ACCURACY)).Value, 0.00001);
   EXPECT_NEAR(1.1,  PE::FUSION::PredictHeading(10,heading,PE::SBasicSensor(PE::MAX_VALUE,0.2)).Accuracy,  0.00001);
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
 * PredictPosition test invalid inputs
 */
TEST_F(PEFusionToolsTest, test_Predict_Position_invalid_inputs )
{
   PE::TTimestamp invalidDeltaTime (0);         //invalid time delta
   PE::TTimestamp deltaTime ( 2 );              //duration 2 seconds
   PE::SPosition invalidPos;                    //invalid position
   PE::SPosition pos ( 50.0, 10.0, 1.0);        //Lat=50[deg] Lon=10[deg] accuracy +/- 1[m]
   PE::SBasicSensor invalidHeading;             //invalid heading
   PE::SBasicSensor worseHeading ( 90.0,45.0);  //worse heading=90[deg] accuracy +/- 45[deg]
   PE::SBasicSensor heading ( 90.0, 1.0);       //Heading=90[deg] accuracy +/- 1[deg]
   PE::SBasicSensor invalidSpeed;               //invalid speed
   PE::SBasicSensor speed ( 5.0, 1.0);          //Speed=5.0[m/s] accuracy +/-1[m/s]
   PE::SBasicSensor invalidAngSpeed;            //invalid angular speed
   PE::SBasicSensor worseAngSpeed ( 10.0, 22.5);//worse left angular speed =10[deg/s] accuracy +/- 22.5[deg/s]
   PE::SBasicSensor angSpeed ( 10.0, 0.5);      //Left angular speed =10[deg/s] accuracy +/- 0.5[deg/s]

   //invalid deltatime -> same position
   EXPECT_EQ( pos, PE::FUSION::PredictPosition(invalidDeltaTime, heading, angSpeed, pos, speed));

   //invalid position -> invalid 
   EXPECT_FALSE( PE::FUSION::PredictPosition(deltaTime, heading, angSpeed, invalidPos, speed).IsValid());

   //invalid speed -> same position but 3 times worse accuracy after 3 second
   EXPECT_EQ( 50.0, PE::FUSION::PredictPosition(deltaTime, heading, angSpeed, pos, invalidSpeed).Latitude);
   EXPECT_EQ( 10.0, PE::FUSION::PredictPosition(deltaTime, heading, angSpeed, pos, invalidSpeed).Longitude);
   EXPECT_EQ(  3.0, PE::FUSION::PredictPosition(deltaTime, heading, angSpeed, pos, invalidSpeed).HorizontalAcc);

   //invalid heading -> same position but very worse accuracy
   EXPECT_EQ( 50.0, PE::FUSION::PredictPosition(deltaTime, invalidHeading, angSpeed, pos, speed).Latitude);
   EXPECT_EQ( 10.0, PE::FUSION::PredictPosition(deltaTime, invalidHeading, angSpeed, pos, speed).Longitude);
   EXPECT_EQ( 13.0, PE::FUSION::PredictPosition(deltaTime, invalidHeading, angSpeed, pos, speed).HorizontalAcc);

   //invalid angular velocity -> same position but very worse accuracy
   EXPECT_EQ( 50.0, PE::FUSION::PredictPosition(deltaTime, heading, invalidAngSpeed, pos, speed).Latitude);
   EXPECT_EQ( 10.0, PE::FUSION::PredictPosition(deltaTime, heading, invalidAngSpeed, pos, speed).Longitude);
   EXPECT_EQ( 13.0, PE::FUSION::PredictPosition(deltaTime, heading, invalidAngSpeed, pos, speed).HorizontalAcc);

   //too worse fused angle accuracy -> same position but very worse accuracy
   EXPECT_EQ( 50.0, PE::FUSION::PredictPosition(deltaTime, worseHeading, worseAngSpeed, pos, speed).Latitude);
   EXPECT_EQ( 10.0, PE::FUSION::PredictPosition(deltaTime, worseHeading, worseAngSpeed, pos, speed).Longitude);
   EXPECT_EQ( 13.0, PE::FUSION::PredictPosition(deltaTime, worseHeading, worseAngSpeed, pos, speed).HorizontalAcc);

   //all value are valid moving around
   EXPECT_NEAR( 50.0000039, PE::FUSION::PredictPosition(1, heading, angSpeed, pos, speed).Latitude, 0.0000001);
   EXPECT_NEAR( 10.0000696, PE::FUSION::PredictPosition(1, heading, angSpeed, pos, speed).Longitude, 0.0000001);
   EXPECT_NEAR(  2.0006855, PE::FUSION::PredictPosition(1, heading, angSpeed, pos, speed).HorizontalAcc, 0.0000001);

   EXPECT_NEAR( 50.0000155, PE::FUSION::PredictPosition(2, heading, angSpeed, pos, speed).Latitude, 0.0000001);
   EXPECT_NEAR( 10.0001370, PE::FUSION::PredictPosition(2, heading, angSpeed, pos, speed).Longitude, 0.0000001);
   EXPECT_NEAR(  3.0018286, PE::FUSION::PredictPosition(2, heading, angSpeed, pos, speed).HorizontalAcc, 0.0000001);

   EXPECT_NEAR( 50.0000345, PE::FUSION::PredictPosition(3, heading, angSpeed, pos, speed).Latitude, 0.0000001);
   EXPECT_NEAR( 10.0002004, PE::FUSION::PredictPosition(3, heading, angSpeed, pos, speed).Longitude, 0.0000001);
   EXPECT_NEAR(  4.0038107, PE::FUSION::PredictPosition(3, heading, angSpeed, pos, speed).HorizontalAcc, 0.0000001);

   EXPECT_NEAR( 50.0000602, PE::FUSION::PredictPosition(4, heading, angSpeed, pos, speed).Latitude, 0.0000001);
   EXPECT_NEAR( 10.0002576, PE::FUSION::PredictPosition(4, heading, angSpeed, pos, speed).Longitude, 0.0000001);
   EXPECT_NEAR(  5.0068617, PE::FUSION::PredictPosition(4, heading, angSpeed, pos, speed).HorizontalAcc, 0.0000001);

   EXPECT_NEAR( 50.0000920, PE::FUSION::PredictPosition(5, heading, angSpeed, pos, speed).Latitude, 0.0000001);
   EXPECT_NEAR( 10.0003070, PE::FUSION::PredictPosition(5, heading, angSpeed, pos, speed).Longitude, 0.0000001);
   EXPECT_NEAR(  6.0112121, PE::FUSION::PredictPosition(5, heading, angSpeed, pos, speed).HorizontalAcc, 0.0000001);

   EXPECT_NEAR( 50.0001288, PE::FUSION::PredictPosition(6, heading, angSpeed, pos, speed).Latitude, 0.0000001);
   EXPECT_NEAR( 10.0003471, PE::FUSION::PredictPosition(6, heading, angSpeed, pos, speed).Longitude, 0.0000001);
   EXPECT_NEAR(  7.0170932, PE::FUSION::PredictPosition(6, heading, angSpeed, pos, speed).HorizontalAcc, 0.0000001);

   EXPECT_NEAR( 50.0001695, PE::FUSION::PredictPosition(7, heading, angSpeed, pos, speed).Latitude, 0.0000001);
   EXPECT_NEAR( 10.0003766, PE::FUSION::PredictPosition(7, heading, angSpeed, pos, speed).Longitude, 0.0000001);
   EXPECT_NEAR(  8.0247375, PE::FUSION::PredictPosition(7, heading, angSpeed, pos, speed).HorizontalAcc, 0.0000001);

   EXPECT_NEAR( 50.0002128, PE::FUSION::PredictPosition(8, heading, angSpeed, pos, speed).Latitude, 0.0000001);
   EXPECT_NEAR( 10.0003947, PE::FUSION::PredictPosition(8, heading, angSpeed, pos, speed).Longitude, 0.0000001);
   EXPECT_NEAR(  9.0343785, PE::FUSION::PredictPosition(8, heading, angSpeed, pos, speed).HorizontalAcc, 0.0000001);

   EXPECT_NEAR( 50.0002576, PE::FUSION::PredictPosition(9, heading, angSpeed, pos, speed).Latitude, 0.0000001);
   EXPECT_NEAR( 10.0004008, PE::FUSION::PredictPosition(9, heading, angSpeed, pos, speed).Longitude, 0.0000001);
   EXPECT_NEAR( 10.0462509, PE::FUSION::PredictPosition(9, heading, angSpeed, pos, speed).HorizontalAcc, 0.0000001);
}

/**
 * PredictPosition turning left for 180 degree
 */
TEST_F(PEFusionToolsTest, test_Predict_Position_and_Heading_head_90_turning_left_10_deg_per_sec_duration_18_sec )
{
   PE::TTimestamp  deltaTime ( 18 );               //duration 18 seconds
   PE::SPosition         pos (  50.0, 10.0, 1.0);  //Lat=50[deg] Lon=10[deg] accuracy +/- 1[m]
   PE::SBasicSensor  heading (  90.0, 1.0);        //Heading=90[deg] accuracy +/- 1[deg]
   PE::SBasicSensor    speed (   5.0, 1.0);        //Speed=5.0[m/s] accuracy +/-1[m/s]
   PE::SBasicSensor angSpeed (  10.0, 0.5);        //Left angular speed =10[deg/s] accuracy +/- 0.5[deg/s]

   const PE::SPosition pos_predict = PE::FUSION::PredictPosition(deltaTime, heading, angSpeed, pos, speed);
   EXPECT_NEAR( 50.000515,pos_predict.Latitude,0.000001);
   EXPECT_NEAR( 10.000000,pos_predict.Longitude,0.000001);
   EXPECT_NEAR( 19.293105,pos_predict.HorizontalAcc,0.000001);
   //check distance
   EXPECT_NEAR( PE::TOOLS::ToDistance(pos, pos_predict), speed.Value * deltaTime * 2 / PE::PI,0.000001);

   const PE::SBasicSensor heading_predict = PE::FUSION::PredictHeading (deltaTime, heading, angSpeed);
   EXPECT_NEAR(270.000000,heading_predict.Value,0.000001);
   EXPECT_NEAR( 10.000000,heading_predict.Accuracy,0.000001);
}

/**
 * PredictPosition turning right for 180 degree
 */
TEST_F(PEFusionToolsTest, test_Predict_Position_and_Heading_head_90_turning_right_10_deg_per_sec_duration_18_sec )
{
   PE::TTimestamp  deltaTime ( 18 );               //duration 18 seconds
   PE::SPosition         pos (  50.0, 10.0, 1.0);  //Lat=50[deg] Lon=10[deg] accuracy +/- 1[m]
   PE::SBasicSensor  heading (  90.0, 1.0);        //Heading=90[deg] accuracy +/- 1[deg]
   PE::SBasicSensor    speed (   5.0, 1.0);        //Speed=5.0[m/s] accuracy +/-1[m/s]
   PE::SBasicSensor angSpeed ( -10.0, 0.5);        //Right angular speed =-10[deg/s] accuracy +/- 0.5[deg/s]

   const PE::SPosition pos_predict = PE::FUSION::PredictPosition(deltaTime, heading, angSpeed, pos, speed);
   EXPECT_NEAR( 49.999485,pos_predict.Latitude,0.000001);
   EXPECT_NEAR( 10.000000,pos_predict.Longitude,0.000001);
   EXPECT_NEAR( 19.293105,pos_predict.HorizontalAcc,0.000001);
   //check distance
   EXPECT_NEAR( PE::TOOLS::ToDistance(pos, pos_predict), speed.Value * deltaTime * 2 / PE::PI,0.000001);

   const PE::SBasicSensor heading_predict = PE::FUSION::PredictHeading (deltaTime, heading, angSpeed);
   EXPECT_NEAR(270.000000,heading_predict.Value,0.000001);
   EXPECT_NEAR( 10.000000,heading_predict.Accuracy,0.000001);
}

/**
 * PredictPosition turning left for 360 degree
 */
TEST_F(PEFusionToolsTest, test_Predict_Position_and_Heading_head_90_turning_left_10_deg_per_sec_duration_36_sec )
{
   PE::TTimestamp  deltaTime ( 36 );               //duration 36 seconds
   PE::SPosition         pos (  50.0, 10.0, 1.0);  //Lat=50[deg] Lon=10[deg] accuracy +/- 1[m]
   PE::SBasicSensor  heading (  90.0, 1.0);        //Heading=90[deg] accuracy +/- 1[deg]
   PE::SBasicSensor    speed (   5.0, 1.0);        //Speed=5.0[m/s] accuracy +/-1[m/s]
   PE::SBasicSensor angSpeed (  10.0, 0.5);        //Left angular speed =10[deg/s] accuracy +/- 0.5[deg/s]

   const PE::SPosition pos_predict = PE::FUSION::PredictPosition(deltaTime, heading, angSpeed, pos, speed);
   EXPECT_NEAR( 50.000000,pos_predict.Latitude,0.000001);
   EXPECT_NEAR( 10.000000,pos_predict.Longitude,0.000001);
   EXPECT_NEAR( 39.131965,pos_predict.HorizontalAcc,0.000001);
   //check distance has to be zero (passed full circle )
   EXPECT_NEAR( 0, PE::TOOLS::ToDistance(pos, pos_predict),0.000001);

   const PE::SBasicSensor heading_predict = PE::FUSION::PredictHeading (deltaTime, heading, angSpeed);
   EXPECT_NEAR( 90.000000,heading_predict.Value,0.000001);
   EXPECT_NEAR( 19.000000,heading_predict.Accuracy,0.000001);
}

/**
 * PredictPosition turning right for 360 degree
 */
TEST_F(PEFusionToolsTest, test_Predict_Position_and_Heading_head_90_turning_right_10_deg_per_sec_duration_36_sec )
{
   PE::TTimestamp  deltaTime ( 36 );               //duration 36 seconds
   PE::SPosition         pos (  50.0, 10.0, 1.0);  //Lat=50[deg] Lon=10[deg] accuracy +/- 1[m]
   PE::SBasicSensor  heading (  90.0, 1.0);        //Heading=90[deg] accuracy +/- 1[deg]
   PE::SBasicSensor    speed (   5.0, 1.0);        //Speed=5.0[m/s] accuracy +/-1[m/s]
   PE::SBasicSensor angSpeed ( -10.0, 0.5);        //Right angular speed =-10[deg/s] accuracy +/- 0.5[deg/s]

   const PE::SPosition pos_predict = PE::FUSION::PredictPosition(deltaTime, heading, angSpeed, pos, speed);
   EXPECT_NEAR( 50.000000,pos_predict.Latitude,0.000001);
   EXPECT_NEAR( 10.000000,pos_predict.Longitude,0.000001);
   EXPECT_NEAR( 39.131965,pos_predict.HorizontalAcc,0.000001);
   //check distance has to be zero (passed full circle )
   EXPECT_NEAR( 0, PE::TOOLS::ToDistance(pos, pos_predict),0.000001);

   const PE::SBasicSensor heading_predict = PE::FUSION::PredictHeading (deltaTime, heading, angSpeed);
   EXPECT_NEAR( 90.000000,heading_predict.Value,0.000001);
   EXPECT_NEAR( 19.000000,heading_predict.Accuracy,0.000001);
}

/**
 * PredictPosition straight driving 5 [m/s]
 */
TEST_F(PEFusionToolsTest, test_Predict_Position_straight_driving_5_m_per_sec_duration_10sec )
{
   PE::TTimestamp  deltaTime ( 10 );               //duration 10 seconds
   PE::SPosition         pos (  50.0, 10.0, 1.0);  //Lat=50[deg] Lon=10[deg] accuracy +/- 1[m]
   PE::SBasicSensor  heading000 (   0.0, 1.0);        //Heading=0[deg] accuracy +/- 1[deg]
   PE::SBasicSensor  heading090 (  90.0, 1.0);        //Heading=90[deg] accuracy +/- 1[deg]
   PE::SBasicSensor  heading180 ( 180.0, 1.0);        //Heading=180[deg] accuracy +/- 1[deg]
   PE::SBasicSensor  heading270 ( 270.0, 1.0);        //Heading=270[deg] accuracy +/- 1[deg]
   PE::SBasicSensor    speed (   5.0, 1.0);        //Speed=5.0[m/s] accuracy +/-1[m/s]
   PE::SBasicSensor angSpeed (   0.0, 0.5);        //Angular speed =0[deg/s] accuracy +/- 0.5[deg/s]

   //Head 0
   EXPECT_NEAR( 50.000449,PE::FUSION::PredictPosition(deltaTime, heading000, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR( 10.000000,PE::FUSION::PredictPosition(deltaTime, heading000, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR( 11.060591,PE::FUSION::PredictPosition(deltaTime, heading000, angSpeed, pos, speed).HorizontalAcc,0.000001);
   //Head 90
   EXPECT_NEAR( 50.000000,PE::FUSION::PredictPosition(deltaTime, heading090, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR( 10.000699,PE::FUSION::PredictPosition(deltaTime, heading090, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR( 11.060591,PE::FUSION::PredictPosition(deltaTime, heading090, angSpeed, pos, speed).HorizontalAcc,0.000001);
   //Head 180
   EXPECT_NEAR( 49.999551,PE::FUSION::PredictPosition(deltaTime, heading180, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR( 10.000000,PE::FUSION::PredictPosition(deltaTime, heading180, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR( 11.060591,PE::FUSION::PredictPosition(deltaTime, heading180, angSpeed, pos, speed).HorizontalAcc,0.000001);
   //Head 270
   EXPECT_NEAR( 50.000000,PE::FUSION::PredictPosition(deltaTime, heading270, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR(  9.999301,PE::FUSION::PredictPosition(deltaTime, heading270, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR( 11.060591,PE::FUSION::PredictPosition(deltaTime, heading270, angSpeed, pos, speed).HorizontalAcc,0.000001);
}

/**
 * PredictPosition straight driving 5 [m/s] and crossing Grinvitch meridian
 */
TEST_F(PEFusionToolsTest, test_Predict_Position_crossing_Grinvitch_meridian )
{
   PE::TTimestamp  deltaTime ( 10 );               //duration 10 seconds
   PE::SPosition         pos (  50.0, 0.0, 1.0);  //Lat=50[deg] Lon=0[deg] accuracy +/- 1[m]
   PE::SBasicSensor  heading000 (   0.0, 1.0);        //Heading=0[deg] accuracy +/- 1[deg]
   PE::SBasicSensor  heading090 (  90.0, 1.0);        //Heading=90[deg] accuracy +/- 1[deg]
   PE::SBasicSensor  heading180 ( 180.0, 1.0);        //Heading=180[deg] accuracy +/- 1[deg]
   PE::SBasicSensor  heading270 ( 270.0, 1.0);        //Heading=270[deg] accuracy +/- 1[deg]
   PE::SBasicSensor    speed (   5.0, 1.0);        //Speed=5.0[m/s] accuracy +/-1[m/s]
   PE::SBasicSensor angSpeed (   0.0, 0.5);        //Angular speed =0[deg/s] accuracy +/- 0.5[deg/s]

   //Head 0
   EXPECT_NEAR( 50.000449,PE::FUSION::PredictPosition(deltaTime, heading000, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR(  0.000000,PE::FUSION::PredictPosition(deltaTime, heading000, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR( 11.060591,PE::FUSION::PredictPosition(deltaTime, heading000, angSpeed, pos, speed).HorizontalAcc,0.000001);
   //Head 90
   EXPECT_NEAR( 50.000000,PE::FUSION::PredictPosition(deltaTime, heading090, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR(  0.000699,PE::FUSION::PredictPosition(deltaTime, heading090, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR( 11.060591,PE::FUSION::PredictPosition(deltaTime, heading090, angSpeed, pos, speed).HorizontalAcc,0.000001);
   //Head 180
   EXPECT_NEAR( 49.999551,PE::FUSION::PredictPosition(deltaTime, heading180, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR(  0.000000,PE::FUSION::PredictPosition(deltaTime, heading180, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR( 11.060591,PE::FUSION::PredictPosition(deltaTime, heading180, angSpeed, pos, speed).HorizontalAcc,0.000001);
   //Head 270
   EXPECT_NEAR( 50.000000,PE::FUSION::PredictPosition(deltaTime, heading270, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR( -0.000699,PE::FUSION::PredictPosition(deltaTime, heading270, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR( 11.060591,PE::FUSION::PredictPosition(deltaTime, heading270, angSpeed, pos, speed).HorizontalAcc,0.000001);
   //EXPECT_FALSE(true);
}

/**
 * PredictPosition straight driving 5 [m/s] and crossing Anti meridian
 */
TEST_F(PEFusionToolsTest, test_Predict_Position_crossing_Anti_meridian )
{
   PE::TTimestamp  deltaTime ( 10 );               //duration 10 seconds
   PE::SPosition         pos (  50.0, 180.0, 1.0);  //Lat=50[deg] Lon=180[deg] accuracy +/- 1[m]
   PE::SBasicSensor  heading000 (   0.0, 1.0);        //Heading=0[deg] accuracy +/- 1[deg]
   PE::SBasicSensor  heading090 (  90.0, 1.0);        //Heading=90[deg] accuracy +/- 1[deg]
   PE::SBasicSensor  heading180 ( 180.0, 1.0);        //Heading=180[deg] accuracy +/- 1[deg]
   PE::SBasicSensor  heading270 ( 270.0, 1.0);        //Heading=270[deg] accuracy +/- 1[deg]
   PE::SBasicSensor    speed (   5.0, 1.0);        //Speed=5.0[m/s] accuracy +/-1[m/s]
   PE::SBasicSensor angSpeed (   0.0, 0.5);        //Angular speed =0[deg/s] accuracy +/- 0.5[deg/s]

   //Head 0
   EXPECT_NEAR(  50.000449,PE::FUSION::PredictPosition(deltaTime, heading000, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR(-180.000000,PE::FUSION::PredictPosition(deltaTime, heading000, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR(  11.060591,PE::FUSION::PredictPosition(deltaTime, heading000, angSpeed, pos, speed).HorizontalAcc,0.000001);
   //Head 90
   EXPECT_NEAR(  50.000000,PE::FUSION::PredictPosition(deltaTime, heading090, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR(-179.999301,PE::FUSION::PredictPosition(deltaTime, heading090, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR(  11.060591,PE::FUSION::PredictPosition(deltaTime, heading090, angSpeed, pos, speed).HorizontalAcc,0.000001);
   //Head 180
   EXPECT_NEAR(  49.999551,PE::FUSION::PredictPosition(deltaTime, heading180, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR(-180.000000,PE::FUSION::PredictPosition(deltaTime, heading180, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR(  11.060591,PE::FUSION::PredictPosition(deltaTime, heading180, angSpeed, pos, speed).HorizontalAcc,0.000001);
   //Head 270
   EXPECT_NEAR(  50.000000,PE::FUSION::PredictPosition(deltaTime, heading270, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR( 179.999301,PE::FUSION::PredictPosition(deltaTime, heading270, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR(  11.060591,PE::FUSION::PredictPosition(deltaTime, heading270, angSpeed, pos, speed).HorizontalAcc,0.000001);
}

/**
 * PredictPosition straight driving 5 [m/s] and crossing Equator
 */
TEST_F(PEFusionToolsTest, test_Predict_Position_crossing_Equator )
{
   PE::TTimestamp  deltaTime ( 10 );               //duration 10 seconds
   PE::SPosition         pos ( 0.0, 10.0, 1.0);  //Lat=0[deg] Lon=10[deg] accuracy +/- 1[m]
   PE::SBasicSensor  heading000 (   0.0, 1.0);        //Heading=0[deg] accuracy +/- 1[deg]
   PE::SBasicSensor  heading090 (  90.0, 1.0);        //Heading=90[deg] accuracy +/- 1[deg]
   PE::SBasicSensor  heading180 ( 180.0, 1.0);        //Heading=180[deg] accuracy +/- 1[deg]
   PE::SBasicSensor  heading270 ( 270.0, 1.0);        //Heading=270[deg] accuracy +/- 1[deg]
   PE::SBasicSensor    speed (   5.0, 1.0);        //Speed=5.0[m/s] accuracy +/-1[m/s]
   PE::SBasicSensor angSpeed (   0.0, 0.5);        //Angular speed =0[deg/s] accuracy +/- 0.5[deg/s]

   //Head 0
   EXPECT_NEAR(  0.000449,PE::FUSION::PredictPosition(deltaTime, heading000, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR( 10.000000,PE::FUSION::PredictPosition(deltaTime, heading000, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR( 11.060591,PE::FUSION::PredictPosition(deltaTime, heading000, angSpeed, pos, speed).HorizontalAcc,0.000001);
   //Head 90
   EXPECT_NEAR(  0.000000,PE::FUSION::PredictPosition(deltaTime, heading090, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR( 10.000449,PE::FUSION::PredictPosition(deltaTime, heading090, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR( 11.060591,PE::FUSION::PredictPosition(deltaTime, heading090, angSpeed, pos, speed).HorizontalAcc,0.000001);
   //Head 180
   EXPECT_NEAR( -0.000449,PE::FUSION::PredictPosition(deltaTime, heading180, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR( 10.000000,PE::FUSION::PredictPosition(deltaTime, heading180, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR( 11.060591,PE::FUSION::PredictPosition(deltaTime, heading180, angSpeed, pos, speed).HorizontalAcc,0.000001);
   //Head 270
   EXPECT_NEAR(  0.000000,PE::FUSION::PredictPosition(deltaTime, heading270, angSpeed, pos, speed).Latitude,0.000001);
   EXPECT_NEAR(  9.999551,PE::FUSION::PredictPosition(deltaTime, heading270, angSpeed, pos, speed).Longitude,0.000001);
   EXPECT_NEAR( 11.060591,PE::FUSION::PredictPosition(deltaTime, heading270, angSpeed, pos, speed).HorizontalAcc,0.000001);
}

/**
 * PredictSpeed test invalid input
 */
TEST_F(PEFusionToolsTest, test_PredictSpeed_invalid_input )
{
   PE::TTimestamp invalidDeltaTime (0);          //invalid time delta
   PE::TTimestamp deltaTime ( 2 );               //duration 2 seconds
   PE::SPosition invalidPos;                     //invalid position
   PE::SPosition pos     ( 50.00000000, 10.00000000, 1.0); //Lat=50[deg] Lon=10[deg] accuracy +/- 1[m]
   //PE::SPosition pos4sec ( 50.000060275243563, 10.000257636985225, 1.0);
   PE::SPosition pos4sec ( 50.00006027, 10.00025763, 1.0); //mm precision
   PE::SBasicSensor invalidAngSpeed;             //invalid angular speed
   PE::SBasicSensor worseAngSpeed ( 10.0, 11.25);//worse left angular speed =10[deg/s] accuracy +/- 11.25[deg/s] after 4 second reach accuracy limit(+/-45deg)
   PE::SBasicSensor bigAngSpeed ( 45.0, 0.5);    //angular velocity more then 180 left angular speed =45.0[deg/s] accuracy +/- 0.5[deg/s]
   PE::SBasicSensor bedAngSpeed ( 10.0, 11.24);  //bed angular velocity left angular speed =10.0[deg/s] accuracy +/- 11.24[deg/s] after 4 seconds very close to accuracy limit(+/-45deg)
   PE::SBasicSensor angSpeed ( 10.0, 0.5);       //Left angular speed =10[deg/s] accuracy +/- 0.5[deg/s]

   //invalid delta time
   EXPECT_FALSE(PE::FUSION::PredictSpeed(invalidDeltaTime,pos,pos4sec,angSpeed).IsValid());

   //invalid first position
   EXPECT_FALSE(PE::FUSION::PredictSpeed(deltaTime,invalidPos,pos4sec,angSpeed).IsValid());

   //invalid second position
   EXPECT_FALSE(PE::FUSION::PredictSpeed(deltaTime,pos,invalidPos,angSpeed).IsValid());

   //same position
   EXPECT_TRUE(     PE::FUSION::PredictSpeed(deltaTime,pos,pos,angSpeed).IsValid());
   EXPECT_NEAR(0.0, PE::FUSION::PredictSpeed(deltaTime,pos,pos,angSpeed).Value, 0.001);
   EXPECT_NEAR(2.828, PE::FUSION::PredictSpeed(deltaTime,pos,pos,angSpeed).Accuracy, 0.001);

   //invalid angular velocity
   EXPECT_TRUE(       PE::FUSION::PredictSpeed(4,pos,pos4sec,invalidAngSpeed).IsValid());
   EXPECT_NEAR(4.899, PE::FUSION::PredictSpeed(4,pos,pos4sec,invalidAngSpeed).Value, 0.001);
   EXPECT_NEAR(2.828, PE::FUSION::PredictSpeed(4,pos,pos4sec,invalidAngSpeed).Accuracy, 0.001);

   //too worse anglular velocity accuracy
   EXPECT_TRUE(       PE::FUSION::PredictSpeed(4,pos,pos4sec,worseAngSpeed).IsValid());
   EXPECT_NEAR(4.899, PE::FUSION::PredictSpeed(4,pos,pos4sec,worseAngSpeed).Value, 0.001);
   EXPECT_NEAR(2.828, PE::FUSION::PredictSpeed(4,pos,pos4sec,worseAngSpeed).Accuracy, 0.001);

   //too big anglular velocity
   EXPECT_TRUE(       PE::FUSION::PredictSpeed(4,pos,pos4sec,bigAngSpeed).IsValid());
   EXPECT_NEAR(4.899, PE::FUSION::PredictSpeed(4,pos,pos4sec,bigAngSpeed).Value, 0.001);
   EXPECT_NEAR(2.828, PE::FUSION::PredictSpeed(4,pos,pos4sec,bigAngSpeed).Accuracy, 0.001);

   //too bed anglular velocity
   EXPECT_TRUE(        PE::FUSION::PredictSpeed(4,pos,pos4sec,bedAngSpeed).IsValid());
   EXPECT_NEAR(5.0,    PE::FUSION::PredictSpeed(4,pos,pos4sec,bedAngSpeed).Value, 0.001);
   EXPECT_NEAR(2.826, PE::FUSION::PredictSpeed(4,pos,pos4sec,bedAngSpeed).Accuracy, 0.001);

   //all is valid and speed is 5[m/s]
   EXPECT_TRUE(       PE::FUSION::PredictSpeed(4,pos,pos4sec,angSpeed).IsValid());
   EXPECT_NEAR(5.0,   PE::FUSION::PredictSpeed(4,pos,pos4sec,angSpeed).Value, 0.001);
   EXPECT_NEAR(2.001, PE::FUSION::PredictSpeed(4,pos,pos4sec,angSpeed).Accuracy, 0.001);
}

/**
 * PredictSpeed speed with different accuracy of angular velocity
 */
TEST_F(PEFusionToolsTest, test_Predict_Speed_different_ang_accuracy_duration_4_sec )
{
   PE::TTimestamp deltaTime ( 4 );                       //duration 4 seconds
   PE::SPosition pos      ( 50.000000, 10.000000, 1.0);  //Lat=50[deg] Lon=10[deg] accuracy +/- 1[m]
   PE::SPosition pos4sec ( 50.00006027, 10.00025763, 1.0); //mm precision

   PE::SBasicSensor angSpeed_acc01 (  10.0, 0.1);               //Left angular speed =10[deg/s] accuracy +/- 0.1[deg/s]
   PE::SBasicSensor angSpeed_acc05 (  10.0, 0.5);               //Left angular speed =10[deg/s] accuracy +/- 0.5[deg/s]
   PE::SBasicSensor angSpeed_acc10 (  10.0, 1.0);               //Left angular speed =10[deg/s] accuracy +/- 1.0[deg/s]
   PE::SBasicSensor angSpeed_acc50 (  10.0, 5.0);               //Left angular speed =10[deg/s] accuracy +/- 5.0[deg/s]
   PE::SBasicSensor angSpeed_acc99 (  10.0, 9.9);               //Left angular speed =10[deg/s] accuracy +/- 9.9[deg/s]

   EXPECT_NEAR( 5.00,   PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc01).Value, 0.01);
   EXPECT_NEAR( 2.0000, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc01).Accuracy, 0.0001);
   EXPECT_NEAR( 5.00,   PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc05).Value, 0.01);
   EXPECT_NEAR( 2.0012, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc05).Accuracy, 0.0001);
   EXPECT_NEAR( 5.00,   PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc10).Value, 0.01);
   EXPECT_NEAR( 2.0048, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc10).Accuracy, 0.0001);
   EXPECT_NEAR( 5.00,   PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc50).Value, 0.01);
   EXPECT_NEAR( 2.1283, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc50).Accuracy, 0.0001);
   EXPECT_NEAR( 5.00,   PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc99).Value, 0.01);
   EXPECT_NEAR( 2.5956, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc99).Accuracy, 0.0001);
}

/**
 * PredictSpeed turning left for 40 degree
 */
TEST_F(PEFusionToolsTest, test_Predict_Speed_turning_left_40_deg )
{
   PE::TTimestamp deltaTime ( 4 );                       //duration 18 seconds
   PE::SPosition  pos     ( 50.000000, 10.000000, 1.0);  //Lat=50[deg] Lon=10[deg] accuracy +/- 1[m]
   PE::SPosition  pos4sec ( 50.00006027, 10.00025763, 1.0); //mm precision
   PE::SBasicSensor angSpeed (  10.0, 0.5);               //Left angular speed =10[deg/s] accuracy +/- 0.5[deg/s]

   EXPECT_TRUE(         PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed).IsValid());
   EXPECT_NEAR( 5.00,   PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed).Value, 0.001);
   EXPECT_NEAR( 2.0012, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed).Accuracy, 0.0001);
}
//straight_driving
/**
 * PredictSpeed turning right for 40 degree
 */
TEST_F(PEFusionToolsTest, test_Predict_Speed_turning_right_40_deg )
{
   PE::TTimestamp deltaTime ( 4 );                       //duration 18 seconds
   PE::SPosition  pos     ( 50.000000, 10.000000, 1.0);  //Lat=50[deg] Lon=10[deg] accuracy +/- 1[m]
   PE::SPosition  pos4sec ( 50.00006027, 10.00025763, 1.0); //mm precision
   PE::SBasicSensor angSpeed ( -10.0, 0.5);               //Rigth angular speed =-10[deg/s] accuracy +/- 0.5[deg/s]

   EXPECT_TRUE(         PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed).IsValid());
   EXPECT_NEAR( 5.00,   PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed).Value, 0.001);
   EXPECT_NEAR( 2.0012, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed).Accuracy, 0.0001);
}

/**
 * PredictSpeed straight driving
 */
TEST_F(PEFusionToolsTest, test_Predict_Speed_turning_straight_driving )
{
   PE::TTimestamp deltaTime ( 4 );                       //duration 18 seconds
   PE::SPosition  pos     ( 50.000000, 10.000000, 1.0);  //Lat/Lon accuracy +/- 1[m]
   PE::SPosition  pos4sec ( 50.001002,  9.997674, 3.0);  //Lat/Lon accuracy +/- 3[m]
   PE::SBasicSensor invalidAngSpeed;            //invalid angular speed
   PE::SBasicSensor angSpeed_acc1125 (  0.0, 11.25);           //angular speed =0[deg/s] accuracy +/- 11.25[deg/s] - after 4 seconds reached accuracy limit - invalid
   PE::SBasicSensor angSpeed_acc01 (  0.0, 0.1);               //angular speed =0[deg/s] accuracy +/- 0.1[deg/s]
   PE::SBasicSensor angSpeed_acc05 (  0.0, 0.5);               //angular speed =0[deg/s] accuracy +/- 0.5[deg/s]
   PE::SBasicSensor angSpeed_acc10 (  0.0, 1.0);               //angular speed =0[deg/s] accuracy +/- 1.0[deg/s]
   PE::SBasicSensor angSpeed_acc50 (  0.0, 5.0);               //angular speed =0[deg/s] accuracy +/- 5.0[deg/s]
   PE::SBasicSensor angSpeed_acc99 (  0.0, 9.9);               //angular speed =0[deg/s] accuracy +/- 9.9[deg/s]
   PE::SBasicSensor angSpeed_acc1124 (  0.0, 11.24);           //angular speed =0[deg/s] accuracy +/- 11.24[deg/s] - after 4 seconds almost reached accuracy limit

   EXPECT_NEAR( 50.0327, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,invalidAngSpeed).Value, 0.001);
   EXPECT_NEAR(  5.6568, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,invalidAngSpeed).Accuracy, 0.0001);

   EXPECT_NEAR( 50.0327, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc1125).Value, 0.001);
   EXPECT_NEAR(  5.6568, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc1125).Accuracy, 0.0001);

   EXPECT_NEAR( 50.0327, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc1124).Value, 0.001);
   EXPECT_NEAR(  5.6529, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc1124).Accuracy, 0.0001);

   EXPECT_NEAR( 50.0327, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc01).Value, 0.001);
   EXPECT_NEAR(  4.0001, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc01).Accuracy, 0.00001);

   EXPECT_NEAR( 50.0327, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc05).Value, 0.001);
   EXPECT_NEAR(  4.0024, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc05).Accuracy, 0.0001);

   EXPECT_NEAR( 50.0327, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc10).Value, 0.001);
   EXPECT_NEAR(  4.0097, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc10).Accuracy, 0.0001);

   EXPECT_NEAR( 50.0327, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc50).Value, 0.001);
   EXPECT_NEAR(  4.2567, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc50).Accuracy, 0.0001);

   EXPECT_NEAR( 50.0327, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc99).Value, 0.001);
   EXPECT_NEAR(  5.1913, PE::FUSION::PredictSpeed(deltaTime,pos,pos4sec,angSpeed_acc99).Accuracy, 0.0001);
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
   EXPECT_FALSE(PE::FUSION::PredictAngSpeed(0,heading_10,heading_45).IsValid());
   //incorrect first heading
   EXPECT_FALSE(PE::FUSION::PredictAngSpeed(1,PE::SBasicSensor(),heading_45).IsValid());
   //incorrect second heading
   EXPECT_FALSE(PE::FUSION::PredictAngSpeed(1,heading_10,PE::SBasicSensor()).IsValid());
   //test same heading
   //--- 10 deg
   EXPECT_NEAR(   0.0, PE::FUSION::PredictAngSpeed(10,heading_10,heading_10).Value,    0.001);
   EXPECT_NEAR( 0.200, PE::FUSION::PredictAngSpeed(10,heading_10,heading_10).Accuracy, 0.001);
   //--- 355 deg
   EXPECT_NEAR(   0.0, PE::FUSION::PredictAngSpeed(1,heading_355,heading_355).Value,    0.001);
   EXPECT_NEAR(   0.4, PE::FUSION::PredictAngSpeed(1,heading_355,heading_355).Accuracy, 0.001);
   //--- 90 deg
   EXPECT_NEAR(   0.0, PE::FUSION::PredictAngSpeed(2,heading_90,heading_90).Value,    0.001);
   EXPECT_NEAR(   1.2, PE::FUSION::PredictAngSpeed(2,heading_90,heading_90).Accuracy, 0.001);
   //--- 270 deg
   EXPECT_NEAR(   0.0, PE::FUSION::PredictAngSpeed(3,heading_270,heading_270).Value,    0.001);
   EXPECT_NEAR(   1.0, PE::FUSION::PredictAngSpeed(3,heading_270,heading_270).Accuracy, 0.001);
   //test prediction after 1 second
   //--- 10 -> 45 deg
   EXPECT_NEAR( -35.0, PE::FUSION::PredictAngSpeed(1,heading_10,heading_45).Value,    0.001);
   EXPECT_NEAR(   0.5, PE::FUSION::PredictAngSpeed(1,heading_10,heading_45).Accuracy, 0.001);
   //--- 45 -> 10 deg
   EXPECT_NEAR(  35.0, PE::FUSION::PredictAngSpeed(1,heading_45,heading_10).Value,    0.001);
   EXPECT_NEAR(   0.5, PE::FUSION::PredictAngSpeed(1,heading_45,heading_10).Accuracy, 0.001);
   //--- 10 -> 355 deg
   EXPECT_NEAR(  15.0, PE::FUSION::PredictAngSpeed(1,heading_10,heading_355).Value,    0.001);
   EXPECT_NEAR(   0.3, PE::FUSION::PredictAngSpeed(1,heading_10,heading_355).Accuracy, 0.001);
   //--- 355 -> 10 deg
   EXPECT_NEAR( -15.0, PE::FUSION::PredictAngSpeed(1,heading_355,heading_10).Value,    0.001);
   EXPECT_NEAR(   0.3, PE::FUSION::PredictAngSpeed(1,heading_355,heading_10).Accuracy, 0.001);
   //--- 90 -> 270 deg
   EXPECT_NEAR(-180.0, PE::FUSION::PredictAngSpeed(1,heading_90,heading_270).Value,    0.001);
   EXPECT_NEAR(   1.1, PE::FUSION::PredictAngSpeed(1,heading_90,heading_270).Accuracy, 0.001);
   //--- 270 -> 90 deg
   EXPECT_NEAR( 180.0, PE::FUSION::PredictAngSpeed(1,heading_270,heading_90).Value,    0.001);
   EXPECT_NEAR(   1.1, PE::FUSION::PredictAngSpeed(1,heading_270,heading_90).Accuracy, 0.001);
   //--- 0 -> 180 deg
   EXPECT_NEAR(-180.0, PE::FUSION::PredictAngSpeed(1,heading_0,heading_180).Value,    0.001);
   EXPECT_NEAR(   0.4, PE::FUSION::PredictAngSpeed(1,heading_0,heading_180).Accuracy, 0.001);
   //--- 180 -> 0 deg
   EXPECT_NEAR( 180.0, PE::FUSION::PredictAngSpeed(1,heading_180,heading_0).Value,    0.001);
   EXPECT_NEAR(   0.4, PE::FUSION::PredictAngSpeed(1,heading_180,heading_0).Accuracy, 0.001);
   //--- 180 -> 270 deg
   EXPECT_NEAR( -90.0, PE::FUSION::PredictAngSpeed(1,heading_180,heading_270).Value,    0.001);
   EXPECT_NEAR(   0.8, PE::FUSION::PredictAngSpeed(1,heading_180,heading_270).Accuracy, 0.001);
   //--- 270 -> 180 deg
   EXPECT_NEAR(  90.0, PE::FUSION::PredictAngSpeed(1,heading_270,heading_180).Value,    0.001);
   EXPECT_NEAR(   0.8, PE::FUSION::PredictAngSpeed(1,heading_270,heading_180).Accuracy, 0.001);
   //--- 355 -> 91 deg
   EXPECT_NEAR( -96.0, PE::FUSION::PredictAngSpeed(1,heading_355,heading_91).Value,    0.001);
   EXPECT_NEAR(   0.9, PE::FUSION::PredictAngSpeed(1,heading_355,heading_91).Accuracy, 0.001);
   //--- 91 -> 355 deg
   EXPECT_NEAR(  96.0, PE::FUSION::PredictAngSpeed(1,heading_91,heading_355).Value,    0.001);
   EXPECT_NEAR(   0.9, PE::FUSION::PredictAngSpeed(1,heading_91,heading_355).Accuracy, 0.001);
   //--- 181 -> 1 deg
   EXPECT_NEAR( 180.0, PE::FUSION::PredictAngSpeed(1,heading_181,heading_1).Value,    0.001);
   EXPECT_NEAR(   1.7, PE::FUSION::PredictAngSpeed(1,heading_181,heading_1).Accuracy, 0.001);
   //--- 1 -> 181 deg
   EXPECT_NEAR(-180.0, PE::FUSION::PredictAngSpeed(1,heading_1,heading_181).Value,    0.001);
   EXPECT_NEAR(   1.7, PE::FUSION::PredictAngSpeed(1,heading_1,heading_181).Accuracy, 0.001);
   //--- 91 -> 181 deg
   EXPECT_NEAR( -90.0, PE::FUSION::PredictAngSpeed(1,heading_91,heading_181).Value,    0.001);
   EXPECT_NEAR(   1.6, PE::FUSION::PredictAngSpeed(1,heading_91,heading_181).Accuracy, 0.001);
   //--- 181 -> 91 deg
   EXPECT_NEAR(  90.0, PE::FUSION::PredictAngSpeed(1,heading_181,heading_91).Value,    0.001);
   EXPECT_NEAR(   1.6, PE::FUSION::PredictAngSpeed(1,heading_181,heading_91).Accuracy, 0.001);
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

