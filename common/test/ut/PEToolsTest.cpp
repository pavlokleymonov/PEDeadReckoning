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
 * Unit test of the PE::TOOLS namespace.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PETools.h"

class PEToolsTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }
};

//Test radian to/from degrees conversions
TEST_F(PEToolsTest, radian_degrees_conversions_test)
{
   //180 <-> PI conversions
   EXPECT_EQ(PE::PI, PE::TOOLS::ToRadians(180.0));
   //EXPECT_NEAR(PE::PI,   PE::TOOLS::to_radians(180.0),0.0001);

   EXPECT_EQ(180.0, PE::TOOLS::ToDegrees(PE::PI));
   //EXPECT_NEAR(180.0, PE::TOOLS::to_degrees(PE::PI),  0.0001);

   EXPECT_EQ(180.0, PE::TOOLS::ToDegrees(PE::TOOLS::ToRadians(180.0)));
   //EXPECT_NEAR(180.0, PE::TOOLS::to_degrees(PE::TOOLS::to_radians(180.0)),  0.0001);

   EXPECT_EQ(PE::PI, PE::TOOLS::ToRadians(PE::TOOLS::ToDegrees(PE::PI)));
   //EXPECT_NEAR(180.0, PE::TOOLS::to_radians(PE::TOOLS::to_degrees(PE::PI)), 0.0001);

   //check 0<->360 limits
   EXPECT_EQ(2*PE::PI, PE::TOOLS::ToRadians(360.0));
   EXPECT_EQ(0.0, PE::TOOLS::ToRadians(0.0));
   EXPECT_EQ(360.0, PE::TOOLS::ToDegrees(2*PE::PI));
   EXPECT_EQ(0.0, PE::TOOLS::ToDegrees(0.0));

   //check values over 360
   EXPECT_EQ(3*PE::PI, PE::TOOLS::ToRadians(540.0));
   EXPECT_EQ(540.0, PE::TOOLS::ToDegrees(3*PE::PI));

   //check negative values
   EXPECT_EQ(-PE::PI, PE::TOOLS::ToRadians(-180.0));
   EXPECT_EQ(-180.0, PE::TOOLS::ToDegrees(-PE::PI));
}

//Test distance calc between two coordinates
TEST_F(PEToolsTest, distance_from_coordinates_test)
{
   //small distance 22.681 meter
   EXPECT_NEAR(22.618,PE::TOOLS::ToDistance(PE::SPosition(52.054274,10.008114),PE::SPosition(52.054447,10.008288)),0.001);
   EXPECT_NEAR(22.618,PE::TOOLS::ToDistancePrecise(PE::SPosition(52.054447,10.008288),PE::SPosition(52.054274,10.008114)),0.001);

   //same distance but oposite dirrection
   EXPECT_NEAR(22.618,PE::TOOLS::ToDistance(PE::SPosition(52.054447,10.008288),PE::SPosition(52.054274,10.008114)),0.001);
   EXPECT_NEAR(22.618,PE::TOOLS::ToDistancePrecise(PE::SPosition(52.054447,10.008288),PE::SPosition(52.054274,10.008114)),0.001);
   
   //middle distance ~6km 
   EXPECT_NEAR(6276.362,PE::TOOLS::ToDistance(PE::SPosition(52.054274,10.008114),PE::SPosition(52.097195,10.067757)),0.001);
   EXPECT_NEAR(6276.362,PE::TOOLS::ToDistancePrecise(PE::SPosition(52.054274,10.008114),PE::SPosition(52.097195,10.067757)),0.001);

   //middle distance ~10km 
   EXPECT_NEAR(10403.661,PE::TOOLS::ToDistance(PE::SPosition(52.054274,10.008114),PE::SPosition(52.109193,10.131375)),0.001);
   EXPECT_NEAR(10403.659,PE::TOOLS::ToDistancePrecise(PE::SPosition(52.054274,10.008114),PE::SPosition(52.109193,10.131375)),0.001);

   //big distance ~225km 
   EXPECT_NEAR(224941.284,PE::TOOLS::ToDistance(PE::SPosition(52.054274,10.008114),PE::SPosition(52.671695,13.162785)),0.001);
   EXPECT_NEAR(224919.158,PE::TOOLS::ToDistancePrecise(PE::SPosition(52.054274,10.008114),PE::SPosition(52.671695,13.162785)),0.001);

   //SOUTH-WEST coordinates
   //small distance
   EXPECT_NEAR(60.704,PE::TOOLS::ToDistance(PE::SPosition(-16.499917,-68.150214),PE::SPosition(-16.500336,-68.149849)),0.001);
   EXPECT_NEAR(60.704,PE::TOOLS::ToDistancePrecise(PE::SPosition(-16.499917,-68.150214),PE::SPosition(-16.500336,-68.149849)),0.001);
}

//Test heading calc between two coordinates
TEST_F(PEToolsTest, heading_from_coordinates_test)
{
   //North 0.0
   EXPECT_NEAR(0.0,PE::TOOLS::ToHeading(PE::SPosition( 52.0524,   10.0548 ),PE::SPosition(  52.0596  ,    10.0548)),0.01);
   //East 90.0
   EXPECT_NEAR(90.0,PE::TOOLS::ToHeading(PE::SPosition( 52.0524,   10.0548 ),PE::SPosition(  52.0524  ,    10.0620)),0.01);
   //South 180.0
   EXPECT_NEAR(180.0,PE::TOOLS::ToHeading(PE::SPosition( 52.0596,   10.0548 ),PE::SPosition(  52.0524  ,    10.0548)),0.01);
   //West 270.0
   EXPECT_NEAR(270.0,PE::TOOLS::ToHeading(PE::SPosition( 52.0524,   10.0620 ),PE::SPosition(  52.0524  ,    10.0548)),0.01);
   //PointLU 307.60
   EXPECT_NEAR(307.60,PE::TOOLS::ToHeading(PE::SPosition( 52.045690, 10.002842 ),PE::SPosition( 52.045753 , 10.002709)),0.01);
   //PointRD 138.96
   EXPECT_NEAR(138.96,PE::TOOLS::ToHeading(PE::SPosition( 52.045690, 10.002842 ),PE::SPosition( 52.045596 , 10.002975)),0.01);
   //SOUTH-WEST coordinates: 140.13
   EXPECT_NEAR(140.13,PE::TOOLS::ToHeading(PE::SPosition(-16.499917,-68.150214),PE::SPosition(-16.500336,-68.149849)),0.01);
}

//Test heading calc based on original heading plus provided angular velocity.
TEST_F(PEToolsTest, heading_from_angular_velocity_test)
{
   const PE::TValue North = 0.0;
   const PE::TValue East  = 90.0;
   const PE::TValue South = 180.0;
   const PE::TValue West  = 270.0;
   //test zero angular velosity
   EXPECT_NEAR(North,PE::TOOLS::ToHeading(North, 1.0, 0.0),0.01);
   //test zero delta time turn left +90.0 deg/s
   EXPECT_NEAR(North,PE::TOOLS::ToHeading(North, 0.0, 90.0),0.01);
   //test left turn +3.0 deg/s jumps over 360 -> 357
   EXPECT_NEAR(357.0,PE::TOOLS::ToHeading(North, 1.0, +3.0),0.01);
   //test jump left from East to West for 2 seconds +90.0 deg/s
   EXPECT_NEAR(West,PE::TOOLS::ToHeading(East, 2.0, +90.0),0.01);
   //test turn right over 360 from North to 3.0
   EXPECT_NEAR(3.0,PE::TOOLS::ToHeading(North, 1.0, -3.0),0.01);
   //test got from South to North by left turn +380.0 for half second
   EXPECT_NEAR(North,PE::TOOLS::ToHeading(South, 0.5, +360.0),0.01);
   //test got from South to North by rigth turn -90.0 for two seconds
   EXPECT_NEAR(North,PE::TOOLS::ToHeading(South, 2.0, -90.0),0.01);
}

//Test calc coordinates in case distance is zero
TEST_F(PEToolsTest, next_coordinates_zero_distance_test)
{
   //East
   PE::SPosition pos1 = PE::TOOLS::ToPosition(PE::SPosition( 50.99999,10.99999),0.0,90.0);
   EXPECT_NEAR(50.99999, pos1.Latitude , 0.000001);
   EXPECT_NEAR(10.99999, pos1.Longitude, 0.000001);
   //South
   PE::SPosition pos2 = PE::TOOLS::ToPosition(PE::SPosition( 50.12345,10.54321),0.0,180.0);
   EXPECT_NEAR(50.12345, pos2.Latitude , 0.000001);
   EXPECT_NEAR(10.54321, pos2.Longitude, 0.000001);
   //West
   PE::SPosition pos3 = PE::TOOLS::ToPosition(PE::SPosition( -50.12345,10.54321),0.0,270.0);
   EXPECT_NEAR(-50.12345, pos3.Latitude , 0.000001);
   EXPECT_NEAR( 10.54321, pos3.Longitude, 0.000001);
   //North
   PE::SPosition pos4 = PE::TOOLS::ToPosition(PE::SPosition( 50.99999,-10.99999),0.0,0.0);
   EXPECT_NEAR( 50.99999, pos4.Latitude , 0.000001);
   EXPECT_NEAR(-10.99999, pos4.Longitude, 0.000001);
   //East lat lon are zero
   PE::SPosition pos5 = PE::TOOLS::ToPosition(PE::SPosition( 0,0),0.0,90.0);
   EXPECT_NEAR(0.0, pos5.Latitude , 0.000001);
   EXPECT_NEAR(0.0, pos5.Longitude, 0.000001);
   //East lat lon are maximum
   PE::SPosition pos6 = PE::TOOLS::ToPosition(PE::SPosition( 89.99999,179.99999),0.0,90.0);
   EXPECT_NEAR(89.99999, pos6.Latitude , 0.000001);
   EXPECT_NEAR(179.99999, pos6.Longitude, 0.000001);
   //East lat lon are minimum
   PE::SPosition pos7 = PE::TOOLS::ToPosition(PE::SPosition( -89.99999,-179.99999),0.0,90.0);
   EXPECT_NEAR(-89.99999, pos7.Latitude , 0.000001);
   EXPECT_NEAR(-179.99999, pos7.Longitude, 0.000001);
}

//Test calc new coordinates based on distance, heading and start coordinates in SOUTH-WEST
TEST_F(PEToolsTest, next_coordinates_SW_test)
{
   //SOUTH-WEST coordinates:
   PE::SPosition pos = PE::TOOLS::ToPosition(PE::SPosition( -16.499917,-68.150214),60.704,140.13);
   
   EXPECT_NEAR(-16.500336, pos.Latitude , 0.000001);
   EXPECT_NEAR(-68.149849, pos.Longitude, 0.000001);
}

//Test calc new coordinates based on distance, heading and start coordinates in NORTH-EAST
TEST_F(PEToolsTest, next_coordinates_NE_test)
{
   //NORTH-EAST coordinates:
   PE::SPosition pos = PE::TOOLS::ToPosition(PE::SPosition( 52.054274,10.008114),10428.22,54.01);
   EXPECT_NEAR( 52.109320, pos.Latitude , 0.000001);
   EXPECT_NEAR( 10.131668, pos.Longitude, 0.000001);
}

//Test calc new coordinates based on distance, heading and start coordinates crossing grinvich
TEST_F(PEToolsTest, next_coordinates_crossing_grinvich_test)
{
   //cross E->W
   PE::SPosition pos1 = PE::TOOLS::ToPosition(PE::SPosition( 52.123456,0.000123),100,270.0);
   EXPECT_NEAR( 52.123456, pos1.Latitude , 0.000001);
   EXPECT_NEAR( -0.001341, pos1.Longitude, 0.000001);

   //cross W->E
   PE::SPosition pos2 = PE::TOOLS::ToPosition(PE::SPosition( 52.123456,-0.001341),100,90.0);
   EXPECT_NEAR( 52.123456, pos2.Latitude , 0.000001);
   EXPECT_NEAR(  0.000123, pos2.Longitude, 0.000001);
}

//Test calc new coordinates based on distance, heading and start coordinates crossing ecvator
TEST_F(PEToolsTest, next_coordinates_crossing_ecvator_test)
{
   //cross N->S
   PE::SPosition pos1 = PE::TOOLS::ToPosition(PE::SPosition( 0.000123,10.123456),100,180.0);
   EXPECT_NEAR( -0.000776, pos1.Latitude , 0.000001);
   EXPECT_NEAR( 10.123456, pos1.Longitude, 0.000001);
   //cross S->N
   PE::SPosition pos2 = PE::TOOLS::ToPosition(PE::SPosition( -0.000776,10.123456),100,0.0);
   EXPECT_NEAR(  0.000123, pos2.Latitude , 0.000001);
   EXPECT_NEAR( 10.123456, pos2.Longitude, 0.000001);
}

TEST_F(PEToolsTest, complex_test)
{
   PE::TValue distance1 = PE::TOOLS::ToDistance(PE::SPosition(50.12345,10.12345),PE::SPosition(50.65432,10.65432));
   PE::TValue heading1  = PE::TOOLS::ToHeading(PE::SPosition(50.12345,10.12345),PE::SPosition(50.65432,10.65432));
   
   PE::SPosition pos1 = PE::TOOLS::ToPosition(PE::SPosition( 50.12345,10.12345),distance1,heading1);
   EXPECT_NEAR( 50.65432, pos1.Latitude , 0.00001);
   EXPECT_NEAR( 10.65432, pos1.Longitude, 0.00001);
   
   PE::TValue distance2 = PE::TOOLS::ToDistancePrecise(PE::SPosition(50.1234567,5.1234567),PE::SPosition(50.7654321,-5.1234567));
   PE::TValue heading2  = PE::TOOLS::ToHeading(PE::SPosition(50.1234567,5.1234567),PE::SPosition(50.7654321,-5.1234567));
   PE::SPosition pos2 = PE::TOOLS::ToPosition(PE::SPosition( 50.1234567,5.1234567),distance2,heading2);
   EXPECT_NEAR( 50.7654321, pos2.Latitude , 0.0000001);
   EXPECT_NEAR( -5.1234567, pos2.Longitude, 0.0000001);
}

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

