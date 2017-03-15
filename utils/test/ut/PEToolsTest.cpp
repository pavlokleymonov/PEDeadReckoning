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
 * Unit test of the PEToolsTest class.
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
   EXPECT_EQ(PE::PI, PE::TOOLS::to_radians(180.0));
   //EXPECT_NEAR(PE::PI,   PE::TOOLS::to_radians(180.0),0.0001);

   EXPECT_EQ(180.0, PE::TOOLS::to_degrees(PE::PI));
   //EXPECT_NEAR(180.0, PE::TOOLS::to_degrees(PE::PI),  0.0001);

   EXPECT_EQ(180.0, PE::TOOLS::to_degrees(PE::TOOLS::to_radians(180.0)));
   //EXPECT_NEAR(180.0, PE::TOOLS::to_degrees(PE::TOOLS::to_radians(180.0)),  0.0001);

   EXPECT_EQ(PE::PI, PE::TOOLS::to_radians(PE::TOOLS::to_degrees(PE::PI)));
   //EXPECT_NEAR(180.0, PE::TOOLS::to_radians(PE::TOOLS::to_degrees(PE::PI)), 0.0001);

   //check 0<->360 limits
   EXPECT_EQ(2*PE::PI, PE::TOOLS::to_radians(360.0));
   EXPECT_EQ(0.0, PE::TOOLS::to_radians(0.0));
   EXPECT_EQ(360.0, PE::TOOLS::to_degrees(2*PE::PI));
   EXPECT_EQ(0.0, PE::TOOLS::to_degrees(0.0));

   //check values over 360
   EXPECT_EQ(3*PE::PI, PE::TOOLS::to_radians(540.0));
   EXPECT_EQ(540.0, PE::TOOLS::to_degrees(3*PE::PI));

   //check negative values
   EXPECT_EQ(-PE::PI, PE::TOOLS::to_radians(-180.0));
   EXPECT_EQ(-180.0, PE::TOOLS::to_degrees(-PE::PI));
}

//Test distance calc between two coordinates
TEST_F(PEToolsTest, distance_from_coordinates_test)
{
   //small distance 22.681 meter
   EXPECT_NEAR(22.618,PE::TOOLS::to_distance(52.054274,10.008114,52.054447,10.008288),0.001);
   EXPECT_NEAR(22.618,PE::TOOLS::to_distance_precise(52.054447,10.008288,52.054274,10.008114),0.001);

   //same distance but oposite dirrection
   EXPECT_NEAR(22.618,PE::TOOLS::to_distance(52.054447,10.008288,52.054274,10.008114),0.001);
   EXPECT_NEAR(22.618,PE::TOOLS::to_distance_precise(52.054447,10.008288,52.054274,10.008114),0.001);
   
   //middle distance ~6km 
   EXPECT_NEAR(6276.362,PE::TOOLS::to_distance(52.054274,10.008114,52.097195,10.067757),0.001);
   EXPECT_NEAR(6276.362,PE::TOOLS::to_distance_precise(52.054274,10.008114,52.097195,10.067757),0.001);

   //middle distance ~10km 
   EXPECT_NEAR(10403.661,PE::TOOLS::to_distance(52.054274,10.008114,52.109193,10.131375),0.001);
   EXPECT_NEAR(10403.659,PE::TOOLS::to_distance_precise(52.054274,10.008114,52.109193,10.131375),0.001);

   //big distance ~225km 
   EXPECT_NEAR(224941.284,PE::TOOLS::to_distance(52.054274,10.008114,52.671695,13.162785),0.001);
   EXPECT_NEAR(224919.158,PE::TOOLS::to_distance_precise(52.054274,10.008114,52.671695,13.162785),0.001);

   //SOUTH-WEST coordinates
   //small distance
   EXPECT_NEAR(60.704,PE::TOOLS::to_distance(-16.499917,-68.150214,-16.500336,-68.149849),0.001);
   EXPECT_NEAR(60.704,PE::TOOLS::to_distance_precise(-16.499917,-68.150214,-16.500336,-68.149849),0.001);
}

//Test heading calc between two coordinates
TEST_F(PEToolsTest, heading_from_coordinates_test)
{
   //North 0.0
   EXPECT_NEAR(0.0,PE::TOOLS::to_heading( 52.0524,   10.0548 ,  52.0596  ,    10.0548),0.01);
   //East 90.0
   EXPECT_NEAR(90.0,PE::TOOLS::to_heading( 52.0524,   10.0548 ,  52.0524  ,    10.0620),0.01);
   //South 180.0
   EXPECT_NEAR(180.0,PE::TOOLS::to_heading( 52.0596,   10.0548 ,  52.0524  ,    10.0548),0.01);
   //West 270.0
   EXPECT_NEAR(270.0,PE::TOOLS::to_heading( 52.0524,   10.0620 ,  52.0524  ,    10.0548),0.01);
   //PointLU 307.60
   EXPECT_NEAR(307.60,PE::TOOLS::to_heading( 52.045690, 10.002842 , 52.045753 , 10.002709),0.01);
   //PointRD 138.96
   EXPECT_NEAR(138.96,PE::TOOLS::to_heading( 52.045690, 10.002842 , 52.045596 , 10.002975),0.01);
   //SOUTH-WEST coordinates: 140.13
   EXPECT_NEAR(140.13,PE::TOOLS::to_heading(-16.499917,-68.150214,-16.500336,-68.149849),0.01);
}


//Test calc new coordinates based on distance, heading and start coordinates in SOUTH-WEST
TEST_F(PEToolsTest, next_coordinates_SW_test)
{
   //SOUTH-WEST coordinates:
   PE::TValue lat = 0;
   PE::TValue lon = 0;
   PE::TOOLS::get_next_coordinates(-16.499917,-68.150214,60.704,140.13,lat,lon);
   EXPECT_NEAR(-16.500336, lat, 0.000001);
   EXPECT_NEAR(-68.149849, lon, 0.000001);
}

//Test calc new coordinates based on distance, heading and start coordinates in NORTH-EAST
TEST_F(PEToolsTest, next_coordinates_NE_test)
{
   //NORTH-EAST coordinates:
   PE::TValue lat = 0;
   PE::TValue lon = 0;
   PE::TOOLS::get_next_coordinates(52.054274,10.008114,10428.22,54.01,lat,lon);
   EXPECT_NEAR( 52.109320, lat, 0.000001);
   EXPECT_NEAR( 10.131668, lon, 0.000001);
}

//Test calc new coordinates based on distance, heading and start coordinates crossing grinvich
TEST_F(PEToolsTest, next_coordinates_crossing_grinvich_test)
{
   PE::TValue lat = 0;
   PE::TValue lon = 0;
   //cross E->W
   PE::TOOLS::get_next_coordinates(52.123456,0.000123,100,270.0,lat,lon);
   EXPECT_NEAR( 52.123456, lat, 0.000001);
   EXPECT_NEAR( -0.001341, lon, 0.000001);

   //cross W->E
   PE::TOOLS::get_next_coordinates(52.123456,-0.001341,100,90.0,lat,lon);
   EXPECT_NEAR( 52.123456, lat, 0.000001);
   EXPECT_NEAR(  0.000123, lon, 0.000001);
}

//Test calc new coordinates based on distance, heading and start coordinates crossing ecvator
TEST_F(PEToolsTest, next_coordinates_crossing_ecvator_test)
{
   PE::TValue lat = 0;
   PE::TValue lon = 0;
   //cross N->S
   PE::TOOLS::get_next_coordinates(0.000123,10.123456,100,180.0,lat,lon);
   EXPECT_NEAR( -0.000776, lat, 0.000001);
   EXPECT_NEAR( 10.123456, lon, 0.000001);
   //cross S->N
   PE::TOOLS::get_next_coordinates(-0.000776,10.123456,100,0.0,lat,lon);
   EXPECT_NEAR(  0.000123, lat, 0.000001);
   EXPECT_NEAR( 10.123456, lon, 0.000001);
}

TEST_F(PEToolsTest, complex_test)
{
   PE::TValue lat = 0;
   PE::TValue lon = 0;
   PE::TValue distance = PE::TOOLS::to_distance(50.12345,10.12345,50.65432,10.65432);
   PE::TValue heading  = PE::TOOLS::to_heading(50.12345,10.12345,50.65432,10.65432);
   
   PE::TOOLS::get_next_coordinates(50.12345,10.12345,distance,heading,lat,lon);
   EXPECT_NEAR( 50.65432, lat, 0.00001);
   EXPECT_NEAR( 10.65432, lon, 0.00001);
   
   distance = PE::TOOLS::to_distance_precise(50.1234567,5.1234567,50.7654321,-5.1234567);
   heading  = PE::TOOLS::to_heading(50.1234567,5.1234567,50.7654321,-5.1234567);
   PE::TOOLS::get_next_coordinates(50.1234567,5.1234567,distance,heading,lat,lon);
   EXPECT_NEAR( 50.7654321, lat, 0.0000001);
   EXPECT_NEAR( -5.1234567, lon, 0.0000001);
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

