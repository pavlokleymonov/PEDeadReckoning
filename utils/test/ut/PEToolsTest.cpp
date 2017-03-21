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
   EXPECT_NEAR(22.618,PE::TOOLS::to_distance(PE::TPosition(52.054274,10.008114),PE::TPosition(52.054447,10.008288)),0.001);
   EXPECT_NEAR(22.618,PE::TOOLS::to_distance_precise(PE::TPosition(52.054447,10.008288),PE::TPosition(52.054274,10.008114)),0.001);

   //same distance but oposite dirrection
   EXPECT_NEAR(22.618,PE::TOOLS::to_distance(PE::TPosition(52.054447,10.008288),PE::TPosition(52.054274,10.008114)),0.001);
   EXPECT_NEAR(22.618,PE::TOOLS::to_distance_precise(PE::TPosition(52.054447,10.008288),PE::TPosition(52.054274,10.008114)),0.001);
   
   //middle distance ~6km 
   EXPECT_NEAR(6276.362,PE::TOOLS::to_distance(PE::TPosition(52.054274,10.008114),PE::TPosition(52.097195,10.067757)),0.001);
   EXPECT_NEAR(6276.362,PE::TOOLS::to_distance_precise(PE::TPosition(52.054274,10.008114),PE::TPosition(52.097195,10.067757)),0.001);

   //middle distance ~10km 
   EXPECT_NEAR(10403.661,PE::TOOLS::to_distance(PE::TPosition(52.054274,10.008114),PE::TPosition(52.109193,10.131375)),0.001);
   EXPECT_NEAR(10403.659,PE::TOOLS::to_distance_precise(PE::TPosition(52.054274,10.008114),PE::TPosition(52.109193,10.131375)),0.001);

   //big distance ~225km 
   EXPECT_NEAR(224941.284,PE::TOOLS::to_distance(PE::TPosition(52.054274,10.008114),PE::TPosition(52.671695,13.162785)),0.001);
   EXPECT_NEAR(224919.158,PE::TOOLS::to_distance_precise(PE::TPosition(52.054274,10.008114),PE::TPosition(52.671695,13.162785)),0.001);

   //SOUTH-WEST coordinates
   //small distance
   EXPECT_NEAR(60.704,PE::TOOLS::to_distance(PE::TPosition(-16.499917,-68.150214),PE::TPosition(-16.500336,-68.149849)),0.001);
   EXPECT_NEAR(60.704,PE::TOOLS::to_distance_precise(PE::TPosition(-16.499917,-68.150214),PE::TPosition(-16.500336,-68.149849)),0.001);
}

//Test heading calc between two coordinates
TEST_F(PEToolsTest, heading_from_coordinates_test)
{
   //North 0.0
   EXPECT_NEAR(0.0,PE::TOOLS::to_heading(PE::TPosition( 52.0524,   10.0548 ),PE::TPosition(  52.0596  ,    10.0548)),0.01);
   //East 90.0
   EXPECT_NEAR(90.0,PE::TOOLS::to_heading(PE::TPosition( 52.0524,   10.0548 ),PE::TPosition(  52.0524  ,    10.0620)),0.01);
   //South 180.0
   EXPECT_NEAR(180.0,PE::TOOLS::to_heading(PE::TPosition( 52.0596,   10.0548 ),PE::TPosition(  52.0524  ,    10.0548)),0.01);
   //West 270.0
   EXPECT_NEAR(270.0,PE::TOOLS::to_heading(PE::TPosition( 52.0524,   10.0620 ),PE::TPosition(  52.0524  ,    10.0548)),0.01);
   //PointLU 307.60
   EXPECT_NEAR(307.60,PE::TOOLS::to_heading(PE::TPosition( 52.045690, 10.002842 ),PE::TPosition( 52.045753 , 10.002709)),0.01);
   //PointRD 138.96
   EXPECT_NEAR(138.96,PE::TOOLS::to_heading(PE::TPosition( 52.045690, 10.002842 ),PE::TPosition( 52.045596 , 10.002975)),0.01);
   //SOUTH-WEST coordinates: 140.13
   EXPECT_NEAR(140.13,PE::TOOLS::to_heading(PE::TPosition(-16.499917,-68.150214),PE::TPosition(-16.500336,-68.149849)),0.01);
}


//Test calc new coordinates based on distance, heading and start coordinates in SOUTH-WEST
TEST_F(PEToolsTest, next_coordinates_SW_test)
{
   //SOUTH-WEST coordinates:
   PE::TPosition pos = PE::TOOLS::to_position(PE::TPosition( -16.499917,-68.150214),60.704,140.13);
   
   EXPECT_NEAR(-16.500336, pos.Latitude , 0.000001);
   EXPECT_NEAR(-68.149849, pos.Longitude, 0.000001);
}

//Test calc new coordinates based on distance, heading and start coordinates in NORTH-EAST
TEST_F(PEToolsTest, next_coordinates_NE_test)
{
   //NORTH-EAST coordinates:
   PE::TPosition pos = PE::TOOLS::to_position(PE::TPosition( 52.054274,10.008114),10428.22,54.01);
   EXPECT_NEAR( 52.109320, pos.Latitude , 0.000001);
   EXPECT_NEAR( 10.131668, pos.Longitude, 0.000001);
}

//Test calc new coordinates based on distance, heading and start coordinates crossing grinvich
TEST_F(PEToolsTest, next_coordinates_crossing_grinvich_test)
{
   //cross E->W
   PE::TPosition pos1 = PE::TOOLS::to_position(PE::TPosition( 52.123456,0.000123),100,270.0);
   EXPECT_NEAR( 52.123456, pos1.Latitude , 0.000001);
   EXPECT_NEAR( -0.001341, pos1.Longitude, 0.000001);

   //cross W->E
   PE::TPosition pos2 = PE::TOOLS::to_position(PE::TPosition( 52.123456,-0.001341),100,90.0);
   EXPECT_NEAR( 52.123456, pos2.Latitude , 0.000001);
   EXPECT_NEAR(  0.000123, pos2.Longitude, 0.000001);
}

//Test calc new coordinates based on distance, heading and start coordinates crossing ecvator
TEST_F(PEToolsTest, next_coordinates_crossing_ecvator_test)
{
   //cross N->S
   PE::TPosition pos1 = PE::TOOLS::to_position(PE::TPosition( 0.000123,10.123456),100,180.0);
   EXPECT_NEAR( -0.000776, pos1.Latitude , 0.000001);
   EXPECT_NEAR( 10.123456, pos1.Longitude, 0.000001);
   //cross S->N
   PE::TPosition pos2 = PE::TOOLS::to_position(PE::TPosition( -0.000776,10.123456),100,0.0);
   EXPECT_NEAR(  0.000123, pos2.Latitude , 0.000001);
   EXPECT_NEAR( 10.123456, pos2.Longitude, 0.000001);
}

TEST_F(PEToolsTest, complex_test)
{
   PE::TValue distance1 = PE::TOOLS::to_distance(PE::TPosition(50.12345,10.12345),PE::TPosition(50.65432,10.65432));
   PE::TValue heading1  = PE::TOOLS::to_heading(PE::TPosition(50.12345,10.12345),PE::TPosition(50.65432,10.65432));
   
   PE::TPosition pos1 = PE::TOOLS::to_position(PE::TPosition( 50.12345,10.12345),distance1,heading1);
   EXPECT_NEAR( 50.65432, pos1.Latitude , 0.00001);
   EXPECT_NEAR( 10.65432, pos1.Longitude, 0.00001);
   
   PE::TValue distance2 = PE::TOOLS::to_distance_precise(PE::TPosition(50.1234567,5.1234567),PE::TPosition(50.7654321,-5.1234567));
   PE::TValue heading2  = PE::TOOLS::to_heading(PE::TPosition(50.1234567,5.1234567),PE::TPosition(50.7654321,-5.1234567));
   PE::TPosition pos2 = PE::TOOLS::to_position(PE::TPosition( 50.1234567,5.1234567),distance2,heading2);
   EXPECT_NEAR( 50.7654321, pos2.Latitude , 0.0000001);
   EXPECT_NEAR( -5.1234567, pos2.Longitude, 0.0000001);
}

TEST_F(PEToolsTest, convergence_zero_test)
{
   EXPECT_NEAR(100.0, PE::TOOLS::to_convergence(0,0),0.0001);
   EXPECT_NEAR(0.0, PE::TOOLS::to_convergence(100,0),0.0001);
   EXPECT_NEAR(0.0, PE::TOOLS::to_convergence(0,100),0.0001);
   EXPECT_NEAR(0.0, PE::TOOLS::to_convergence(-1,0),0.0001);
   EXPECT_NEAR(0.0, PE::TOOLS::to_convergence(0,-1),0.0001);
}

TEST_F(PEToolsTest, convergence_negative_positive_values_test)
{
   EXPECT_NEAR(0.0, PE::TOOLS::to_convergence(-100,100),0.0001);
   EXPECT_NEAR(0.0, PE::TOOLS::to_convergence(100,-100),0.0001);
}

TEST_F(PEToolsTest, convergence_negative_values_test)
{
   EXPECT_NEAR(1.0, PE::TOOLS::to_convergence(-100,-1),0.0001);
   EXPECT_NEAR(1.0, PE::TOOLS::to_convergence(-0.1,-0.001),0.0001);
}

TEST_F(PEToolsTest, convergence_positive_values_test)
{
   EXPECT_NEAR(10.0, PE::TOOLS::to_convergence(0.0001,0.00001),0.0001);
   EXPECT_NEAR(10.0, PE::TOOLS::to_convergence(0.00001,0.0001),0.0001);
   EXPECT_NEAR(99.0, PE::TOOLS::to_convergence(0.0001,0.000099),0.0001);
   EXPECT_NEAR(99.0, PE::TOOLS::to_convergence(0.000099,0.0001),0.0001);
   EXPECT_NEAR(99.99, PE::TOOLS::to_convergence(100,99.99),0.0001);
   EXPECT_NEAR(99.99, PE::TOOLS::to_convergence(99.99,100),0.0001);
}

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

