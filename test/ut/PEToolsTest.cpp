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


//Test calc new coordinates based on distance, heading and start coordinates crossing antimeridian
TEST_F(PEToolsTest, next_coordinates_crossing_antimeredian_test)
{
   //cross E->W
   PE::SPosition pos1 = PE::TOOLS::ToPosition(PE::SPosition( 52.123456,179.999123),100,90.0);
   EXPECT_NEAR( 52.123456, pos1.Latitude , 0.000001);
   EXPECT_NEAR(-179.999412, pos1.Longitude, 0.000001);

   //cross W->E
   PE::SPosition pos2 = PE::TOOLS::ToPosition(PE::SPosition( 52.123456,-179.999132),100,270.0);
   EXPECT_NEAR( 52.123456, pos2.Latitude , 0.000001);
   EXPECT_NEAR(179.999403, pos2.Longitude, 0.000001);
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


TEST_F(PEToolsTest, coordinate_plus_0_deg_transform_2d_test)
{
   PE::TValue x = 1;
   PE::TValue y = 0;
   PE::TValue zAng = 0;
   PE::TOOLS::Transform2D(x, y, zAng);
   EXPECT_NEAR(1,x,0.001);
   EXPECT_NEAR(0,y,0.001);
}


TEST_F(PEToolsTest, coordinate_plus_45_deg_transform_2d_test)
{
   PE::TValue x = 1;
   PE::TValue y = 0;
   PE::TValue zAng = 45;
   PE::TOOLS::Transform2D(x, y, zAng);
   EXPECT_NEAR(0.7071,x,0.001);
   EXPECT_NEAR(0.7071,y,0.001);
}


TEST_F(PEToolsTest, coordinate_plus_90_deg_transform_2d_test)
{
   PE::TValue x = 1;
   PE::TValue y = 0;
   PE::TValue zAng = 90;
   PE::TOOLS::Transform2D(x, y, zAng);
   EXPECT_NEAR(0,x,0.001);
   EXPECT_NEAR(1,y,0.001);
}


TEST_F(PEToolsTest, coordinate_plus_180_deg_transform_2d_test)
{
   PE::TValue x = 1;
   PE::TValue y = 0;
   PE::TValue zAng = 180;
   PE::TOOLS::Transform2D(x, y, zAng);
   EXPECT_NEAR(-1,x,0.001);
   EXPECT_NEAR(0,y,0.001);
}


TEST_F(PEToolsTest, coordinate_plus_270_deg_transform_2d_test)
{
   PE::TValue x = 1;
   PE::TValue y = 0;
   PE::TValue zAng = 270;
   PE::TOOLS::Transform2D(x, y, zAng);
   EXPECT_NEAR(0,x,0.001);
   EXPECT_NEAR(-1,y,0.001);
}


TEST_F(PEToolsTest, coordinate_plus_360_deg_transform_2d_test)
{
   PE::TValue x = 1;
   PE::TValue y = 0;
   PE::TValue zAng = 360;
   PE::TOOLS::Transform2D(x, y, zAng);
   EXPECT_NEAR(1,x,0.001);
   EXPECT_NEAR(0,y,0.001);
}


TEST_F(PEToolsTest, coordinate_minus_90_deg_transform_2d_test)
{
   PE::TValue x = 1;
   PE::TValue y = 0;
   PE::TValue zAng = -90;
   PE::TOOLS::Transform2D(x, y, zAng);
   EXPECT_NEAR(0,x,0.001);
   EXPECT_NEAR(-1,y,0.001);
}


TEST_F(PEToolsTest, coordinate_x0_y0_z90_deg_transform_3d_test)
{
   PE::TValue x = 1;
   PE::TValue y = 0;
   PE::TValue z = 0;
   PE::TValue xAng = 0;
   PE::TValue yAng = 0;
   PE::TValue zAng = 90;
   PE::TOOLS::Transform3D(x, y, z, xAng, yAng, zAng);
   EXPECT_NEAR(0,x,0.001);
   EXPECT_NEAR(1,y,0.001);
   EXPECT_NEAR(0,z,0.001);
}


TEST_F(PEToolsTest, coordinate_x90_y0_z90_deg_transform_3d_test)
{
   PE::TValue x = 1;
   PE::TValue y = 0;
   PE::TValue z = 0;
   PE::TValue xAng = 90;
   PE::TValue yAng = 0;
   PE::TValue zAng = 90;
   PE::TOOLS::Transform3D(x, y, z, xAng, yAng, zAng);
   EXPECT_NEAR(0,x,0.001);
   EXPECT_NEAR(1,y,0.001);
   EXPECT_NEAR(0,z,0.001);
}


TEST_F(PEToolsTest, coordinate_x90_y90_z90_deg_transform_3d_test)
{
   PE::TValue x = 1;
   PE::TValue y = 0;
   PE::TValue z = 0;
   PE::TValue xAng = 90;
   PE::TValue yAng = 90;
   PE::TValue zAng = 90;
   PE::TOOLS::Transform3D(x, y, z, xAng, yAng, zAng);
   EXPECT_NEAR(0,x,0.001);
   EXPECT_NEAR(0,y,0.001);
   EXPECT_NEAR(-1,z,0.001);
}


TEST_F(PEToolsTest, coordinate_vX1_vY2_vZ3_x90_y90_z90_deg_transform_3d_test)
{
   PE::TValue x = 1;
   PE::TValue y = 2;
   PE::TValue z = 3;
   PE::TValue xAng = 90;
   PE::TValue yAng = 90;
   PE::TValue zAng = 90;
   PE::TOOLS::Transform3D(x, y, z, xAng, yAng, zAng);
   EXPECT_NEAR(3,x,0.001);
   EXPECT_NEAR(2,y,0.001);
   EXPECT_NEAR(-1,z,0.001);
}


TEST_F(PEToolsTest, split_string_test)
{
   //Empty string test
   EXPECT_EQ(0, PE::TOOLS::Split("",',').size());

   //no delimiter test
   EXPECT_EQ(1, PE::TOOLS::Split("12345",',').size());
   EXPECT_EQ("12345", PE::TOOLS::Split("12345",',')[0]);

   //no content test
   EXPECT_EQ(2, PE::TOOLS::Split(",",',').size());
   EXPECT_TRUE(PE::TOOLS::Split(",",',')[0].empty());
   EXPECT_TRUE(PE::TOOLS::Split(",",',')[1].empty());

   //CASE: "111,222,333"
   EXPECT_EQ(3, PE::TOOLS::Split("111,222,333",',').size());
   EXPECT_EQ("111", PE::TOOLS::Split("111,222,333",',')[0]);
   EXPECT_EQ("222", PE::TOOLS::Split("111,222,333",',')[1]);
   EXPECT_EQ("333", PE::TOOLS::Split("111,222,333",',')[2]);

   //CASE: ",222,333"
   EXPECT_EQ(3, PE::TOOLS::Split(",222,333",',').size());
   EXPECT_EQ("",    PE::TOOLS::Split(",222,333",',')[0]);
   EXPECT_EQ("222", PE::TOOLS::Split(",222,333",',')[1]);
   EXPECT_EQ("333", PE::TOOLS::Split(",222,333",',')[2]);

   //CASE: "111,222,"
   EXPECT_EQ(3, PE::TOOLS::Split("111,222,",',').size());
   EXPECT_EQ("111", PE::TOOLS::Split("111,222,",',')[0]);
   EXPECT_EQ("222", PE::TOOLS::Split("111,222,",',')[1]);
   EXPECT_EQ("",    PE::TOOLS::Split("111,222,",',')[2]);
}

TEST_F(PEToolsTest, pos_set_1m_3m_6m_10m_15m_21m_28m_36m_45m_55m_test)
{
   PE::SPosition start(50.0, 10.0, 1.0);

   //1m
   EXPECT_NEAR(50.00000636, PE::TOOLS::ToPosition(start, 1, 45.0).Latitude, 0.00000001);
   EXPECT_NEAR(10.00000989, PE::TOOLS::ToPosition(start, 1, 45.0).Longitude, 0.00000001);

   //3m (2m/s)
   EXPECT_NEAR(50.00001907, PE::TOOLS::ToPosition(start, 3, 45.0).Latitude, 0.00000001);
   EXPECT_NEAR(10.00002967, PE::TOOLS::ToPosition(start, 3, 45.0).Longitude, 0.00000001);

   //6m (3m/s)
   EXPECT_NEAR(50.00003815, PE::TOOLS::ToPosition(start, 6, 45.0).Latitude, 0.00000001);
   EXPECT_NEAR(10.00005935, PE::TOOLS::ToPosition(start, 6, 45.0).Longitude, 0.00000001);

   //10m (4m/s)
   EXPECT_NEAR(50.00006359, PE::TOOLS::ToPosition(start, 10, 45.0).Latitude, 0.00000001);
   EXPECT_NEAR(10.00009893, PE::TOOLS::ToPosition(start, 10, 45.0).Longitude, 0.00000001);

   //15m (5m/s)
   EXPECT_NEAR(50.00009538, PE::TOOLS::ToPosition(start, 15, 45.0).Latitude, 0.00000001);
   EXPECT_NEAR(10.00014839, PE::TOOLS::ToPosition(start, 15, 45.0).Longitude, 0.00000001);

   //21m (6m/s)
   EXPECT_NEAR(50.00013354, PE::TOOLS::ToPosition(start, 21, 45.0).Latitude, 0.00000001);
   EXPECT_NEAR(10.00020775, PE::TOOLS::ToPosition(start, 21, 45.0).Longitude, 0.00000001);

   //28m (7m/s)
   EXPECT_NEAR(50.00017805, PE::TOOLS::ToPosition(start, 28, 45.0).Latitude, 0.00000001);
   EXPECT_NEAR(10.00027700, PE::TOOLS::ToPosition(start, 28, 45.0).Longitude, 0.00000001);

   //36m (8m/s)
   EXPECT_NEAR(50.00022892, PE::TOOLS::ToPosition(start, 36, 45.0).Latitude, 0.00000001);
   EXPECT_NEAR(10.00035615, PE::TOOLS::ToPosition(start, 36, 45.0).Longitude, 0.00000001);

   //45m (9m/s)
   EXPECT_NEAR(50.00028616, PE::TOOLS::ToPosition(start, 45, 45.0).Latitude, 0.00000001);
   EXPECT_NEAR(10.00044519, PE::TOOLS::ToPosition(start, 45, 45.0).Longitude, 0.00000001);

   //55m (10m/s)
   EXPECT_NEAR(50.00034975, PE::TOOLS::ToPosition(start, 55, 45.0).Latitude, 0.00000001);
   EXPECT_NEAR(10.00054412, PE::TOOLS::ToPosition(start, 55, 45.0).Longitude, 0.00000001);

   //63m (8m/s)
   EXPECT_NEAR(50.00040062, PE::TOOLS::ToPosition(start, 63, 45.0).Latitude, 0.00000001);
   EXPECT_NEAR(10.00062327, PE::TOOLS::ToPosition(start, 63, 45.0).Longitude, 0.00000001);

   //67m (4m/s)
   EXPECT_NEAR(50.00042606, PE::TOOLS::ToPosition(start, 67, 45.0).Latitude, 0.00000001);
   EXPECT_NEAR(10.00066284, PE::TOOLS::ToPosition(start, 67, 45.0).Longitude, 0.00000001);

   //76m (9m/s)
   EXPECT_NEAR(50.00048329, PE::TOOLS::ToPosition(start, 76, 45.0).Latitude, 0.00000001);
   EXPECT_NEAR(10.00075188, PE::TOOLS::ToPosition(start, 76, 45.0).Longitude, 0.00000001);
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
