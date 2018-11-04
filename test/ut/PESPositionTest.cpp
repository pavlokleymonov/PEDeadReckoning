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
 * Unit test of the PESPosition class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PESPosition.h"

class PESPositionTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }
};


//Test Default constructor
TEST_F(PESPositionTest, default_constructor_test)
{
   PE::SPosition pos;

   EXPECT_LT(PE::ABS_MAX_LATITUDE, pos.Latitude);
   EXPECT_LT(PE::ABS_MAX_LONGITUDE, pos.Longitude);
   EXPECT_LE(PE::MAX_ACCURACY, pos.HorizontalAcc);
   EXPECT_FALSE(pos.IsValid());
}


//Test simple constructor
TEST_F(PESPositionTest, constructor_test)
{
   PE::SPosition pos_valid = PE::SPosition(50,120,1);

   EXPECT_TRUE(pos_valid.IsValid());
   EXPECT_EQ(50, pos_valid.Latitude);
   EXPECT_EQ(120, pos_valid.Longitude);
   EXPECT_EQ(1, pos_valid.HorizontalAcc);

   //lat invalid positive
   EXPECT_FALSE(PE::SPosition(91,120,1).IsValid());
   //lat invalid negative
   EXPECT_FALSE(PE::SPosition(-91,120,1).IsValid());
   //lon invalid positive
   EXPECT_FALSE(PE::SPosition(50,181,1).IsValid());
   //lon invalid negative
   EXPECT_FALSE(PE::SPosition(50,-181,1).IsValid());
   //max valid positive
   EXPECT_TRUE(PE::SPosition(90,180,1).IsValid());
   //max valid negative
   EXPECT_TRUE(PE::SPosition(-90,-180,1).IsValid());
   //accuracy invalid
   EXPECT_FALSE(PE::SPosition(90,180).IsValid());
   //accuracy invalid
   EXPECT_FALSE(PE::SPosition(-90,-180).IsValid());
   //accuracy invalid
   EXPECT_FALSE(PE::SPosition(10,20).IsValid());
}


//Test operator==
TEST_F(PESPositionTest, operator_eq_test)
{
   PE::SPosition pos1;
   PE::SPosition pos2;
   PE::SPosition pos3(10.0,0.2);
   PE::SPosition pos4(10.0,0.2);
   PE::SPosition pos5(20.0,0.4);
   PE::SPosition pos6(20.0,0.4);

   EXPECT_EQ(pos1,pos2);
   EXPECT_EQ(pos3,pos4);
   EXPECT_EQ(pos5,pos6);

   EXPECT_FALSE(pos1 == pos3);
   EXPECT_FALSE(pos1 == pos5);
   EXPECT_FALSE(pos2 == pos4);
   EXPECT_FALSE(pos2 == pos6);
   EXPECT_FALSE(pos3 == pos5);
   EXPECT_FALSE(pos4 == pos6);

   EXPECT_FALSE(PE::SPosition(10,20) == PE::SPosition(0,20));
   EXPECT_FALSE(PE::SPosition(10,20) == PE::SPosition(10,0));

   EXPECT_FALSE(PE::SPosition(10,20,30) == PE::SPosition(0,20,30));
   EXPECT_FALSE(PE::SPosition(10,20,30) == PE::SPosition(10,0,30));
   EXPECT_FALSE(PE::SPosition(10,20,30) == PE::SPosition(10,20,0));
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
