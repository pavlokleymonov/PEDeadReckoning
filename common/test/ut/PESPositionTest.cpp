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
TEST_F(PESPositionTest, simple_constructor_test)
{
   PE::SPosition pos_valid = PE::SPosition(50,120,1);

   EXPECT_TRUE(pos_valid.IsValid());
   EXPECT_EQ(50, pos_valid.Latitude);
   EXPECT_EQ(120, pos_valid.Longitude);
   EXPECT_EQ(1, pos_valid.HorizontalAcc);

   //lat invalid
   PE::SPosition pos_invalid_1 = PE::SPosition(91,120,1);
   EXPECT_FALSE(pos_invalid_1.IsValid());
   //lon invalid
   PE::SPosition pos_invalid_2 = PE::SPosition(50,181,1);
   EXPECT_FALSE(pos_invalid_2.IsValid());
   //accuracyn invalid
   PE::SPosition pos_invalid_3 = PE::SPosition(50,180);
   EXPECT_FALSE(pos_invalid_3.IsValid());
}

//Test full constructor
TEST_F(PESPositionTest, full_constructor_test)
{
   PE::SPosition pos_valid = PE::SPosition(50,120, 10);

   EXPECT_TRUE(pos_valid.IsValid());
   EXPECT_EQ(50, pos_valid.Latitude);
   EXPECT_EQ(120, pos_valid.Longitude);
   EXPECT_EQ(10, pos_valid.HorizontalAcc);

   PE::SPosition pos_invalid_1 = PE::SPosition(91,120,1);
   EXPECT_FALSE(pos_invalid_1.IsValid());

   PE::SPosition pos_invalid_2 = PE::SPosition(50,181,1);
   EXPECT_FALSE(pos_invalid_2.IsValid());
}

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
