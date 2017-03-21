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
 * Unit test of the PEPositionFilterTest class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PEPositionFilter.h"

class PEPositionFilterTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }
};

/**
 * tests invalid position handling
 */
TEST_F(PEPositionFilterTest, invalid_position_test)
{
   PE::position_filter filter(0);

   EXPECT_FALSE(filter.get_position().is_valid());
   EXPECT_EQ(0.0, filter.get_timestamp());
   EXPECT_EQ(0.0, filter.get_speed_limit());

   //set correct first timesatmp and position
   filter.add_position(1.000, PE::TPosition(52.054274,10.008114));
   EXPECT_TRUE(filter.get_position().is_valid());
   EXPECT_EQ(1.000, filter.get_timestamp());

   //set incorrect timestamp same like before
   filter.add_position(1.000, PE::TPosition(1.0,2.0));
   EXPECT_TRUE(filter.get_position().is_valid());
   EXPECT_EQ(1.000, filter.get_timestamp());
   EXPECT_NEAR(52.054274, filter.get_position().Latitude,0.000001);
   EXPECT_NEAR(10.008114, filter.get_position().Longitude,0.000001);

   //set incorrect position
   filter.add_position(2.000, PE::TPosition());
   EXPECT_TRUE(filter.get_position().is_valid());
   EXPECT_EQ(1.000, filter.get_timestamp());
   EXPECT_NEAR(52.054274, filter.get_position().Latitude,0.000001);
   EXPECT_NEAR(10.008114, filter.get_position().Longitude,0.000001);

   //set correct data
   filter.add_position(2.000, PE::TPosition(52.054447,10.008288));
   EXPECT_TRUE(filter.get_position().is_valid());
   EXPECT_EQ(2.000, filter.get_timestamp());
   EXPECT_NEAR(52.054447, filter.get_position().Latitude,0.000001);
   EXPECT_NEAR(10.008288, filter.get_position().Longitude,0.000001);
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

