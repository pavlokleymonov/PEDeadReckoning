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
#include "PETools.h"

class PEPositionFilterTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }
};

/**
 * tests move position one direction
 */
TEST_F(PEPositionFilterTest, move_position_same_direction)
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.0; //1 m/s
   const PE::TValue            DISTANCE_1M = 1.0; //1 meter
   const PE::TValue            DISTANCE_2M = 2.0; //2 meter
   const PE::TPosition           POS_START = PE::TPosition(52.054313, 10.008143);
   const PE::TPosition   POS_START_PLUS_1M = PE::TOOLS::to_position(POS_START,DISTANCE_1M,0);
   const PE::TPosition   POS_START_PLUS_2M = PE::TOOLS::to_position(POS_START,DISTANCE_2M,0);
   
   PE::position_filter_speed filter(SPEED_LIMIT_1M_PER_SEC);
   EXPECT_FALSE(filter.get_position().is_valid());

   filter.add_position(10.000, POS_START );
   EXPECT_TRUE(filter.get_position().is_valid());
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 1 meter per 1 second from original - no change
   filter.add_position(11.000, POS_START_PLUS_1M);
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 2 meter per 1 second from original - changed
   filter.add_position(12.000, POS_START_PLUS_2M);
   EXPECT_EQ(POS_START_PLUS_2M, filter.get_position());
}

/**
 * tests move position around
 */
TEST_F(PEPositionFilterTest, move_position_around)
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.0; //1 m/s
   const PE::TValue            DISTANCE_1M = 1.0; //1 meter
   const PE::TValue            DISTANCE_2M = 2.0; //2 meter
   const PE::TPosition           POS_START = PE::TPosition(52.054313, 10.008143);
   const PE::TPosition   POS_START_PLUS_1M_UP    = PE::TOOLS::to_position(POS_START,DISTANCE_1M,0);
   const PE::TPosition   POS_START_PLUS_1M_LEFT  = PE::TOOLS::to_position(POS_START,DISTANCE_1M,270);
   const PE::TPosition   POS_START_PLUS_1M_RIGHT = PE::TOOLS::to_position(POS_START,DISTANCE_1M,90);
   const PE::TPosition   POS_START_PLUS_1M_DOWN  = PE::TOOLS::to_position(POS_START,DISTANCE_1M,180);
   const PE::TPosition   POS_START_PLUS_2M_UP    = PE::TOOLS::to_position(POS_START,DISTANCE_2M,0);
   
   PE::position_filter_speed filter(SPEED_LIMIT_1M_PER_SEC);
   EXPECT_FALSE(filter.get_position().is_valid());

   filter.add_position(10.000, POS_START );
   EXPECT_TRUE(filter.get_position().is_valid());
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 1 meter per 1 second from original - no change
   filter.add_position(11.000, POS_START_PLUS_1M_UP);
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 1 meter per 1 second from original - no change
   filter.add_position(12.000, POS_START_PLUS_1M_DOWN);
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 1 meter per 1 second from original - no change
   filter.add_position(13.000, POS_START_PLUS_1M_RIGHT);
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 1 meter per 1 second from original - no change
   filter.add_position(14.000, POS_START_PLUS_1M_LEFT);
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 2 meter per 1 second from original - changed
   filter.add_position(15.000, POS_START_PLUS_2M_UP);
   EXPECT_EQ(POS_START_PLUS_2M_UP, filter.get_position());
}

/**
 * tests move position always above limit
 */
TEST_F(PEPositionFilterTest, move_position_around)
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.0; //1 m/s
   const PE::TValue            DISTANCE_1M = 1.0; //1 meter
   const PE::TValue            DISTANCE_2M = 2.0; //2 meter
   const PE::TPosition           POS_START = PE::TPosition(52.054313, 10.008143);
   const PE::TPosition               POS_1 = PE::TOOLS::to_position(POS_START,DISTANCE_2M,0);
   const PE::TPosition               POS_2 = PE::TOOLS::to_position(POS_1,DISTANCE_2M,90);
   const PE::TPosition               POS_3 = PE::TOOLS::to_position(POS_2,DISTANCE_2M,180);
   const PE::TPosition               POS_4 = PE::TOOLS::to_position(POS_3,DISTANCE_2M,270);
   const PE::TPosition               POS_5 = PE::TOOLS::to_position(POS_4,DISTANCE_2M,0);
   
   PE::position_filter_speed filter(SPEED_LIMIT_1M_PER_SEC);
   EXPECT_FALSE(filter.get_position().is_valid());

   filter.add_position(10.000, POS_START );
   EXPECT_TRUE(filter.get_position().is_valid());
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 2 meter per 1 second from original - change
   filter.add_position(11.000, POS_1);
   EXPECT_EQ(POS_1, filter.get_position());

   //distance 2 meter per 1 second from original - change
   filter.add_position(12.000, POS_2);
   EXPECT_EQ(POS_2, filter.get_position());

   //distance 2 meter per 1 second from original - change
   filter.add_position(13.000, POS_3);
   EXPECT_EQ(POS_3, filter.get_position());

   //distance 2 meter per 1 second from original - change
   filter.add_position(14.000, POS_4);
   EXPECT_EQ(POS_4, filter.get_position());

   //distance 2 meter per 1 second from original - change
   filter.add_position(15.000, POS_5);
   EXPECT_EQ(POS_5, filter.get_position());
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

