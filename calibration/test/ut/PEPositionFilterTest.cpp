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
namespace PE
{
bool operator==(const PE::TPosition& lhs, const PE::TPosition& rhs)
{
   return ( lhs.Latitude == rhs.Latitude &&
            lhs.Longitude == rhs.Longitude &&
            lhs.LatitudeAcc == rhs.LatitudeAcc &&
            lhs.LongitudeAcc == rhs.LongitudeAcc );
}
}

/**
 * test interface class
 */
TEST_F(PEPositionFilterTest, test_interface_class_position)
{
   PE::I_position_filter i_pos;
   const PE::TPosition POS_START = PE::TPosition(52.0, 10.0);
   EXPECT_EQ(0, i_pos.get_timestamp());
   EXPECT_FALSE(i_pos.get_position().is_valid());

   i_pos.add_position(10.000, POS_START);
   EXPECT_TRUE(i_pos.get_position().is_valid());
   EXPECT_EQ(10.000, i_pos.get_timestamp());
   EXPECT_EQ(POS_START, i_pos.get_position());
}

/**
 * tests invalid timestamp 0
 */
TEST_F(PEPositionFilterTest, test_invalid_timestamp_zero)
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::TPosition           POS_START = PE::TPosition(52.0, 10.0);
   PE::position_filter_speed filter(SPEED_LIMIT_1M_PER_SEC);

   //TS = 0
   filter.add_position(0.000, POS_START );
   EXPECT_FALSE(filter.get_position().is_valid());

   //TS >0
   filter.add_position(0.001, POS_START );
   EXPECT_EQ(0.001, filter.get_timestamp());
   EXPECT_TRUE(filter.get_position().is_valid());
}

/**
 * tests invalid negative timestamp
 */
TEST_F(PEPositionFilterTest, test_invalid_negative_timestamp )
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::TPosition           POS_START = PE::TPosition(52.0, 10.0);
   PE::position_filter_speed filter(SPEED_LIMIT_1M_PER_SEC);

   //TS < 0
   filter.add_position(-1.000, POS_START );
   EXPECT_FALSE(filter.get_position().is_valid());

   //TS >0
   filter.add_position(0.001, POS_START );
   EXPECT_EQ(0.001, filter.get_timestamp());
   EXPECT_TRUE(filter.get_position().is_valid());
}

/**
 * tests ignore invalid position
 */
TEST_F(PEPositionFilterTest, tests_ignore_invalid_position )
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::TPosition           POS_START = PE::TPosition(52.0, 10.0);
   const PE::TPosition         POS_INVALID = PE::TPosition(90.0001, 10.0); //invalid latitude
   const PE::TPosition           POS_VALID = PE::TPosition(90.0, 10.0);
   PE::position_filter_speed filter(SPEED_LIMIT_1M_PER_SEC);

   filter.add_position(1.000, POS_START );
   EXPECT_EQ(POS_START, filter.get_position());

   //invalid position
   filter.add_position(2.000, POS_INVALID );
   EXPECT_EQ(1.000, filter.get_timestamp()); //still old timestamp
   EXPECT_EQ(POS_START, filter.get_position()); //still old position

   //valid position
   filter.add_position(3.000, POS_VALID );
   EXPECT_EQ(3.000, filter.get_timestamp()); //new timestamp
   EXPECT_EQ(POS_VALID, filter.get_position()); //new position
   
}

/**
 * tests ignore outdated position
 */
TEST_F(PEPositionFilterTest, tests_ignore_outdated_position )
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::TPosition           POS_START = PE::TPosition(52.0, 10.0);
   const PE::TPosition           POS_VALID = PE::TPosition(90.0, 10.0);
   PE::position_filter_speed filter(SPEED_LIMIT_1M_PER_SEC);

   filter.add_position(1.000, POS_START );
   EXPECT_EQ(POS_START, filter.get_position());

   filter.add_position(0.999, POS_VALID );
   EXPECT_EQ(1.000, filter.get_timestamp()); //still old timestamp
   EXPECT_EQ(POS_START, filter.get_position()); //still old position

   filter.add_position(3.000, POS_VALID );
   EXPECT_EQ(3.000, filter.get_timestamp()); //new timestamp
   EXPECT_EQ(POS_VALID, filter.get_position()); //new position
}

/**
 * tests ignore position with same timestamp
 */
TEST_F(PEPositionFilterTest, tests_ignore_position_same_timestamp )
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::TPosition           POS_START = PE::TPosition(52.0, 10.0);
   const PE::TPosition           POS_VALID = PE::TPosition(90.0, 10.0);
   PE::position_filter_speed filter(SPEED_LIMIT_1M_PER_SEC);

   filter.add_position(1.000, POS_START );
   EXPECT_EQ(POS_START, filter.get_position());

   filter.add_position(1.000, POS_VALID );
   EXPECT_EQ(1.000, filter.get_timestamp()); //still old timestamp
   EXPECT_EQ(POS_START, filter.get_position()); //still old position
}

/**
 * tests move position one direction
 */
TEST_F(PEPositionFilterTest, move_position_one_direction)
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::TValue            DISTANCE_1M = 1.0; //1 meter
   const PE::TValue            DISTANCE_2M = 2.0; //2 meter
   const PE::TPosition           POS_START = PE::TPosition(52.0, 10.0);
   const PE::TPosition   POS_START_PLUS_1M = PE::TOOLS::to_position(POS_START,DISTANCE_1M,0);
   const PE::TPosition   POS_START_PLUS_2M = PE::TOOLS::to_position(POS_START,DISTANCE_2M,0);
   
   PE::position_filter_speed filter(SPEED_LIMIT_1M_PER_SEC);
   EXPECT_FALSE(filter.get_position().is_valid());

   filter.add_position(10.000, POS_START );
   EXPECT_TRUE(filter.get_position().is_valid());
   EXPECT_EQ(10.000, filter.get_timestamp());
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 1 meter per 1 second from original - no change
   filter.add_position(11.000, POS_START_PLUS_1M);
   EXPECT_EQ(11.000, filter.get_timestamp());
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 2 meter per 1 second from original - changed
   filter.add_position(12.000, POS_START_PLUS_2M);
   EXPECT_EQ(12.000, filter.get_timestamp());
   EXPECT_EQ(POS_START_PLUS_2M, filter.get_position());
}

/**
 * tests move position around
 */
TEST_F(PEPositionFilterTest, move_position_around)
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::TValue            DISTANCE_1M = 1.0; //1 meter
   const PE::TValue            DISTANCE_2M = 2.0; //2 meter
   const PE::TPosition           POS_START = PE::TPosition(52.0, 10.0);
   const PE::TPosition   POS_START_PLUS_1M_UP    = PE::TOOLS::to_position(POS_START,DISTANCE_1M,0);
   const PE::TPosition   POS_START_PLUS_1M_LEFT  = PE::TOOLS::to_position(POS_START,DISTANCE_1M,270);
   const PE::TPosition   POS_START_PLUS_1M_RIGHT = PE::TOOLS::to_position(POS_START,DISTANCE_1M,90);
   const PE::TPosition   POS_START_PLUS_1M_DOWN  = PE::TOOLS::to_position(POS_START,DISTANCE_1M,180);
   const PE::TPosition   POS_START_PLUS_2M_UP    = PE::TOOLS::to_position(POS_START,DISTANCE_2M,0);
   
   PE::position_filter_speed filter(SPEED_LIMIT_1M_PER_SEC);
   EXPECT_FALSE(filter.get_position().is_valid());
   EXPECT_EQ(SPEED_LIMIT_1M_PER_SEC, filter.get_speed_limit());

   filter.add_position(10.000, POS_START );
   EXPECT_TRUE(filter.get_position().is_valid());
   EXPECT_EQ(10.000, filter.get_timestamp());
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 1 meter per 1 second from original - no change
   filter.add_position(11.000, POS_START_PLUS_1M_UP);
   EXPECT_EQ(11.000, filter.get_timestamp());
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 1 meter per 1 second from original - no change
   filter.add_position(12.000, POS_START_PLUS_1M_DOWN);
   EXPECT_EQ(12.000, filter.get_timestamp());
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 1 meter per 1 second from original - no change
   filter.add_position(13.000, POS_START_PLUS_1M_RIGHT);
   EXPECT_EQ(13.000, filter.get_timestamp());
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 1 meter per 1 second from original - no change
   filter.add_position(14.000, POS_START_PLUS_1M_LEFT);
   EXPECT_EQ(14.000, filter.get_timestamp());
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 2 meter per 1 second from original - changed
   filter.add_position(15.000, POS_START_PLUS_2M_UP);
   EXPECT_EQ(15.000, filter.get_timestamp());
   EXPECT_EQ(POS_START_PLUS_2M_UP, filter.get_position());
}

/**
 * tests move position always above limit
 */
TEST_F(PEPositionFilterTest, move_position_always_above_limit)
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::TValue            DISTANCE_1M = 1.0; //1 meter
   const PE::TValue            DISTANCE_2M = 2.0; //2 meter
   const PE::TPosition           POS_START = PE::TPosition(52.0, 10.0);
   const PE::TPosition               POS_1 = PE::TOOLS::to_position(POS_START,DISTANCE_2M,0);
   const PE::TPosition               POS_2 = PE::TOOLS::to_position(POS_1,DISTANCE_2M,90);
   const PE::TPosition               POS_3 = PE::TOOLS::to_position(POS_2,DISTANCE_2M,180);
   const PE::TPosition               POS_4 = PE::TOOLS::to_position(POS_3,DISTANCE_2M,270);
   const PE::TPosition               POS_5 = PE::TOOLS::to_position(POS_4,DISTANCE_2M,0);
   
   PE::position_filter_speed filter(SPEED_LIMIT_1M_PER_SEC);
   EXPECT_FALSE(filter.get_position().is_valid());

   filter.add_position(10.000, POS_START );
   EXPECT_TRUE(filter.get_position().is_valid());
   EXPECT_EQ(10.000, filter.get_timestamp());
   EXPECT_EQ(POS_START, filter.get_position());

   //distance 2 meter per 1 second from original - change
   filter.add_position(11.000, POS_1);
   EXPECT_EQ(11.000, filter.get_timestamp());
   EXPECT_EQ(POS_1, filter.get_position());

   //distance 2 meter per 1 second from original - change
   filter.add_position(12.000, POS_2);
   EXPECT_EQ(12.000, filter.get_timestamp());
   EXPECT_EQ(POS_2, filter.get_position());

   //distance 2 meter per 1 second from original - change
   filter.add_position(13.000, POS_3);
   EXPECT_EQ(13.000, filter.get_timestamp());
   EXPECT_EQ(POS_3, filter.get_position());

   //distance 2 meter per 1 second from original - change
   filter.add_position(14.000, POS_4);
   EXPECT_EQ(14.000, filter.get_timestamp());
   EXPECT_EQ(POS_4, filter.get_position());

   //distance 2 meter per 1 second from original - change
   filter.add_position(15.000, POS_5);
   EXPECT_EQ(15.000, filter.get_timestamp());
   EXPECT_EQ(POS_5, filter.get_position());
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

