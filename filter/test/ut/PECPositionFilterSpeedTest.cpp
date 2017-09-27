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
 * Unit test of the PECPositionFilterSpeed class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PECPositionFilterSpeed.h"
#include "PETools.h"

class PECPositionFilterSpeedTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }
};
namespace PE
{
bool operator==(const PE::SPosition& lhs, const PE::SPosition& rhs)
{
   return ( lhs.Latitude == rhs.Latitude &&
            lhs.Longitude == rhs.Longitude &&
            lhs.LatitudeAcc == rhs.LatitudeAcc &&
            lhs.LongitudeAcc == rhs.LongitudeAcc );
}
}

/**
 * tests invalid timestamp 0
 */
TEST_F(PECPositionFilterSpeedTest, test_invalid_timestamp_zero)
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::SPosition           POS_START = PE::SPosition(52.0, 10.0);
   PE::CPositionFilterSpeed filter(SPEED_LIMIT_1M_PER_SEC);

   //TS = 0
   filter.AddPosition(0.000, POS_START );
   EXPECT_FALSE(filter.GetPosition().IsValid());

   //TS >0
   filter.AddPosition(0.001, POS_START );
   EXPECT_EQ(0.001, filter.GetTimestamp());
   EXPECT_TRUE(filter.GetPosition().IsValid());
}

/**
 * tests invalid negative timestamp
 */
TEST_F(PECPositionFilterSpeedTest, test_invalid_negative_timestamp )
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::SPosition           POS_START = PE::SPosition(52.0, 10.0);
   PE::CPositionFilterSpeed filter(SPEED_LIMIT_1M_PER_SEC);

   //TS < 0
   filter.AddPosition(-1.000, POS_START );
   EXPECT_FALSE(filter.GetPosition().IsValid());

   //TS >0
   filter.AddPosition(0.001, POS_START );
   EXPECT_EQ(0.001, filter.GetTimestamp());
   EXPECT_TRUE(filter.GetPosition().IsValid());
}

/**
 * tests ignore invalid position
 */
TEST_F(PECPositionFilterSpeedTest, tests_ignore_invalid_position )
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::SPosition           POS_START = PE::SPosition(52.0, 10.0);
   const PE::SPosition         POS_INVALID = PE::SPosition(90.0001, 10.0); //invalid latitude
   const PE::SPosition           POS_VALID = PE::SPosition(90.0, 10.0);
   PE::CPositionFilterSpeed filter(SPEED_LIMIT_1M_PER_SEC);

   filter.AddPosition(1.000, POS_START );
   EXPECT_EQ(POS_START, filter.GetPosition());

   //invalid position
   filter.AddPosition(2.000, POS_INVALID );
   EXPECT_EQ(1.000, filter.GetTimestamp()); //still old timestamp
   EXPECT_EQ(POS_START, filter.GetPosition()); //still old position

   //valid position
   filter.AddPosition(3.000, POS_VALID );
   EXPECT_EQ(3.000, filter.GetTimestamp()); //new timestamp
   EXPECT_EQ(POS_VALID, filter.GetPosition()); //new position
   
}

/**
 * tests ignore outdated position
 */
TEST_F(PECPositionFilterSpeedTest, tests_ignore_outdated_position )
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::SPosition           POS_START = PE::SPosition(52.0, 10.0);
   const PE::SPosition           POS_VALID = PE::SPosition(90.0, 10.0);
   PE::CPositionFilterSpeed filter(SPEED_LIMIT_1M_PER_SEC);

   filter.AddPosition(1.000, POS_START );
   EXPECT_EQ(POS_START, filter.GetPosition());

   filter.AddPosition(0.999, POS_VALID );
   EXPECT_EQ(1.000, filter.GetTimestamp()); //still old timestamp
   EXPECT_EQ(POS_START, filter.GetPosition()); //still old position

   filter.AddPosition(3.000, POS_VALID );
   EXPECT_EQ(3.000, filter.GetTimestamp()); //new timestamp
   EXPECT_EQ(POS_VALID, filter.GetPosition()); //new position
}

/**
 * tests ignore position with same timestamp
 */
TEST_F(PECPositionFilterSpeedTest, tests_ignore_position_same_timestamp )
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::SPosition           POS_START = PE::SPosition(52.0, 10.0);
   const PE::SPosition           POS_VALID = PE::SPosition(90.0, 10.0);
   PE::CPositionFilterSpeed filter(SPEED_LIMIT_1M_PER_SEC);

   filter.AddPosition(1.000, POS_START );
   EXPECT_EQ(POS_START, filter.GetPosition());

   filter.AddPosition(1.000, POS_VALID );
   EXPECT_EQ(1.000, filter.GetTimestamp()); //still old timestamp
   EXPECT_EQ(POS_START, filter.GetPosition()); //still old position
}

/**
 * tests move position one direction
 */
TEST_F(PECPositionFilterSpeedTest, move_position_one_direction)
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::TValue            DISTANCE_1M = 1.0; //1 meter
   const PE::TValue            DISTANCE_2M = 2.0; //2 meter
   const PE::SPosition           POS_START = PE::SPosition(52.0, 10.0);
   const PE::SPosition   POS_START_PLUS_1M = PE::TOOLS::ToPosition(POS_START,DISTANCE_1M,0);
   const PE::SPosition   POS_START_PLUS_2M = PE::TOOLS::ToPosition(POS_START,DISTANCE_2M,0);
   
   PE::CPositionFilterSpeed filter(SPEED_LIMIT_1M_PER_SEC);
   EXPECT_FALSE(filter.GetPosition().IsValid());

   filter.AddPosition(10.000, POS_START );
   EXPECT_TRUE(filter.GetPosition().IsValid());
   EXPECT_EQ(10.000, filter.GetTimestamp());
   EXPECT_EQ(POS_START, filter.GetPosition());

   //distance 1 meter per 1 second from original - no change
   filter.AddPosition(11.000, POS_START_PLUS_1M);
   EXPECT_EQ(11.000, filter.GetTimestamp());
   EXPECT_EQ(POS_START, filter.GetPosition());

   //distance 2 meter per 1 second from original - changed
   filter.AddPosition(12.000, POS_START_PLUS_2M);
   EXPECT_EQ(12.000, filter.GetTimestamp());
   EXPECT_EQ(POS_START_PLUS_2M, filter.GetPosition());
}

/**
 * tests move position around
 */
TEST_F(PECPositionFilterSpeedTest, move_position_around)
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::TValue            DISTANCE_1M = 1.0; //1 meter
   const PE::TValue            DISTANCE_2M = 2.0; //2 meter
   const PE::SPosition           POS_START = PE::SPosition(52.0, 10.0);
   const PE::SPosition   POS_START_PLUS_1M_UP    = PE::TOOLS::ToPosition(POS_START,DISTANCE_1M,0);
   const PE::SPosition   POS_START_PLUS_1M_LEFT  = PE::TOOLS::ToPosition(POS_START,DISTANCE_1M,270);
   const PE::SPosition   POS_START_PLUS_1M_RIGHT = PE::TOOLS::ToPosition(POS_START,DISTANCE_1M,90);
   const PE::SPosition   POS_START_PLUS_1M_DOWN  = PE::TOOLS::ToPosition(POS_START,DISTANCE_1M,180);
   const PE::SPosition   POS_START_PLUS_2M_UP    = PE::TOOLS::ToPosition(POS_START,DISTANCE_2M,0);
   
   PE::CPositionFilterSpeed filter(SPEED_LIMIT_1M_PER_SEC);
   EXPECT_FALSE(filter.GetPosition().IsValid());
   EXPECT_EQ(SPEED_LIMIT_1M_PER_SEC, filter.GetSpeedLimit());

   filter.AddPosition(10.000, POS_START );
   EXPECT_TRUE(filter.GetPosition().IsValid());
   EXPECT_EQ(10.000, filter.GetTimestamp());
   EXPECT_EQ(POS_START, filter.GetPosition());

   //distance 1 meter per 1 second from original - no change
   filter.AddPosition(11.000, POS_START_PLUS_1M_UP);
   EXPECT_EQ(11.000, filter.GetTimestamp());
   EXPECT_EQ(POS_START, filter.GetPosition());

   //distance 1 meter per 1 second from original - no change
   filter.AddPosition(12.000, POS_START_PLUS_1M_DOWN);
   EXPECT_EQ(12.000, filter.GetTimestamp());
   EXPECT_EQ(POS_START, filter.GetPosition());

   //distance 1 meter per 1 second from original - no change
   filter.AddPosition(13.000, POS_START_PLUS_1M_RIGHT);
   EXPECT_EQ(13.000, filter.GetTimestamp());
   EXPECT_EQ(POS_START, filter.GetPosition());

   //distance 1 meter per 1 second from original - no change
   filter.AddPosition(14.000, POS_START_PLUS_1M_LEFT);
   EXPECT_EQ(14.000, filter.GetTimestamp());
   EXPECT_EQ(POS_START, filter.GetPosition());

   //distance 2 meter per 1 second from original - changed
   filter.AddPosition(15.000, POS_START_PLUS_2M_UP);
   EXPECT_EQ(15.000, filter.GetTimestamp());
   EXPECT_EQ(POS_START_PLUS_2M_UP, filter.GetPosition());
}

/**
 * tests move position always above limit
 */
TEST_F(PECPositionFilterSpeedTest, move_position_always_above_limit)
{
   const PE::TValue SPEED_LIMIT_1M_PER_SEC = 1.00000001; //1 m/s - alhorithm makes and error with 7 number after decimal point
   const PE::TValue            DISTANCE_1M = 1.0; //1 meter
   const PE::TValue            DISTANCE_2M = 2.0; //2 meter
   const PE::SPosition           POS_START = PE::SPosition(52.0, 10.0);
   const PE::SPosition               POS_1 = PE::TOOLS::ToPosition(POS_START,DISTANCE_2M,0);
   const PE::SPosition               POS_2 = PE::TOOLS::ToPosition(POS_1,DISTANCE_2M,90);
   const PE::SPosition               POS_3 = PE::TOOLS::ToPosition(POS_2,DISTANCE_2M,180);
   const PE::SPosition               POS_4 = PE::TOOLS::ToPosition(POS_3,DISTANCE_2M,270);
   const PE::SPosition               POS_5 = PE::TOOLS::ToPosition(POS_4,DISTANCE_2M,0);
   
   PE::CPositionFilterSpeed filter(SPEED_LIMIT_1M_PER_SEC);
   EXPECT_FALSE(filter.GetPosition().IsValid());

   filter.AddPosition(10.000, POS_START );
   EXPECT_TRUE(filter.GetPosition().IsValid());
   EXPECT_EQ(10.000, filter.GetTimestamp());
   EXPECT_EQ(POS_START, filter.GetPosition());

   //distance 2 meter per 1 second from original - change
   filter.AddPosition(11.000, POS_1);
   EXPECT_EQ(11.000, filter.GetTimestamp());
   EXPECT_EQ(POS_1, filter.GetPosition());

   //distance 2 meter per 1 second from original - change
   filter.AddPosition(12.000, POS_2);
   EXPECT_EQ(12.000, filter.GetTimestamp());
   EXPECT_EQ(POS_2, filter.GetPosition());

   //distance 2 meter per 1 second from original - change
   filter.AddPosition(13.000, POS_3);
   EXPECT_EQ(13.000, filter.GetTimestamp());
   EXPECT_EQ(POS_3, filter.GetPosition());

   //distance 2 meter per 1 second from original - change
   filter.AddPosition(14.000, POS_4);
   EXPECT_EQ(14.000, filter.GetTimestamp());
   EXPECT_EQ(POS_4, filter.GetPosition());

   //distance 2 meter per 1 second from original - change
   filter.AddPosition(15.000, POS_5);
   EXPECT_EQ(15.000, filter.GetTimestamp());
   EXPECT_EQ(POS_5, filter.GetPosition());
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

