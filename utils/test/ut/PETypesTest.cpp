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
#include "PETypes.h"

class PETypesTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }
};

//Test TPosition structure
TEST_F(PETypesTest, TPosition_test)
{
   //empty constructor test
   PE::TPosition pos1;
   //EXPECT_EQ(0.0, pos1.Latitude);
   //EXPECT_EQ(0.0, pos1.Longitude);
   EXPECT_FALSE(pos1.is_valid());
   
   pos1.Latitude = 1.0;
   EXPECT_FALSE(pos1.is_valid());

   pos1.Longitude = 1.0;
   EXPECT_TRUE(pos1.is_valid());
   
   //useful constructor test
   PE::TPosition pos2(2.0,3.0);
   EXPECT_TRUE(pos2.is_valid());
   EXPECT_EQ(2.0, pos2.Latitude);
   EXPECT_EQ(3.0, pos2.Longitude);
}

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

