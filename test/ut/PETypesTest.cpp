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
 * Unit test of the PE::Types namespace.
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


//Test isnan
TEST_F(PETypesTest, isnan_test)
{
   EXPECT_TRUE( PE::isnan(std::numeric_limits<double>::quiet_NaN()) );
   EXPECT_FALSE( PE::isnan(PE::EPSILON) );
   EXPECT_FALSE( PE::isnan(PE::MAX_VALUE) );
   EXPECT_FALSE( PE::isnan(-PE::MAX_VALUE) );
   EXPECT_FALSE( PE::isnan(0.0) );
}


//Test isepsilon
TEST_F(PETypesTest, isepsilon_test)
{
   EXPECT_TRUE ( PE::isepsilon(0.0) );
   EXPECT_TRUE ( PE::isepsilon( PE::EPSILON - std::numeric_limits<double>::epsilon()) );
   EXPECT_TRUE ( PE::isepsilon(-PE::EPSILON + std::numeric_limits<double>::epsilon()) );

   EXPECT_FALSE( PE::isepsilon( PE::EPSILON) );
   EXPECT_FALSE( PE::isepsilon(-PE::EPSILON) );
   EXPECT_FALSE( PE::isepsilon( PE::MAX_VALUE) );
   EXPECT_FALSE( PE::isepsilon(-PE::MAX_VALUE) );
   EXPECT_FALSE( PE::isepsilon(std::numeric_limits<double>::quiet_NaN()) );
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
