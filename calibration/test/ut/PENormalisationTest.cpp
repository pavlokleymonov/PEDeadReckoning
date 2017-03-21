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
 * Unit test of the PENormalisationTest class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PENormalisation.h"

class PENormalisationTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }
};

/**
 * tests +/-100 init value 0 normalization value 0
 */
TEST_F(PENormalisationTest, sensor_plus_minus_100_test)
{
   PE::normalisation norm(0);

   norm.add_sensor_steps(100);
   EXPECT_NEAR(0.0,norm.get_null_reliable(),0.01);
   EXPECT_NEAR(100.0,norm.get_null(),0.01);
   
   norm.add_sensor_steps(90);
   EXPECT_NEAR(95.0,norm.get_null_reliable(),0.01);
   EXPECT_NEAR(95.0,norm.get_null(),0.01);

   norm.add_sensor_steps(80);
   EXPECT_NEAR(94.74,norm.get_null_reliable(),0.01);
   EXPECT_NEAR(90.0,norm.get_null(),0.01);

   norm.add_sensor_steps(-100);
   EXPECT_NEAR(47.22,norm.get_null_reliable(),0.01);
   EXPECT_NEAR(42.5,norm.get_null(),0.01);

   norm.add_sensor_steps(-100);
   EXPECT_NEAR(32.94,norm.get_null_reliable(),0.01);
   EXPECT_NEAR(14.0,norm.get_null(),0.01);

   norm.add_sensor_steps(-40);
   EXPECT_NEAR(35.71,norm.get_null_reliable(),0.01);
   EXPECT_NEAR(5.0,norm.get_null(),0.01);

   norm.add_sensor_steps(-25);
   EXPECT_NEAR(14.29,norm.get_null_reliable(),0.01);
   EXPECT_NEAR(0.71,norm.get_null(),0.01);

   norm.add_sensor_steps(-5);
   EXPECT_NEAR(0.0,norm.get_null_reliable(),0.01);
   EXPECT_NEAR(0.0,norm.get_null(),0.01);

   norm.add_sensor_steps(0);
   EXPECT_NEAR(100.0,norm.get_null_reliable(),0.01);
   EXPECT_NEAR(0.0,norm.get_null(),0.01);
}

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

