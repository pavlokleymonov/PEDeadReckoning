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
 * check set of same values
 * init_mean = 0 - no init expected value
 * init_sigma = 0 - no init standart deviation value
 * init_reliable = 0% - uncallibrated value
 * minimum smaples count is 1
 */
TEST_F(PENormalisationTest, no_init_values_count_is_1_amount_of_same_values_5)
{
   PE::normalisation norm(1);
   norm.add_sensor(10.000000);
   EXPECT_NEAR(   0.0, norm.get_mean() ,0.00000001);
   EXPECT_NEAR(   0.0, norm.get_sigma(),0.0000000001);
   EXPECT_EQ  (     0, norm.get_reliable());
   norm.add_sensor(10.000000);
   EXPECT_NEAR(  10.0, norm.get_mean() ,0.00000001);
   EXPECT_NEAR(   0.0, norm.get_sigma(),0.0000000001);
   EXPECT_EQ  (   100, norm.get_reliable());
   norm.add_sensor(10.000000);
   norm.add_sensor(10.000000);
   norm.add_sensor(10.000000);
}
/**
 * check mean value form set [10.0...10.000050]
 * init_mean = 0 - no init expected value
 * init_sigma = 0 - no init standart deviation value
 * init_reliable = 0% - uncallibrated value
 * minimum smaples count is 0
 */
TEST_F(PENormalisationTest, no_init_values_count_is_0_amount_of_values_22)
{
   PE::normalisation norm(0);

   norm.add_sensor(10.000000);
   norm.add_sensor(10.000050);
   norm.add_sensor(10.000000);
   norm.add_sensor(10.000025);
   norm.add_sensor(10.000020);
   norm.add_sensor(10.000030);
   norm.add_sensor(10.000010);
   norm.add_sensor(10.000030);
   norm.add_sensor(10.000045);
   norm.add_sensor(10.000050);
   norm.add_sensor(10.000045);
   norm.add_sensor(10.000030);
   norm.add_sensor(10.000010);
   norm.add_sensor(10.000000);
   norm.add_sensor(10.000010);
   norm.add_sensor(10.000040);
   norm.add_sensor(10.000000);
   norm.add_sensor(10.000027);
   norm.add_sensor(10.000025);
   norm.add_sensor(10.000028);

   norm.add_sensor(10.000024);
   EXPECT_NEAR(  10.00002376, norm.get_mean() ,0.00000001);
   EXPECT_NEAR( 0.0000122340, norm.get_sigma(),0.0000000001);
   EXPECT_EQ  (          100, norm.get_reliable());


   norm.add_sensor(10.000026);
   EXPECT_NEAR(  10.00002386, norm.get_mean() ,0.00000001);
   EXPECT_NEAR( 0.0000117532, norm.get_sigma(),0.0000000001);
   EXPECT_EQ  (           99, norm.get_reliable());
}

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

