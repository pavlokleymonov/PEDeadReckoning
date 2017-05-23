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
 */
TEST_F(PENormalisationTest, no_init_values_amount_of_same_values_3)
{
   PE::normalisation norm = PE::normalisation();
   EXPECT_EQ(0.0, norm.get_mean());
   EXPECT_EQ(0.0, norm.get_sigma());
   EXPECT_EQ(0.0, norm.get_reliable());

   norm.add_sensor(10.0);
   EXPECT_EQ(0.0, norm.get_mean());
   EXPECT_EQ(0.0, norm.get_sigma());
   EXPECT_EQ(0.0, norm.get_reliable());

   norm.add_sensor(10.0);
   EXPECT_EQ(10.0, norm.get_mean());
   EXPECT_EQ(0.0, norm.get_sigma());
   EXPECT_EQ(100.0, norm.get_reliable());

   norm.add_sensor(10.000000);
   EXPECT_EQ(10.0, norm.get_mean());
   EXPECT_EQ(0.0, norm.get_sigma());
   EXPECT_EQ(100.0, norm.get_reliable());
}

/**
 * check mean value form set [10.0...10.05]
 * init_mean = 0 - no init expected value
 * init_sigma = 0 - no init standart deviation value
 * init_reliable = 0% - uncallibrated value
 */
TEST_F(PENormalisationTest, no_init_values_amount_of_values_40)
{
   PE::normalisation norm = PE::normalisation();
   norm.add_sensor(10.0016967);
   norm.add_sensor(10.017085);
   EXPECT_NEAR(10.009391, norm.get_mean(),    0.000001);
   EXPECT_NEAR( 0.007694, norm.get_sigma(),   0.000001);
   EXPECT_NEAR( 0.000000, norm.get_reliable(),0.000001);

   norm.add_sensor(10.01674144);
   norm.add_sensor(10.00577947);
   norm.add_sensor(10.0319618);
   norm.add_sensor(10.02513243);
   norm.add_sensor(10.0049031);
   norm.add_sensor(10.04936262);
   norm.add_sensor(10.02806204);
   norm.add_sensor(10.03876338);
   norm.add_sensor(10.03785415);
   norm.add_sensor(10.02486409);
   EXPECT_NEAR(10.023518, norm.get_mean(),    0.000001);
   EXPECT_NEAR( 0.011265, norm.get_sigma(),   0.000001);
   EXPECT_NEAR(70.181770, norm.get_reliable(),0.000001);

   norm.add_sensor(10.02808851);
   EXPECT_NEAR(10.023869, norm.get_mean(),    0.000001);
   EXPECT_NEAR( 0.010678, norm.get_sigma(),   0.000001);
   EXPECT_NEAR(72.392200, norm.get_reliable(),0.000001);

   norm.add_sensor(10.02160893);
   norm.add_sensor(10.0345712);
   norm.add_sensor(10.00268356);
   norm.add_sensor(10.01072438);
   norm.add_sensor(10.00000485);
   norm.add_sensor(10.04422098);
   norm.add_sensor(10.01388693);
   norm.add_sensor(10.036653);
   norm.add_sensor(10.04984578);
   norm.add_sensor(10.014407);
   norm.add_sensor(10.00288788);
   norm.add_sensor(10.04565213);
   norm.add_sensor(10.02560543);
   EXPECT_NEAR(10.023579, norm.get_mean(),    0.000001);
   EXPECT_NEAR( 0.012654, norm.get_sigma(),   0.000001);
   EXPECT_NEAR(83.290465, norm.get_reliable(),0.000001);
   
   norm.add_sensor(10.04907064);
   norm.add_sensor(10.01231752);
   norm.add_sensor(10.03113507);
   norm.add_sensor(10.01523936);
   norm.add_sensor(10.02933993);
   norm.add_sensor(10.03231828);
   norm.add_sensor(10.01362208);
   norm.add_sensor(10.00672482);
   norm.add_sensor(10.03193204);
   norm.add_sensor(10.00515203);
   norm.add_sensor(10.02589951);
   norm.add_sensor(10.01857826);
   norm.add_sensor(10.03582339);
   norm.add_sensor(10.04985731);
   EXPECT_NEAR(10.024251, norm.get_mean(),    0.000001);
   EXPECT_NEAR( 0.012306, norm.get_sigma(),   0.000001);
   EXPECT_NEAR(88.241321, norm.get_reliable(),0.000001);
}

/**
 * check mean value form set [10.0...10.05]
 * init_mean = 10.023518
 * init_sigma = 0.011265
 * init_reliable = 70.181770 %
 */
TEST_F(PENormalisationTest, set_init_and_amount_of_values_28)
{
   PE::normalisation norm = PE::normalisation(10.023518, 0.011265, 70.181770);
   EXPECT_NEAR(10.023518, norm.get_mean(),    0.000001);
   EXPECT_NEAR( 0.011265, norm.get_sigma(),   0.000001);
   EXPECT_NEAR(70.181770, norm.get_reliable(),0.000001);

   /*TODO: finished unittest investigation
   norm.add_sensor(10.02808851);
   EXPECT_NEAR(10.025803, norm.get_mean(),    0.000001);
   EXPECT_NEAR( 0.006776, norm.get_sigma(),   0.000001);
   EXPECT_NEAR(68.223777, norm.get_reliable(),0.000001);

   norm.add_sensor(10.02160893);
   norm.add_sensor(10.0345712);
   norm.add_sensor(10.00268356);
   norm.add_sensor(10.01072438);
   norm.add_sensor(10.00000485);
   norm.add_sensor(10.04422098);
   norm.add_sensor(10.01388693);
   norm.add_sensor(10.036653);
   norm.add_sensor(10.04984578);
   norm.add_sensor(10.014407);
   norm.add_sensor(10.00288788);
   norm.add_sensor(10.04565213);
   norm.add_sensor(10.02560543);
   EXPECT_NEAR(10.023579, norm.get_mean(),    0.000001);
   EXPECT_NEAR( 0.012654, norm.get_sigma(),   0.000001);
   EXPECT_NEAR(83.290465, norm.get_reliable(),0.000001);
   
   norm.add_sensor(10.04907064);
   norm.add_sensor(10.01231752);
   norm.add_sensor(10.03113507);
   norm.add_sensor(10.01523936);
   norm.add_sensor(10.02933993);
   norm.add_sensor(10.03231828);
   norm.add_sensor(10.01362208);
   norm.add_sensor(10.00672482);
   norm.add_sensor(10.03193204);
   norm.add_sensor(10.00515203);
   norm.add_sensor(10.02589951);
   norm.add_sensor(10.01857826);
   norm.add_sensor(10.03582339);
   norm.add_sensor(10.04985731);
   EXPECT_NEAR(10.024251, norm.get_mean(),    0.000001);
   EXPECT_NEAR( 0.012306, norm.get_sigma(),   0.000001);
   EXPECT_NEAR(88.241321, norm.get_reliable(),0.000001);
   */
}

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

