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
 * Unit test of the PECNormalisation class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PECNormalisation.h"

class PECNormalisationTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }
};

/**
 * check set of same values till full reliable
 */
TEST_F(PECNormalisationTest, test_no_init_values_amount_of_same_values_till_full_reliable)
{
   PE::CNormalisation norm = PE::CNormalisation();
   EXPECT_EQ(0.0, norm.GetMean());
   EXPECT_EQ(0.0, norm.GetMld());
   EXPECT_EQ(0.0, norm.GetReliable());

   norm.AddSensor(10.0);
   EXPECT_EQ(0.0, norm.GetMean());
   EXPECT_EQ(0.0, norm.GetMld());
   EXPECT_NEAR(0.0, norm.GetReliable(),0.01);

   norm.AddSensor(10.0);
   EXPECT_EQ(10.0, norm.GetMean());
   EXPECT_EQ(0.0, norm.GetMld());
   EXPECT_NEAR(50.0, norm.GetReliable(),0.01);

   norm.AddSensor(10.0);
   EXPECT_NEAR(66.66, norm.GetReliable(),0.01);

   norm.AddSensor(10.0);
   EXPECT_NEAR(75.00, norm.GetReliable(),0.01);

   norm.AddSensor(10.0);
   EXPECT_NEAR(80.00, norm.GetReliable(),0.01);

   norm.AddSensor(10.0);
   EXPECT_NEAR(83.33, norm.GetReliable(),0.01);

   norm.AddSensor(10.0);
   EXPECT_NEAR(85.71, norm.GetReliable(),0.01);

   norm.AddSensor(10.0);
   EXPECT_NEAR(87.50, norm.GetReliable(),0.01);

   norm.AddSensor(10.0);
   EXPECT_NEAR(88.88, norm.GetReliable(),0.01);

   norm.AddSensor(10.0); //#10
   EXPECT_NEAR(90.00, norm.GetReliable(),0.01);

   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0); //#23
   EXPECT_NEAR(95.65, norm.GetReliable(),0.01);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0); //#30
   EXPECT_NEAR(96.66, norm.GetReliable(),0.01);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0); //#40
   EXPECT_NEAR(97.50, norm.GetReliable(),0.01);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0); //#50
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0); //#60
   EXPECT_NEAR(98.33, norm.GetReliable(),0.01);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0); //#70
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0); //#80
   EXPECT_NEAR(98.75, norm.GetReliable(),0.01);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0); //#90
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0);
   norm.AddSensor(10.0); //#100
   for (int i=100; i<200; i++)
      norm.AddSensor(10.0);
   norm.AddSensor(10.0); //#200
   EXPECT_NEAR(99.50, norm.GetReliable(),0.01);
}

/**
 * check values form set [10.0...10.05]
 * no initial settings
 */
TEST_F(PECNormalisationTest, test_no_init_values_amount_of_values_40)
{
   PE::CNormalisation norm = PE::CNormalisation();
   norm.AddSensor(10.0016967);
   norm.AddSensor(10.017085);
   EXPECT_NEAR(10.009391, norm.GetMean(),    0.000001);
   EXPECT_NEAR( 0.007694, norm.GetMld(),   0.000001);
   EXPECT_NEAR( 0.000000, norm.GetReliable(),0.000001);

   norm.AddSensor(10.01674144);
   norm.AddSensor(10.00577947);
   norm.AddSensor(10.0319618);
   norm.AddSensor(10.02513243);
   norm.AddSensor(10.0049031);
   norm.AddSensor(10.04936262);
   norm.AddSensor(10.02806204);
   norm.AddSensor(10.03876338);
   norm.AddSensor(10.03785415);
   norm.AddSensor(10.02486409);
   EXPECT_NEAR(10.023518, norm.GetMean(),    0.000001);
   EXPECT_NEAR( 0.011265, norm.GetMld(),   0.000001);
   EXPECT_NEAR(64.33328928, norm.GetReliable(),0.000001);
   EXPECT_NEAR(120.2822062, norm.GetAccumulatedValue(),   0.000001);
   EXPECT_NEAR(0.123918864, norm.GetAccumulatedMld(),   0.000001);
   EXPECT_NEAR(771.9994714, norm.GetAccumulatedReliable(),0.000001);
   EXPECT_NEAR(         12, norm.GetSampleCount(),        0.000001);

   norm.AddSensor(10.02808851);
   EXPECT_NEAR(10.023869, norm.GetMean(),    0.000001);
   EXPECT_NEAR( 0.010678, norm.GetMld(),   0.000001);
   EXPECT_NEAR(66.823569, norm.GetReliable(),0.000001);

   norm.AddSensor(10.02160893);
   norm.AddSensor(10.0345712);
   norm.AddSensor(10.00268356);
   norm.AddSensor(10.01072438);
   norm.AddSensor(10.00000485);
   norm.AddSensor(10.04422098);
   norm.AddSensor(10.01388693);
   norm.AddSensor(10.036653);
   norm.AddSensor(10.04984578);
   norm.AddSensor(10.014407);
   norm.AddSensor(10.00288788);
   norm.AddSensor(10.04565213);
   norm.AddSensor(10.02560543);
   EXPECT_NEAR(10.023579, norm.GetMean(),    0.000001);
   EXPECT_NEAR( 0.012654, norm.GetMld(),   0.000001);
   EXPECT_NEAR(80.086986, norm.GetReliable(),0.000001);
   
   norm.AddSensor(10.04907064);
   norm.AddSensor(10.01231752);
   norm.AddSensor(10.03113507);
   norm.AddSensor(10.01523936);
   norm.AddSensor(10.02933993);
   norm.AddSensor(10.03231828);
   norm.AddSensor(10.01362208);
   norm.AddSensor(10.00672482);
   norm.AddSensor(10.03193204);
   norm.AddSensor(10.00515203);
   norm.AddSensor(10.02589951);
   norm.AddSensor(10.01857826);
   norm.AddSensor(10.03582339);
   norm.AddSensor(10.04985731);
   EXPECT_NEAR(10.024251, norm.GetMean(),    0.000001);
   EXPECT_NEAR( 0.012306, norm.GetMld(),   0.000001);
   EXPECT_NEAR(86.035287, norm.GetReliable(),0.000001);
}

/**
 * check values form set [10.0...10.05]
 * Initial settings are correct
 * 
 */
TEST_F(PECNormalisationTest, test_set_init_and_amount_of_values_28)
{
   PE::CNormalisation norm = PE::CNormalisation(120.2822062, 0.123918864, 771.9994714, 12);
   norm.AddSensor(10.02808851);
   EXPECT_NEAR(10.023869, norm.GetMean(),    0.000001);
   EXPECT_NEAR( 0.010678, norm.GetMld(),   0.000001);
   EXPECT_NEAR(66.823569, norm.GetReliable(),0.000001);

   norm.AddSensor(10.02160893);
   norm.AddSensor(10.0345712);
   norm.AddSensor(10.00268356);
   norm.AddSensor(10.01072438);
   norm.AddSensor(10.00000485);
   norm.AddSensor(10.04422098);
   norm.AddSensor(10.01388693);
   norm.AddSensor(10.036653);
   norm.AddSensor(10.04984578);
   norm.AddSensor(10.014407);
   norm.AddSensor(10.00288788);
   norm.AddSensor(10.04565213);
   norm.AddSensor(10.02560543);
   EXPECT_NEAR(10.023579, norm.GetMean(),    0.000001);
   EXPECT_NEAR( 0.012654, norm.GetMld(),   0.000001);
   EXPECT_NEAR(80.086986, norm.GetReliable(),0.000001);
   
   norm.AddSensor(10.04907064);
   norm.AddSensor(10.01231752);
   norm.AddSensor(10.03113507);
   norm.AddSensor(10.01523936);
   norm.AddSensor(10.02933993);
   norm.AddSensor(10.03231828);
   norm.AddSensor(10.01362208);
   norm.AddSensor(10.00672482);
   norm.AddSensor(10.03193204);
   norm.AddSensor(10.00515203);
   norm.AddSensor(10.02589951);
   norm.AddSensor(10.01857826);
   norm.AddSensor(10.03582339);
   norm.AddSensor(10.04985731);
   EXPECT_NEAR(10.024251, norm.GetMean(),    0.000001);
   EXPECT_NEAR( 0.012306, norm.GetMld(),   0.000001);
   EXPECT_NEAR(86.035287, norm.GetReliable(),0.000001);
}

/**
 * test invalid sample count
 * 
 */
TEST_F(PECNormalisationTest, test_invalid_sample_count)
{
   PE::CNormalisation norm = PE::CNormalisation(0.0, 0.0, 100, 0);
   norm.AddSensor(1.00);
   EXPECT_NEAR( 0.0, norm.GetMean(), 0.01);
   EXPECT_NEAR( 0.0, norm.GetMld(), 0.01);
   EXPECT_NEAR( 0.0, norm.GetReliable(),0.01);
   EXPECT_NEAR( 1.0, norm.GetAccumulatedValue(),0.01);
   EXPECT_NEAR( 0.0, norm.GetAccumulatedMld(),0.01);
   EXPECT_NEAR( 0.0, norm.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (   1, norm.GetSampleCount());

   norm.AddSensor(2.00);
   EXPECT_NEAR( 1.5, norm.GetMean(), 0.01);
   EXPECT_NEAR( 0.5, norm.GetMld(), 0.01);
   EXPECT_NEAR( 0.0, norm.GetReliable(),0.01);
   EXPECT_NEAR( 3.0, norm.GetAccumulatedValue(),0.01);
   EXPECT_NEAR( 0.5, norm.GetAccumulatedMld(),0.01);
   EXPECT_NEAR( 0.0, norm.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (   2, norm.GetSampleCount());

}

/**
 * test invalid accumulated reliable value
 * 
 */
TEST_F(PECNormalisationTest, test_invalid_accumulated_reliable_value)
{
   PE::CNormalisation norm = PE::CNormalisation(0.0, 0.0, -100, 1);
   norm.AddSensor(1.00);
   EXPECT_NEAR( 0.0, norm.GetMean(), 0.01);
   EXPECT_NEAR( 0.0, norm.GetMld(), 0.01);
   EXPECT_NEAR( 0.0, norm.GetReliable(),0.01);
   EXPECT_NEAR( 1.0, norm.GetAccumulatedValue(),0.01);
   EXPECT_NEAR( 0.0, norm.GetAccumulatedMld(),0.01);
   EXPECT_NEAR( 0.0, norm.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (   1, norm.GetSampleCount());

   norm.AddSensor(2.00);
   EXPECT_NEAR( 1.5, norm.GetMean(), 0.01);
   EXPECT_NEAR( 0.5, norm.GetMld(), 0.01);
   EXPECT_NEAR( 0.0, norm.GetReliable(),0.01);
   EXPECT_NEAR( 3.0, norm.GetAccumulatedValue(),0.01);
   EXPECT_NEAR( 0.5, norm.GetAccumulatedMld(),0.01);
   EXPECT_NEAR( 0.0, norm.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (   2, norm.GetSampleCount());

}

/**
 * test zero accumulated reliable value
 * 
 */
TEST_F(PECNormalisationTest, test_zero_accumulated_reliable_value)
{
   PE::CNormalisation norm = PE::CNormalisation(0.0, 0.0, 0, 1);
   norm.AddSensor(1.00);
   EXPECT_NEAR( 0.5, norm.GetMean(), 0.01);
   EXPECT_NEAR( 0.5, norm.GetMld(), 0.01);
   EXPECT_NEAR( 0.0, norm.GetReliable(),0.01);
   EXPECT_NEAR( 1.0, norm.GetAccumulatedValue(),0.01);
   EXPECT_NEAR( 0.5, norm.GetAccumulatedMld(),0.01);
   EXPECT_NEAR( 0.0, norm.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (   2, norm.GetSampleCount());

   norm.AddSensor(2.00);
   EXPECT_NEAR(  1.0, norm.GetMean(), 0.01);
   EXPECT_NEAR( 0.75, norm.GetMld(), 0.01);
   EXPECT_NEAR(11.11, norm.GetReliable(),0.01);
   EXPECT_NEAR(  3.0, norm.GetAccumulatedValue(),0.01);
   EXPECT_NEAR(  1.5, norm.GetAccumulatedMld(),0.01);
   EXPECT_NEAR(33.33, norm.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (    3, norm.GetSampleCount());

}

/**
 * test zero accumulated reliable zero value
 * 
 */
TEST_F(PECNormalisationTest, test_zero_accumulated_reliable_zero_value)
{
   PE::CNormalisation norm = PE::CNormalisation(0.0, 0.0, 0, 1);
   norm.AddSensor(0.00);
   EXPECT_NEAR( 0.0, norm.GetMean(), 0.01);
   EXPECT_NEAR( 0.0, norm.GetMld(), 0.01);
   EXPECT_NEAR(50.0, norm.GetReliable(),0.01);
   EXPECT_NEAR( 0.0, norm.GetAccumulatedValue(),0.01);
   EXPECT_NEAR( 0.0, norm.GetAccumulatedMld(),0.01);
   EXPECT_NEAR( 100.0, norm.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (   2, norm.GetSampleCount());
}


/**
 * test zero accumulated reliable zero value
 * 
 */
TEST_F(PECNormalisationTest, test_zero_all_accumulated_and_values_around_zero)
{
   PE::CNormalisation norm = PE::CNormalisation(0.0, 0.0, 0, 0);
   norm.AddSensor(0.00);
   EXPECT_NEAR( 0.0, norm.GetMean(), 0.001);
   EXPECT_NEAR( 0.0, norm.GetMld(), 0.001);
   EXPECT_NEAR( 0.0, norm.GetReliable(),0.001);
   norm.AddSensor(0.00);
   EXPECT_NEAR( 0.0, norm.GetMean(), 0.001);
   EXPECT_NEAR( 0.0, norm.GetMld(), 0.001);
   EXPECT_NEAR(50.00, norm.GetReliable(),0.001);
   norm.AddSensor(0.00);
   EXPECT_NEAR( 0.0, norm.GetMean(), 0.001);
   EXPECT_NEAR( 0.0, norm.GetMld(), 0.001);
   EXPECT_NEAR(66.667, norm.GetReliable(),0.001);
   norm.AddSensor(+0.1);
   EXPECT_NEAR( 0.025, norm.GetMean(), 0.001);
   EXPECT_NEAR( 0.025, norm.GetMld(), 0.001);
   EXPECT_NEAR(50.000, norm.GetReliable(),0.001);
   norm.AddSensor(-0.1);
   EXPECT_NEAR( 0.000, norm.GetMean(), 0.001);
   EXPECT_NEAR( 0.044, norm.GetMld(), 0.001);
   EXPECT_NEAR(48.571, norm.GetReliable(),0.001);
   norm.AddSensor(-0.15);
   EXPECT_NEAR(-0.025, norm.GetMean(), 0.001);
   EXPECT_NEAR( 0.060, norm.GetMld(), 0.001);
   EXPECT_NEAR(50.198, norm.GetReliable(),0.001);
   norm.AddSensor(+0.15);
   EXPECT_NEAR( 0.000, norm.GetMean(), 0.001);
   EXPECT_NEAR( 0.075, norm.GetMld(), 0.001);
   EXPECT_NEAR(52.551, norm.GetReliable(),0.001);
}


/**
 * test big precalibrated values
 * 
 */
TEST_F(PECNormalisationTest, test_big_precalibrated_values)
{
   //start accumulated is known
   PE::CNormalisation norm1 = PE::CNormalisation(123000.0, 0.0, 100000, 1000); //123.00 x 1000
   norm1.AddSensor(123.00);
   EXPECT_NEAR(   123.00, norm1.GetMean(), 0.01);
   EXPECT_NEAR(     0.00, norm1.GetMld(), 0.01);
   EXPECT_NEAR(   100.00, norm1.GetReliable(),0.01);
   EXPECT_NEAR(123123.00, norm1.GetAccumulatedValue(),0.01);
   EXPECT_NEAR(     0.00, norm1.GetAccumulatedMld(),0.01);
   EXPECT_NEAR(100100.00, norm1.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (  1001, norm1.GetSampleCount());

   norm1.AddSensor(110.00);
   EXPECT_NEAR(   122.98703, norm1.GetMean(), 0.0001);
   EXPECT_NEAR(     0.01297, norm1.GetMld(), 0.0001);
   EXPECT_NEAR(   99.900199, norm1.GetReliable(),0.0001);
   EXPECT_NEAR(123233.00000, norm1.GetAccumulatedValue(),0.0001);
   EXPECT_NEAR(    12.98703, norm1.GetAccumulatedMld(),0.0001);
   EXPECT_NEAR(100100.00000, norm1.GetAccumulatedReliable(),0.0001);
   EXPECT_EQ  (  1002, norm1.GetSampleCount());
}

/**
 * test value without changes at beginning
 * 
 */
TEST_F(PECNormalisationTest, test_value_without_changes_at_beginning)
{
   //start accumulated is known
   PE::CNormalisation norm1 = PE::CNormalisation(123.0, 0.0, 100, 1);
   norm1.AddSensor(123.00);
   EXPECT_NEAR( 123.0, norm1.GetMean(), 0.01);
   EXPECT_NEAR(   0.0, norm1.GetMld(), 0.01);
   EXPECT_NEAR( 100.0, norm1.GetReliable(),0.01);
   EXPECT_NEAR( 246.0, norm1.GetAccumulatedValue(),0.01);
   EXPECT_NEAR(   0.0, norm1.GetAccumulatedMld(),0.01);
   EXPECT_NEAR( 200.0, norm1.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (     2, norm1.GetSampleCount());

   //all start values are invalid
   PE::CNormalisation norm2 = PE::CNormalisation();
   norm2.AddSensor(123.00);
   EXPECT_NEAR(   0.0, norm2.GetMean(), 0.01);
   EXPECT_NEAR(   0.0, norm2.GetMld(), 0.01);
   EXPECT_NEAR(   0.0, norm2.GetReliable(),0.01);
   EXPECT_NEAR( 123.0, norm2.GetAccumulatedValue(),0.01);
   EXPECT_NEAR(   0.0, norm2.GetAccumulatedMld(),0.01);
   EXPECT_NEAR(   0.0, norm2.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (     1, norm2.GetSampleCount());
   norm2.AddSensor(123.00);
   EXPECT_NEAR( 123.0, norm2.GetMean(), 0.01);
   EXPECT_NEAR(   0.0, norm2.GetMld(), 0.01);
   EXPECT_NEAR(  50.0, norm2.GetReliable(),0.01);
   EXPECT_NEAR( 246.0, norm2.GetAccumulatedValue(),0.01);
   EXPECT_NEAR(   0.0, norm2.GetAccumulatedMld(),0.01);
   EXPECT_NEAR( 100.0, norm2.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (     2, norm2.GetSampleCount());
   norm2.AddSensor(123.00);
   EXPECT_NEAR( 123.0, norm2.GetMean(), 0.01);
   EXPECT_NEAR(   0.0, norm2.GetMld(), 0.01);
   EXPECT_NEAR( 66.66, norm2.GetReliable(),0.01);
   EXPECT_NEAR( 369.0, norm2.GetAccumulatedValue(),0.01);
   EXPECT_NEAR(   0.0, norm2.GetAccumulatedMld(),0.01);
   EXPECT_NEAR( 200.0, norm2.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (     3, norm2.GetSampleCount());
   norm2.AddSensor(123.00);
   EXPECT_NEAR( 123.0, norm2.GetMean(), 0.01);
   EXPECT_NEAR(   0.0, norm2.GetMld(), 0.01);
   EXPECT_NEAR(  75.0, norm2.GetReliable(),0.01);
   EXPECT_NEAR( 492.0, norm2.GetAccumulatedValue(),0.01);
   EXPECT_NEAR(   0.0, norm2.GetAccumulatedMld(),0.01);
   EXPECT_NEAR( 300.0, norm2.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (     4, norm2.GetSampleCount());
}


/**
 * test reduced reliable value by big difference
 * 
 */
TEST_F(PECNormalisationTest, test_reduced_reliable_value_by_big_difference)
{
   PE::CNormalisation norm = PE::CNormalisation(0.0, 0.0, 0, 1);
   norm.AddSensor(1.00);
   EXPECT_NEAR( 0.5, norm.GetMean(), 0.01);
   EXPECT_NEAR( 0.5, norm.GetMld(), 0.01);
   EXPECT_NEAR( 0.0, norm.GetReliable(),0.01);
   EXPECT_NEAR( 1.0, norm.GetAccumulatedValue(),0.01);
   EXPECT_NEAR( 0.5, norm.GetAccumulatedMld(),0.01);
   EXPECT_NEAR( 0.0, norm.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (   2, norm.GetSampleCount());

   norm.AddSensor(2.00);
   EXPECT_NEAR(  1.0, norm.GetMean(), 0.01);
   EXPECT_NEAR( 0.75, norm.GetMld(), 0.01);
   EXPECT_NEAR(11.11, norm.GetReliable(),0.01);
   EXPECT_NEAR(  3.0, norm.GetAccumulatedValue(),0.01);
   EXPECT_NEAR(  1.5, norm.GetAccumulatedMld(),0.01);
   EXPECT_NEAR(33.33, norm.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (    3, norm.GetSampleCount());

   norm.AddSensor(2.00);
   EXPECT_NEAR(  1.25, norm.GetMean(), 0.01);
   EXPECT_NEAR(  0.75, norm.GetMld(), 0.01);
   EXPECT_NEAR( 25.00, norm.GetReliable(),0.01);
   EXPECT_NEAR(   5.0, norm.GetAccumulatedValue(),0.01);
   EXPECT_NEAR(  2.25, norm.GetAccumulatedMld(),0.01);
   EXPECT_NEAR(100.00, norm.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (     4, norm.GetSampleCount());

   norm.AddSensor(1.00);
   EXPECT_NEAR(  1.20, norm.GetMean(), 0.01);
   EXPECT_NEAR(  0.61, norm.GetMld(), 0.01);
   EXPECT_NEAR( 38.36, norm.GetReliable(),0.01);
   EXPECT_NEAR(   6.0, norm.GetAccumulatedValue(),0.01);
   EXPECT_NEAR(  2.45, norm.GetAccumulatedMld(),0.01);
   EXPECT_NEAR(191.84, norm.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (     5, norm.GetSampleCount());

   //big jump UP by implausible value reliable has to be less then before
   norm.AddSensor(10.00);
   EXPECT_NEAR(  2.66, norm.GetMean(), 0.01);
   EXPECT_NEAR(  1.96, norm.GetMld(), 0.01); //ACCURACY decreased
   EXPECT_NEAR( 36.14, norm.GetReliable(),0.01); //RELIABLE decriased
   EXPECT_NEAR(  16.0, norm.GetAccumulatedValue(),0.01);
   EXPECT_NEAR(  9.78, norm.GetAccumulatedMld(),0.01);
   EXPECT_NEAR(216.88, norm.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (     6, norm.GetSampleCount());

   //value again in reasonable range 
   norm.AddSensor(2.00);
   EXPECT_NEAR(  2.57, norm.GetMean(), 0.01);
   EXPECT_NEAR(  1.72, norm.GetMld(), 0.01); //accuracy INCREASED
   EXPECT_NEAR( 44.48, norm.GetReliable(),0.01); //reliable INCREASED
   EXPECT_NEAR(  18.0, norm.GetAccumulatedValue(),0.01);
   EXPECT_NEAR( 10.35, norm.GetAccumulatedMld(),0.01);
   EXPECT_NEAR(311.36, norm.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (     7, norm.GetSampleCount());

   //big jump DOWN by implausible value reliable has to be less then before
   norm.AddSensor(-10.00);
   EXPECT_NEAR(  1.00, norm.GetMean(), 0.01);
   EXPECT_NEAR(  3.05, norm.GetMld(), 0.01); //ACCURACY decreased
   EXPECT_NEAR( 44.98, norm.GetReliable(),0.01); //RELIABLE decriased
   EXPECT_NEAR(   8.0, norm.GetAccumulatedValue(),0.01);
   EXPECT_NEAR( 21.35, norm.GetAccumulatedMld(),0.01);
   EXPECT_NEAR(359.85, norm.GetAccumulatedReliable(),0.01);
   EXPECT_EQ  (     8, norm.GetSampleCount());

}

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

