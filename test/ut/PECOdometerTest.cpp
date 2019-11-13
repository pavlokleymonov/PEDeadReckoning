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
 * Unit test of the PE::COdometer class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PECOdometer.h"

class PECOdometerTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }
};


/**
 * checks initialization staff
 */
TEST_F(PECOdometerTest, test_init_all_settings_are_correct)
{
   PE::TValue       ODO_INTERVAL_50MS = 0.050;
   PE::TValue    SPEED_INTERVAL_200MS = 0.200;
   PE::TValue    BIAS_LIMIT_99PROCENT = 99.00;
   PE::TValue   SCALE_LIMIT_99PROCENT = 99.00;
   uint32_t   SPEED_ACCURACY_RATIO_x2 = 2;
   uint32_t SPEED_CALIBRATION_COUNT_1 = 1;

   PE::COdometer odo;

   EXPECT_EQ   ( 0.0, odo.GetOdoTimeStamp() );

   //try to add before init
   odo.AddSpeed(1.000, 3.6, 0.01);
   odo.AddOdo  (1.001, 0.36, true);
   odo.AddOdo  (1.051, 0.36, true);
   odo.AddOdo  (1.101, 0.36, true);
   odo.AddOdo  (1.151, 0.36, true);
   odo.AddSpeed(1.200, 3.6, 0.01);
   EXPECT_EQ   (0.000, odo.GetOdoTimeStamp() );

   //call init
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_99PROCENT, SCALE_LIMIT_99PROCENT, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));
   odo.AddSpeed(1.000, 3.6, 0.01);
   odo.AddOdo  (1.001, 0.36, true);
   odo.AddOdo  (1.051, 0.36, true);
   odo.AddOdo  (1.101, 0.36, true);
   odo.AddOdo  (1.151, 0.36, true);
   odo.AddSpeed(1.200, 3.6, 0.01);
   EXPECT_EQ   (1.151, odo.GetOdoTimeStamp() );
}


/**
 * checks wrong initialization staff
 */
TEST_F(PECOdometerTest, test_init_wrong_settings)
{
   PE::TValue       ODO_INTERVAL_50MS = 0.050;
   PE::TValue    SPEED_INTERVAL_200MS = 0.200;
   PE::TValue    BIAS_LIMIT_99PROCENT = 99.00;
   PE::TValue   SCALE_LIMIT_99PROCENT = 99.00;
   uint32_t   SPEED_ACCURACY_RATIO_x2 = 2;
   uint32_t SPEED_CALIBRATION_COUNT_1 = 1;

   PE::COdometer odo;

   //call init wrong odo interval
   EXPECT_FALSE(odo.Init(PE::EPSILON, SPEED_INTERVAL_200MS, BIAS_LIMIT_99PROCENT, SCALE_LIMIT_99PROCENT, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));

   //call init wrong speed interval
   EXPECT_FALSE(odo.Init(ODO_INTERVAL_50MS, ODO_INTERVAL_50MS, BIAS_LIMIT_99PROCENT, SCALE_LIMIT_99PROCENT, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));

   //call init wrong bias limit
   EXPECT_FALSE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, PE::EPSILON, SCALE_LIMIT_99PROCENT, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));

   //call init wrong scale limit
   EXPECT_FALSE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_99PROCENT, PE::EPSILON, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));

   //call init wrong accuracy ratio
   EXPECT_FALSE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_99PROCENT, SCALE_LIMIT_99PROCENT, 0, SPEED_CALIBRATION_COUNT_1));

   //call init wrong calibration count
   EXPECT_FALSE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_99PROCENT, SCALE_LIMIT_99PROCENT, SPEED_ACCURACY_RATIO_x2, 0));

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_99PROCENT, SCALE_LIMIT_99PROCENT, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));
}


/**
 * checks calibration
 */
TEST_F(PECOdometerTest, test_calibration_check)
{
   PE::TValue       ODO_INTERVAL_50MS = 0.050;
   PE::TValue    SPEED_INTERVAL_200MS = 0.200;
   PE::TValue    BIAS_LIMIT_26PROCENT = 26.00;
   PE::TValue   SCALE_LIMIT_26PROCENT = 26.00;
   uint32_t   SPEED_ACCURACY_RATIO_x2 = 2;
   uint32_t SPEED_CALIBRATION_COUNT_1 = 1;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));

   odo.AddSpeed(0.999, 10, 0.01);
   odo.AddOdo  (1.000, 250, true);
   odo.AddOdo  (1.050, 250, true);
   odo.AddOdo  (1.100, 250, true);
   odo.AddOdo  (1.150, 250, true);
   odo.AddSpeed(1.199, 10, 0.01);
   odo.AddOdo  (1.200, 250, true);
   odo.AddOdo  (1.250, 500, true);
   odo.AddOdo  (1.300, 500, true);
   odo.AddOdo  (1.350, 500, true);
   odo.AddSpeed(1.399, 20, 0.01);
   odo.AddOdo  (1.400, 500, true);
   odo.AddOdo  (1.450, 50, true);
   odo.AddOdo  (1.500, 50, true);
   odo.AddOdo  (1.550, 50, true);
   odo.AddSpeed(1.599, 2, 0.01);
   odo.AddOdo  (1.600, 50, true);
   odo.AddOdo  (1.650, 250, true);
   odo.AddOdo  (1.700, 250, true);
   odo.AddOdo  (1.750, 250, true);
   odo.AddSpeed(1.799, 10, 0.01);
   odo.AddOdo  (1.800, 250, true);
   EXPECT_EQ   (1.800, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 4.59, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 5.54, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.850, 250, true);
   odo.AddOdo  (1.900, 250, true);
   odo.AddOdo  (1.950, 50, true);
   odo.AddSpeed(1.999, 2, 0.01);
   odo.AddOdo  (2.000, 50, true);
   EXPECT_EQ   (2.000, odo.GetOdoTimeStamp() );
   EXPECT_NEAR (21.03, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR (21.47, odo.ScaleCalibartedTo(), 0.01 );
   EXPECT_FALSE( odo.GetOdoSpeed().IsValid() );
   odo.AddOdo  (2.050, 250, true);
   odo.AddOdo  (2.100, 250, true);
   odo.AddOdo  (2.150, 250, true);
   odo.AddSpeed(2.199, 10, 0.01);
   odo.AddOdo  (2.200, 250, true);
   EXPECT_EQ   (2.200, odo.GetOdoTimeStamp() );
   EXPECT_NEAR (26.52, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR (26.08, odo.ScaleCalibartedTo(), 0.01 );
   //reliable limit is 26%
   EXPECT_TRUE ( odo.GetOdoSpeed().IsValid() );
   EXPECT_NEAR (10.0, odo.GetOdoSpeed().Value, 0.01 );
   EXPECT_NEAR ( 0.0, odo.GetOdoSpeed().Accuracy, PE::EPSILON );
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
