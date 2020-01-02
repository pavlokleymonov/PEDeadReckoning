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

#include <fstream>
#include <iostream>
#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "PECOdometer.h"
#include "PETools.h"

class PECOdometerTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }


   bool Call_IsSpeedOk( PE::COdometer& obj, const PE::TTimestamp& deltaTs, const PE::TValue& speed, const PE::TAccuracy& acc)
   {
      return obj.IsSpeedOk( deltaTs, speed, acc);
   }

   bool Call_IsOdoOk( PE::COdometer& obj, const PE::TTimestamp& deltaTs, const PE::TValue& ticks, bool IsValid )
   {
      return obj.IsOdoOk(deltaTs, ticks, IsValid );
   }

   bool Call_IsOdoCalibrated( PE::COdometer& obj, const PE::TValue& biasCalibartedTo, const PE::TValue& scaleCalibartedTo)
   {
      return obj.IsOdoCalibrated( biasCalibartedTo, scaleCalibartedTo);
   }

   bool Call_IsCalibrationPossible( PE::COdometer& obj, const PE::TTimestamp& speedTs, const PE::TValue& speed, const PE::TTimestamp& OdoTsBefore, const PE::TValue& OdoTickSpeedBefore, const PE::TTimestamp& OdoTsAfter, const PE::TValue& OdoTickSpeedAfter )
   {
      return obj.IsCalibrationPossible( speedTs, speed, OdoTsBefore, OdoTickSpeedBefore, OdoTsAfter, OdoTickSpeedAfter );
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
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_99PROCENT, SCALE_LIMIT_99PROCENT, SPEED_ACCURACY_RATIO_x2));
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

   PE::COdometer odo;

   //call init wrong odo interval
   EXPECT_FALSE(odo.Init(PE::EPSILON, SPEED_INTERVAL_200MS, BIAS_LIMIT_99PROCENT, SCALE_LIMIT_99PROCENT, SPEED_ACCURACY_RATIO_x2));

   //call init wrong speed interval
   EXPECT_FALSE(odo.Init(ODO_INTERVAL_50MS, ODO_INTERVAL_50MS, BIAS_LIMIT_99PROCENT, SCALE_LIMIT_99PROCENT, SPEED_ACCURACY_RATIO_x2));

   //call init wrong bias limit
   EXPECT_FALSE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, PE::EPSILON, SCALE_LIMIT_99PROCENT, SPEED_ACCURACY_RATIO_x2));

   //call init wrong scale limit
   EXPECT_FALSE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_99PROCENT, PE::EPSILON, SPEED_ACCURACY_RATIO_x2));

   //call init wrong accuracy ratio
   EXPECT_FALSE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_99PROCENT, SCALE_LIMIT_99PROCENT, 0));

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_99PROCENT, SCALE_LIMIT_99PROCENT, SPEED_ACCURACY_RATIO_x2));
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

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2));

   odo.AddSpeed(0.999, 10, 0.01);
   odo.AddOdo  (1.000, 250, true);
   EXPECT_EQ   (1.000, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.050, 250, true);
   odo.AddOdo  (1.100, 250, true);
   odo.AddOdo  (1.150, 250, true);
   odo.AddSpeed(1.199, 10, 0.01);
   odo.AddOdo  (1.200, 250, true);
   EXPECT_EQ   (1.200, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.250, 500, true);
   odo.AddOdo  (1.300, 500, true);
   odo.AddOdo  (1.350, 500, true);
   odo.AddSpeed(1.399, 20, 0.01);
   odo.AddOdo  (1.400, 500, true);
   EXPECT_EQ   (1.400, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.450, 50, true);
   odo.AddOdo  (1.500, 50, true);
   odo.AddOdo  (1.550, 50, true);
   odo.AddSpeed(1.599, 2, 0.01);
   odo.AddOdo  (1.600, 50, true);
   EXPECT_EQ   (1.600, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 3.57, odo.ScaleCalibartedTo(), 0.01 );
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
   EXPECT_FALSE( odo.IsOdoSpeedCalibrated() );
   odo.AddOdo  (2.050, 250, true);
   odo.AddOdo  (2.100, 250, true);
   odo.AddOdo  (2.150, 250, true);
   odo.AddSpeed(2.199, 10, 0.01);
   odo.AddOdo  (2.200, 250, true);
   EXPECT_EQ   (2.200, odo.GetOdoTimeStamp() );
   EXPECT_NEAR (26.52, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR (00.000, odo.GetOdoBias(), 0.001 );
   EXPECT_NEAR (26.08, odo.ScaleCalibartedTo(), 0.01 );
   EXPECT_NEAR (00.002, odo.GetOdoScale(), 0.001 );
   //reliable limit is 26%
   EXPECT_TRUE ( odo.IsOdoSpeedCalibrated() );
   EXPECT_NEAR (10.0, odo.GetOdoSpeedValue(), 0.01 );
   EXPECT_NEAR ( 0.0, odo.GetOdoSpeedAccuracy(), PE::EPSILON );
}


/**
 * checks missed one odometer value
 */
TEST_F(PECOdometerTest, test_missed_one_odometer_value)
{
   PE::TValue       ODO_INTERVAL_50MS = 0.050;
   PE::TValue    SPEED_INTERVAL_200MS = 0.200;
   PE::TValue    BIAS_LIMIT_26PROCENT = 26.00;
   PE::TValue   SCALE_LIMIT_26PROCENT = 26.00;
   uint32_t   SPEED_ACCURACY_RATIO_x2 = 2;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2));

   odo.AddSpeed(0.999, 10, 0.01);
   odo.AddOdo  (1.000, 250, true);
   EXPECT_EQ   (1.000, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.050, 250, true);
   odo.AddOdo  (1.100, 250, true);
   odo.AddOdo  (1.150, 250, true);
   odo.AddSpeed(1.199, 10, 0.01);
   odo.AddOdo  (1.200, 250, true);
   EXPECT_EQ   (1.200, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.250, 500, true);
   odo.AddOdo  (1.300, 500, true);
   odo.AddOdo  (1.350, 500, true);
   odo.AddSpeed(1.399, 20, 0.01);
   odo.AddOdo  (1.400, 500, true);
   EXPECT_EQ   (1.400, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.450, 50, true);
//   odo.AddOdo  (1.500, 50, true); //simulate an odo error
   odo.AddOdo  (1.550, 50, true);
   odo.AddSpeed(1.599, 2, 0.01);
   odo.AddOdo  (1.600, 50, true);
   EXPECT_EQ   (1.600, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   //EXPECT_NEAR ( 3.57, odo.ScaleCalibartedTo(), 0.01 ); //3.57% is a result if all data are correct
}


/**
 * checks invalid odometer value
 */
TEST_F(PECOdometerTest, test_invalid_odometer_value)
{
   PE::TValue       ODO_INTERVAL_50MS = 0.050;
   PE::TValue    SPEED_INTERVAL_200MS = 0.200;
   PE::TValue    BIAS_LIMIT_26PROCENT = 26.00;
   PE::TValue   SCALE_LIMIT_26PROCENT = 26.00;
   uint32_t   SPEED_ACCURACY_RATIO_x2 = 2;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2));

   odo.AddSpeed(0.999, 10, 0.01);
   odo.AddOdo  (1.000, 250, true);
   EXPECT_EQ   (1.000, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.050, 250, true);
   odo.AddOdo  (1.100, 250, true);
   odo.AddOdo  (1.150, 250, true);
   odo.AddSpeed(1.199, 10, 0.01);
   odo.AddOdo  (1.200, 250, true);
   EXPECT_EQ   (1.200, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.250, 500, true);
   odo.AddOdo  (1.300, 500, true);
   odo.AddOdo  (1.350, 500, true);
   odo.AddSpeed(1.399, 20, 0.01);
   odo.AddOdo  (1.400, 500, true);
   EXPECT_EQ   (1.400, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.450, 50, false); //provides invalid odo value
   odo.AddOdo  (1.500, 50, true);
   odo.AddOdo  (1.550, 50, true);
   odo.AddSpeed(1.599, 2, 0.01);
   odo.AddOdo  (1.600, 50, true);
   EXPECT_EQ   (1.600, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   //EXPECT_NEAR ( 3.57, odo.ScaleCalibartedTo(), 0.01 ); //3.57% is a result if all data are correct
}


/**
 * checks zero odometer value
 */
TEST_F(PECOdometerTest, test_zero_odometer_value)
{
   PE::TValue       ODO_INTERVAL_50MS = 0.050;
   PE::TValue    SPEED_INTERVAL_200MS = 0.200;
   PE::TValue    BIAS_LIMIT_26PROCENT = 26.00;
   PE::TValue   SCALE_LIMIT_26PROCENT = 26.00;
   uint32_t   SPEED_ACCURACY_RATIO_x2 = 2;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2));

   odo.AddSpeed(0.999, 10, 0.01);
   odo.AddOdo  (1.000, 250, true);
   EXPECT_EQ   (1.000, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.050, 250, true);
   odo.AddOdo  (1.100, 250, true);
   odo.AddOdo  (1.150, 250, true);
   odo.AddSpeed(1.199, 10, 0.01);
   odo.AddOdo  (1.200, 250, true);
   EXPECT_EQ   (1.200, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.250, 500, true);
   odo.AddOdo  (1.300, 500, true);
   odo.AddOdo  (1.350, 500, true);
   odo.AddSpeed(1.399, 20, 0.01);
   odo.AddOdo  (1.400, 500, true);
   EXPECT_EQ   (1.400, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.450, 50, true);
   odo.AddOdo  (1.500, 50, true);
   odo.AddOdo  (1.550, PE::EPSILON, true); //provides zero odometer value
   odo.AddSpeed(1.599, 2, 0.01);
   odo.AddOdo  (1.600, 50, true);
   EXPECT_EQ   (1.600, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   //EXPECT_NEAR ( 3.57, odo.ScaleCalibartedTo(), 0.01 ); //3.57% is a result if all data are correct
}


/**
 * checks same timestamp of  odometer value
 */
TEST_F(PECOdometerTest, test_same_timesatmp_of_odometer_value)
{
   PE::TValue       ODO_INTERVAL_50MS = 0.050;
   PE::TValue    SPEED_INTERVAL_200MS = 0.200;
   PE::TValue    BIAS_LIMIT_26PROCENT = 26.00;
   PE::TValue   SCALE_LIMIT_26PROCENT = 26.00;
   uint32_t   SPEED_ACCURACY_RATIO_x2 = 2;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2));

   odo.AddSpeed(0.999, 10, 0.01);
   odo.AddOdo  (1.000, 250, true);
   EXPECT_EQ   (1.000, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.050, 250, true);
   odo.AddOdo  (1.100, 250, true);
   odo.AddOdo  (1.150, 250, true);
   odo.AddSpeed(1.199, 10, 0.01);
   odo.AddOdo  (1.200, 250, true);
   EXPECT_EQ   (1.200, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.250, 500, true);
   odo.AddOdo  (1.300, 500, true);
   odo.AddOdo  (1.350, 500, true);
   odo.AddSpeed(1.399, 20, 0.01);
   odo.AddOdo  (1.400, 500, true);
   EXPECT_EQ   (1.400, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.450, 50, true);
   odo.AddOdo  (1.500, 50, true);
   //odo.AddOdo  (1.550, 50, true);
   odo.AddOdo  (1.500, 50, true); //same timestamp
   odo.AddSpeed(1.599, 2, 0.01);
   odo.AddOdo  (1.600, 50, true);
   EXPECT_EQ   (1.600, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   //EXPECT_NEAR ( 3.57, odo.ScaleCalibartedTo(), 0.01 ); //3.57% is a result if all data are correct
}


/**
 * checks missed one speed value
 */
TEST_F(PECOdometerTest, test_missed_one_speed_value)
{
   PE::TValue       ODO_INTERVAL_50MS = 0.050;
   PE::TValue    SPEED_INTERVAL_200MS = 0.200;
   PE::TValue    BIAS_LIMIT_26PROCENT = 26.00;
   PE::TValue   SCALE_LIMIT_26PROCENT = 26.00;
   uint32_t   SPEED_ACCURACY_RATIO_x2 = 2;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2));

   odo.AddSpeed(0.999, 10, 0.01);
   odo.AddOdo  (1.000, 250, true);
   EXPECT_EQ   (1.000, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.050, 250, true);
   odo.AddOdo  (1.100, 250, true);
   odo.AddOdo  (1.150, 250, true);
   odo.AddSpeed(1.199, 10, 0.01);
   odo.AddOdo  (1.200, 250, true);
   EXPECT_EQ   (1.200, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.250, 500, true);
   odo.AddOdo  (1.300, 500, true);
   odo.AddOdo  (1.350, 500, true);
//    odo.AddSpeed(1.399, 20, 0.01); simulate missed speed
   odo.AddOdo  (1.400, 500, true);
   EXPECT_EQ   (1.400, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.450, 50, true);
   odo.AddOdo  (1.500, 50, true);
   odo.AddOdo  (1.550, 50, true);
   odo.AddSpeed(1.599, 2, 0.01);
   odo.AddOdo  (1.600, 50, true);
   //EXPECT_EQ   (1.600, odo.GetOdoTimeStamp() ); //odometer value is ignored if there is no valid speed
   EXPECT_EQ   ( 0.00, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   //EXPECT_NEAR ( 3.57, odo.ScaleCalibartedTo(), 0.01 ); //3.57% is a result if all data are correct
}


/**
 * checks zero speed value
 */
TEST_F(PECOdometerTest, test_zero_speed_value)
{
   PE::TValue       ODO_INTERVAL_50MS = 0.050;
   PE::TValue    SPEED_INTERVAL_200MS = 0.200;
   PE::TValue    BIAS_LIMIT_26PROCENT = 26.00;
   PE::TValue   SCALE_LIMIT_26PROCENT = 26.00;
   uint32_t   SPEED_ACCURACY_RATIO_x2 = 2;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2));

   odo.AddSpeed(0.999, 10, 0.01);
   odo.AddOdo  (1.000, 250, true);
   EXPECT_EQ   (1.000, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.050, 250, true);
   odo.AddOdo  (1.100, 250, true);
   odo.AddOdo  (1.150, 250, true);
   odo.AddSpeed(1.199, 10, 0.01);
   odo.AddOdo  (1.200, 250, true);
   EXPECT_EQ   (1.200, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.250, 500, true);
   odo.AddOdo  (1.300, 500, true);
   odo.AddOdo  (1.350, 500, true);
   odo.AddSpeed(1.399, PE::EPSILON, 0.01); //simulate almost zero speed value
   odo.AddOdo  (1.400, 500, true);
   //EXPECT_EQ   (1.400, odo.GetOdoTimeStamp() ); //because of wrong speed odometer value are reseted
   EXPECT_EQ   ( 0.00, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   odo.AddOdo  (1.450, 50, true);
   odo.AddOdo  (1.500, 50, true);
   odo.AddOdo  (1.550, 50, true);
   odo.AddSpeed(1.599, 2, 0.01);
   odo.AddOdo  (1.600, 50, true);
   EXPECT_EQ   (1.600, odo.GetOdoTimeStamp() );
   EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
   EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
   //EXPECT_NEAR ( 3.57, odo.ScaleCalibartedTo(), 0.01 ); //3.57% is a result if all data are correct
}


/**
 * checks IsSpeedOk
 */
TEST_F(PECOdometerTest, test_IsSpeedOk )
{
   PE::TValue       ODO_INTERVAL_50MS = 0.050;
   PE::TValue    SPEED_INTERVAL_200MS = 0.200;
   PE::TValue    BIAS_LIMIT_26PROCENT = 26.00;
   PE::TValue   SCALE_LIMIT_26PROCENT = 26.00;
   uint32_t   SPEED_ACCURACY_RATIO_x2 = 2;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2));

   //all is valid
   EXPECT_TRUE(Call_IsSpeedOk(odo, 0.200, 10.00, 0.01));
   //timeinterval almost zero
   EXPECT_FALSE(Call_IsSpeedOk(odo, PE::EPSILON, 10.00, 0.01));
   //speed is negative
   EXPECT_FALSE(Call_IsSpeedOk(odo, 0.200, -1, 0.01));
   //speed is zero
   EXPECT_FALSE(Call_IsSpeedOk(odo, 0.200, PE::EPSILON, PE::EPSILON / (SPEED_ACCURACY_RATIO_x2 + 1)));
   //speed is out interval -10%
   EXPECT_FALSE(Call_IsSpeedOk(odo, 0.180, 10.00, 0.01));
   //speed is in interval -10%
   EXPECT_TRUE(Call_IsSpeedOk(odo, 0.181, 10.00, 0.01));
   //speed is out interval +10%
   EXPECT_FALSE(Call_IsSpeedOk(odo, 0.220, 10.00, 0.01));
   //speed is in interval +10%
   EXPECT_TRUE(Call_IsSpeedOk(odo, 0.219, 10.00, 0.01));
   //speed does not feet to accuracy ratio
   EXPECT_FALSE(Call_IsSpeedOk(odo, 0.200, 1.00, 0.50));
   //speed feets to accuracy ratio
   EXPECT_TRUE(Call_IsSpeedOk(odo, 0.200, 1.00, 0.49));
}


/**
 * checks IsOdoOk
 */
TEST_F(PECOdometerTest, test_IsOdoOk )
{
   PE::TValue       ODO_INTERVAL_50MS = 0.050;
   PE::TValue    SPEED_INTERVAL_200MS = 0.200;
   PE::TValue    BIAS_LIMIT_26PROCENT = 26.00;
   PE::TValue   SCALE_LIMIT_26PROCENT = 26.00;
   uint32_t   SPEED_ACCURACY_RATIO_x2 = 2;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2));

   //all is valid
   EXPECT_TRUE(Call_IsOdoOk (odo, 0.050, 10.00, true));
   //tick is invalid
   EXPECT_FALSE(Call_IsOdoOk(odo, 0.050, 10.00, false));
   //timeinterval almost zero
   EXPECT_FALSE(Call_IsOdoOk(odo, PE::EPSILON, 10.00, true));
   //tick is negative
   EXPECT_FALSE(Call_IsOdoOk(odo, 0.050, -10.00, true));
   //tick is zero
   EXPECT_FALSE(Call_IsOdoOk(odo, 0.050,   0.00, true));
   //tick is out interval -5%
   EXPECT_FALSE(Call_IsOdoOk(odo, 0.0474999, 10.00, true));
   //tick is in interval -5%
   EXPECT_TRUE(Call_IsOdoOk (odo, 0.0475001, 10.00, true));
   //tick is out interval +5%
   EXPECT_FALSE(Call_IsOdoOk(odo, 0.0525001, 10.00, true));
   //tick is in interval +5%
   EXPECT_TRUE(Call_IsOdoOk (odo, 0.0524999, 10.00, true));
}


/**
 * checks IsOdoCalibrated
 */
TEST_F(PECOdometerTest, test_IsOdoCalibrated )
{
   PE::TValue       ODO_INTERVAL_50MS = 0.050;
   PE::TValue    SPEED_INTERVAL_200MS = 0.200;
   PE::TValue    BIAS_LIMIT_22PROCENT = 22.00;
   PE::TValue   SCALE_LIMIT_33PROCENT = 33.00;
   uint32_t   SPEED_ACCURACY_RATIO_x2 = 2;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_22PROCENT, SCALE_LIMIT_33PROCENT, SPEED_ACCURACY_RATIO_x2));

   //all is calibrated
   EXPECT_TRUE (Call_IsOdoCalibrated(odo, BIAS_LIMIT_22PROCENT + 0.01, SCALE_LIMIT_33PROCENT + 0.01));
   //bias is not callibrated
   EXPECT_FALSE(Call_IsOdoCalibrated(odo,        BIAS_LIMIT_22PROCENT, SCALE_LIMIT_33PROCENT + 0.01));
   //scale is not callibrated
   EXPECT_FALSE(Call_IsOdoCalibrated(odo, BIAS_LIMIT_22PROCENT + 0.01,        SCALE_LIMIT_33PROCENT));
   //bias and scale are not callibrated
   EXPECT_FALSE(Call_IsOdoCalibrated(odo,        BIAS_LIMIT_22PROCENT,        SCALE_LIMIT_33PROCENT));
}


/**
 * checks IsCalibrationPossible
 */
TEST_F(PECOdometerTest, test_IsCalibrationPossible )
{
   PE::COdometer odo;

   PE::TTimestamp SPEED_TS_1001S = 1001;
   PE::TValue   SPEED_10_M_PER_S = 10;
   PE::TTimestamp   ODO_TS_1000S = 1000;
   PE::TValue      ODO_100_TICKS = 100;
   PE::TTimestamp   ODO_TS_1002S = 1002;
   PE::TValue      ODO_200_TICKS = 200;

   //all values are correct
   EXPECT_TRUE (Call_IsCalibrationPossible(odo, SPEED_TS_1001S, SPEED_10_M_PER_S, ODO_TS_1000S, ODO_100_TICKS, ODO_TS_1002S, ODO_200_TICKS));
   //speed value is zero
   EXPECT_FALSE(Call_IsCalibrationPossible(odo, SPEED_TS_1001S,                0, ODO_TS_1000S, ODO_100_TICKS, ODO_TS_1002S, ODO_200_TICKS));
   //odometer from the left is zero
   EXPECT_FALSE(Call_IsCalibrationPossible(odo, SPEED_TS_1001S, SPEED_10_M_PER_S, ODO_TS_1000S,             0, ODO_TS_1002S, ODO_200_TICKS));
   //odometer from the right is zero
   EXPECT_FALSE(Call_IsCalibrationPossible(odo, SPEED_TS_1001S, SPEED_10_M_PER_S, ODO_TS_1000S, ODO_100_TICKS, ODO_TS_1002S,             0));
   //speed is out of the range
   EXPECT_FALSE(Call_IsCalibrationPossible(odo,              0, SPEED_10_M_PER_S, ODO_TS_1000S, ODO_100_TICKS, ODO_TS_1002S, ODO_200_TICKS));
}

#ifdef TTT

#define PE_ODO_RAW_VALUE_MAX      2047   ///< Maximum value of odometer
uint32_t GetDeltaTick(uint32_t odo_raw_value)
{
   static uint32_t last_odo_raw_value = 0;
   uint32_t ticks = odo_raw_value - last_odo_raw_value;
   if ( last_odo_raw_value > odo_raw_value )
   {
      ticks = PE_ODO_RAW_VALUE_MAX + 1 + odo_raw_value - last_odo_raw_value;
   }
   last_odo_raw_value = odo_raw_value;
   return ticks;
}


/**
 * checks 60seconds drive track file
 */
TEST_F(PECOdometerTest, test_ODO_40ms_60sec_GNSS_100ms_32sec)
{
   PE::TValue       ODO_INTERVAL_40MS = 0.040;
   PE::TValue   SPEED_INTERVAL_1000MS = 1.000;
   PE::TValue      BIAS_LIMIT_PROCENT = 26.00;
   PE::TValue     SCALE_LIMIT_PROCENT = 26.00;
   uint32_t   SPEED_ACCURACY_RATIO_x2 = 2;

   std::string TRACK_ODO_40MS_60SEC_GNSS_100MS_32SEC = "ODO_40ms_60sec_GNSS_100ms_32sec.txt";
   std::ifstream* trk = new std::ifstream(TRACK_ODO_40MS_60SEC_GNSS_100MS_32SEC);
   EXPECT_TRUE(trk->is_open());

   std::string line;
   PE::COdometer odo;
   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_40MS, SPEED_INTERVAL_1000MS, BIAS_LIMIT_PROCENT, SCALE_LIMIT_PROCENT, SPEED_ACCURACY_RATIO_x2));

   uint32_t          ts_ms = 0;
   uint32_t     speed_orig = 0;
   uint32_t acc_speed_orig = 0;
   uint32_t       odo_orig = 0;
   bool       is_new_speed = false;
   uint32_t  tries = 0;
   do
   {
      *trk >> line;
      std::vector<std::string> group = PE::TOOLS::Split(line, ',');
      if (3 < group.size())
      {
         //printf("0-%s 1-%s 3-%s\n", group[0].c_str(), group[1].c_str(), group[3].c_str());
         ts_ms = atoi(group[0].c_str());
         PE::TTimestamp ts = ts_ms / 1000.0;
         if ( 0 == group[1].compare("ODO") )
         {
            odo_orig = atoi(group[3].c_str());
            uint32_t ticks = GetDeltaTick(odo_orig);
            odo.AddOdo(ts, ticks, true);
            printf("ts=%0.3f ticks=%d odo=%d\n", ts, ticks, odo_orig);
         }
         if ( 0 == group[1].compare("SPEED") )
         {
            if ( atoi(group[3].c_str()) != speed_orig )
            {
               speed_orig = atoi(group[3].c_str());
               is_new_speed = true;
            }
            else
            {
               is_new_speed = false;
            }
         }
         if ( 0 == group[1].compare("ACC") )
         {
            if ( true == is_new_speed )
            {
               acc_speed_orig = atoi(group[3].c_str());
               odo.AddSpeed(ts, speed_orig / 100.0, acc_speed_orig / 100.0);
               printf("ts=%0.3f speed=%0.2f[m/s] acc=%0.2f[m/s]\n", ts, speed_orig / 100.0, acc_speed_orig / 100.0);
               is_new_speed = false;
            }
         }

         //CASE: 10x speeds were provided
//          if ( 270128 == ts_ms)
//          {
//             EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
//             EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
//          }
         //CASE: last speed were provided (round about 40 seconds one iteration)
         if ( 301608 == ts_ms)
         {
            EXPECT_NEAR ( 0.00, odo.BiasCalibartedTo(), 0.01 );
            EXPECT_NEAR ( 0.00, odo.GetOdoBias(), 0.01 );
            EXPECT_NEAR ( 0.00, odo.ScaleCalibartedTo(), 0.01 );
            EXPECT_NEAR ( 0.00, odo.GetOdoScale(), 0.01 );
            tries++;
            if ( tries > 100 ) //100 iteration 1 hour 6 minutes
            {
               break;
            }
            else
            {
               ts_ms = 0;
               speed_orig = 0;
               acc_speed_orig = 0;
               odo_orig = 0;
               is_new_speed = false;
               //reopen track
               trk->close();
               trk->open(TRACK_ODO_40MS_60SEC_GNSS_100MS_32SEC);
            }
         }
      }
   }
   while (trk->good());

   delete trk;
}
#endif

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
