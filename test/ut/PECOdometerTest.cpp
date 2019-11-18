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


   PE::TValue Call_PredictOdoTickSpeed(PE::COdometer& obj, const PE::TTimestamp& referenceSpeedTs, const PE::TTimestamp& odoTsBefore, const PE::TTimestamp& odoTsAfter, const PE::TValue& odoTickSpeedBefore, const PE::TValue& odoTickSpeedAfter )
   {
      return obj.PredictOdoTickSpeed( referenceSpeedTs, odoTsBefore, odoTsAfter, odoTickSpeedBefore, odoTickSpeedAfter );
   }


   bool Call_IsSpeedOk( PE::COdometer& obj, const PE::TTimestamp& deltaTs, const PE::TValue& speed, const PE::TAccuracy& acc)
   {
      return obj.IsSpeedOk( deltaTs, speed, acc);
   }

   bool Call_IsOdoOk( PE::COdometer& obj, const PE::TTimestamp& deltaTs, const PE::TValue& ticks, bool IsValid )
   {
      return obj.IsOdoOk(deltaTs, ticks, IsValid );
   }

   bool Call_IsIntervalOk( PE::COdometer& obj, const PE::TTimestamp& deltaTs, const PE::TValue& interval, const PE::TValue& hysteresis)
   {
      return obj.IsIntervalOk( deltaTs, interval, hysteresis);
   }

   bool Call_IsAccuracyOk( PE::COdometer& obj, const PE::TValue& value, const PE::TAccuracy& accuracy, const PE::TValue& ratio)
   {
      return obj.IsAccuracyOk( value, accuracy, ratio);
   }

   bool Call_IsOdoCalibrated( PE::COdometer& obj, const PE::TValue& biasCalibartedTo, const PE::TValue& scaleCalibartedTo)
   {
      return obj.IsOdoCalibrated( biasCalibartedTo, scaleCalibartedTo);
   }

   bool Call_IsInRange( PE::COdometer& obj, const PE::TTimestamp& testedTS, const PE::TTimestamp& beginTS, const PE::TTimestamp& endTS)
   {
      return obj.IsInRange(testedTS, beginTS, endTS);
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
   EXPECT_FALSE( odo.GetOdoSpeed().IsValid() );
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
   EXPECT_TRUE ( odo.GetOdoSpeed().IsValid() );
   EXPECT_NEAR (10.0, odo.GetOdoSpeed().Value, 0.01 );
   EXPECT_NEAR ( 0.0, odo.GetOdoSpeed().Accuracy, PE::EPSILON );
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
   uint32_t SPEED_CALIBRATION_COUNT_1 = 1;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));

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
   uint32_t SPEED_CALIBRATION_COUNT_1 = 1;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));

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
   uint32_t SPEED_CALIBRATION_COUNT_1 = 1;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));

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
   uint32_t SPEED_CALIBRATION_COUNT_1 = 1;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));

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
   uint32_t SPEED_CALIBRATION_COUNT_1 = 1;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));

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
   uint32_t SPEED_CALIBRATION_COUNT_1 = 1;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));

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
 * checks PredictOdoTickSpeed
 */
TEST_F(PECOdometerTest, test_PredictOdoTickSpeed )
{
   PE::COdometer odo;
   /**
    *  Case: speed in between of two odometers
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1005s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1010s |    20 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR ( 15, Call_PredictOdoTickSpeed(odo, 1005, 1000, 1010, 10,20), PE::EPSILON );

   /**
    *  Case: speed same as first odometer timestamp
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1000s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1010s |    20 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR ( 10, Call_PredictOdoTickSpeed(odo, 1000, 1000, 1010, 10,20), PE::EPSILON );

   /**
    *  Case: speed same as last odometer timestamp
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1010s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1010s |    20 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR ( 20, Call_PredictOdoTickSpeed(odo, 1010, 1000, 1010, 10,20), PE::EPSILON );

   /**
    *  Case: speed close to the first odometer timestamp
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1001s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1010s |    20 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR ( 11, Call_PredictOdoTickSpeed(odo, 1001, 1000, 1010, 10,20), PE::EPSILON );

   /**
    *  Case: speed close to the last odometer timestamp
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1009s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1010s |    20 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR ( 19, Call_PredictOdoTickSpeed(odo, 1009, 1000, 1010, 10,20), PE::EPSILON );

   /**
    *  Case: odometer values are equal
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1001s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1010s |    10 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR ( 10, Call_PredictOdoTickSpeed(odo, 1001, 1000, 1010, 10,10), PE::EPSILON );

   /**
    *  Case: odometer timestam and values are equal -- wrong case - has to be avoided!!! - NaN value
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1000s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1000s |    10 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_TRUE ( PE::isnan(Call_PredictOdoTickSpeed(odo, 1000, 1000, 1000, 10, 10)) );

   /**
    *  Case: odometer timestam are equal but value are different -- wrong case - has to be avoided!!! - NaN value
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1000s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1000s |    20 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_TRUE ( PE::isnan(Call_PredictOdoTickSpeed(odo, 1000, 1000, 1000, 10, 20)) );

   /**
    *  Case: speed timestamp is before the first odometer speed -- wrong case - has to be avoided!!!
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |      999s |  ???? | Predicted odometer for timestamp of reference speed |
    *  |     1000s |    10 | First odometer                                      |
    *  |     1010s |    20 | Last odometer                                       |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR (  9, Call_PredictOdoTickSpeed(odo, 999, 1000, 1010, 10,20), PE::EPSILON );

   /**
    *  Case: speed timestamp is after the last odometer speed -- wrong case - has to be avoided!!!
    *  +-----------+-------+-----------------------------------------------------+
    *  | timestamp |  odo  |  comment                                            |
    *  +-----------+-------+-----------------------------------------------------+
    *  |     1000s |    10 | First odometer                                      |
    *  |     1010s |    20 | Last odometer                                       |
    *  |     1011s |  ???? | Predicted odometer for timestamp of reference speed |
    *  +-----------+-------+-----------------------------------------------------+
    */
   EXPECT_NEAR ( 21, Call_PredictOdoTickSpeed(odo, 1011, 1000, 1010, 10,20), PE::EPSILON );
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
   uint32_t SPEED_CALIBRATION_COUNT_1 = 1;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));

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
   uint32_t SPEED_CALIBRATION_COUNT_1 = 1;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_26PROCENT, SCALE_LIMIT_26PROCENT, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));

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
 * checks IsIntervalOk
 */
TEST_F(PECOdometerTest, test_IsIntervalOk )
{
   PE::TValue       INTERVAL_1000MS = 1.000;
   PE::TValue       HYSTERESIS_10MS = 0.010;

   PE::COdometer odo;

   //interval 1 second +/-10ms
   //all is valid
   EXPECT_TRUE (Call_IsIntervalOk(odo,       1.000, INTERVAL_1000MS, HYSTERESIS_10MS));
   //timeinterval almost zero
   EXPECT_FALSE(Call_IsIntervalOk(odo, PE::EPSILON, INTERVAL_1000MS, HYSTERESIS_10MS));
   //timeinterval is out +10ms
   EXPECT_FALSE(Call_IsIntervalOk(odo,       1.010001, INTERVAL_1000MS, HYSTERESIS_10MS));
   //timeinterval is out -10ms
   EXPECT_FALSE(Call_IsIntervalOk(odo,       0.989999, INTERVAL_1000MS, HYSTERESIS_10MS));
}


/**
 * checks IsAccuracyOk
 */
TEST_F(PECOdometerTest, test_IsAccuracyOk )
{
   uint32_t   ACCURACY_RATIO_x2 = 2;
   uint32_t  ACCURACY_RATIO_x10 = 10;
   uint32_t   ACCURACY_RATIO_x0 = 0;

   PE::COdometer odo;

   //all is valid
   EXPECT_TRUE (Call_IsAccuracyOk(odo,  10.000, 0.1, ACCURACY_RATIO_x2));
   EXPECT_TRUE (Call_IsAccuracyOk(odo,  10.000, 0.1, ACCURACY_RATIO_x10));
   EXPECT_TRUE (Call_IsAccuracyOk(odo,  10.000, 0.1, ACCURACY_RATIO_x0));
   //value is out of accuracy ratio
   EXPECT_FALSE(Call_IsAccuracyOk(odo,  10.000, 1.0, ACCURACY_RATIO_x10));
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
   uint32_t SPEED_CALIBRATION_COUNT_1 = 1;

   PE::COdometer odo;

   //call init all is correct
   EXPECT_TRUE(odo.Init(ODO_INTERVAL_50MS, SPEED_INTERVAL_200MS, BIAS_LIMIT_22PROCENT, SCALE_LIMIT_33PROCENT, SPEED_ACCURACY_RATIO_x2, SPEED_CALIBRATION_COUNT_1));

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
 * checks IsInRange
 */
TEST_F(PECOdometerTest, test_IsInRange )
{
   PE::COdometer odo;

   //data is in range from left
   EXPECT_TRUE (Call_IsInRange(odo, 10, 10, 100));
   //data is in range from right
   EXPECT_TRUE (Call_IsInRange(odo, 100, 10, 100));
   //data positive is in range in between
   EXPECT_TRUE (Call_IsInRange(odo,  50, 10, 100));
   //data negative is in range in between
   EXPECT_TRUE (Call_IsInRange(odo, -5, -10, 100));
   //data is out of range from the left
   EXPECT_FALSE(Call_IsInRange(odo,   1, 10, 100));
   //data is out of range from the right
   EXPECT_FALSE(Call_IsInRange(odo, 101, 10, 100));
   //wrong range definition
   EXPECT_FALSE(Call_IsInRange(odo,  50, 100, 10));
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


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
