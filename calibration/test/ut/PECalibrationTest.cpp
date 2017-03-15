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
 * Unit test of the PECalibrationTest class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PECalibration.h"

class PECalibrationTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }
};

TEST_F(PECalibrationTest, calc_callibration_no_init_calibartion_limit_10m_test)
{
   PE::calibration calib(0,10.0);

   calib.add_sensor_steps(0,1.0); //no changing
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.1);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.1);

   calib.add_sensor_steps(0,1.0); //no changing
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.1);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.1);

   calib.add_reference_value(0,11.0); //no changing accuracy above limit
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.1);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.1);
  
   calib.add_sensor_steps(1,1.0); //1
   calib.add_sensor_steps(2,1.0); //1+2
   calib.add_sensor_steps(2,1.0); //1+2+2
   calib.add_sensor_steps(3,1.0); //1+2+2+3
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.1);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.1);

   calib.add_reference_value(8,11.0); //no changing accuracy above limit
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.1);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.1);

   calib.add_sensor_steps(3,1.0); //3
   calib.add_sensor_steps(3,1.0); //3+3
   calib.add_sensor_steps(4,1.0); //3+3+4=10 step=10
   calib.add_reference_value(20.0, 5.0); //accuracy=5 is ok value=20
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.1);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.1);

   calib.add_sensor_steps(10,1.0); //20
   calib.add_sensor_steps(10,1.0); //30
   calib.add_sensor_steps(15,1.0); //45
   calib.add_reference_value(70.0, 1.0); //accuracy=1 is ok value=90
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.1);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.1);

   calib.add_sensor_steps(15,1.0); //60
   calib.add_sensor_steps(25,1.0); //85
   calib.add_sensor_steps(30,1.0); //115
   calib.add_reference_value(140.0, 2.0); //accuracy=2 is ok value=230
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.1);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.1);

   calib.add_sensor_steps(20,1.0); //135
   calib.add_sensor_steps(25,1.0); //160
   calib.add_reference_value(90.0, 4.0); //accuracy=4 is ok value=320
   EXPECT_NEAR(3.0, calib.get_sensor_accuracy(), 0.1);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.1);

   calib.add_sensor_steps(40,1.0); //200
   calib.add_sensor_steps(50,1.0); //250
   calib.add_reference_value(180.0, 3.0); //accuracy=4 is ok value=500
   EXPECT_NEAR(3.0, calib.get_sensor_accuracy(), 0.1);
   EXPECT_NEAR(100.0, calib.get_sensor_calibration(), 0.1);
   EXPECT_NEAR(2.0, calib.get_values_per_step(), 0.001);
}

//Test calc calibration status
TEST_F(PECalibrationTest, calc_callibration_status_test)
{
   //both values are null
   EXPECT_EQ(0.0, PE::calc_callibration(0,0));

   //first value is null
   EXPECT_EQ(0.0, PE::calc_callibration(0,1000));

   //second value is null
   EXPECT_EQ(0.0, PE::calc_callibration(10,0));

   //fisrt > second*10 = 10%
   EXPECT_EQ(10.0, PE::calc_callibration(10,1));

   //100 < 1000 = 10%
   EXPECT_EQ(10.0, PE::calc_callibration(100,1000));

   //1000 > 100 = 10%
   EXPECT_EQ(10.0, PE::calc_callibration(1000,100));

   //1 > 10000 = 0.01%
   EXPECT_NEAR(0.01, PE::calc_callibration(1,10000), 0.01);

   //2430 > 2428 = 99.92%
   EXPECT_NEAR(99.92, PE::calc_callibration(2430,2428),0.01);

   //1500 < 2500 = 60.0%
   EXPECT_NEAR(60.0, PE::calc_callibration(1500,2500),0.01);

   //2500 > 1500 = 60.0%
   EXPECT_NEAR(60.0, PE::calc_callibration(2500, 1500),0.01);
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

