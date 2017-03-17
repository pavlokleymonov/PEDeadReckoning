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

/**
* tests wrong negative reference accuracy
*/
TEST_F(PECalibrationTest, negative_reference_accuracy_test)
{
   PE::calibration calib(0,10.0);

   calib.add_sensor_steps(100,1.0);
   calib.add_reference_value(500,-1.0);
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);

   calib.add_sensor_steps(100,1.0);
   calib.add_reference_value(500,-1.0);
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);

   calib.add_sensor_steps(100,1.0);
   calib.add_reference_value(500,-1.0);
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);

   calib.add_sensor_steps(100,1.0);
   calib.add_reference_value(500,-1.0);
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);
}



/**
* tests wrong negative sensor accuracy
*/
TEST_F(PECalibrationTest, negative_sensor_accuracy_test)
{
   PE::calibration calib(0,10.0);

   calib.add_sensor_steps(10,-1.0);
   calib.add_reference_value(500,10.0);
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);

   calib.add_sensor_steps(10,-1.0);
   calib.add_reference_value(500,10.0);
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);

   calib.add_sensor_steps(10,-1.0);
   calib.add_reference_value(500,10.0);
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);
}

/**
* tests wrong zero sensor accuracy
*/
TEST_F(PECalibrationTest, zero_sensor_accuracy_test)
{
   PE::calibration calib(0,10.0);

   calib.add_sensor_steps(10,0.0);
   calib.add_reference_value(500,10.0);
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);

   calib.add_sensor_steps(10,0.0);
   calib.add_reference_value(500,10.0);
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);

   calib.add_sensor_steps(10,0.0);
   calib.add_reference_value(500,10.0);
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);
}


/**
* tests slow sensors data
*/
TEST_F(PECalibrationTest, slow_sensor_data_test)
{
   PE::calibration calib(0,10.0);

   calib.add_reference_value(500,10.0);
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);

   calib.add_reference_value(600,10.0);
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);

   calib.add_sensor_steps(120,1.0);
   calib.add_reference_value(100,10.0);
   EXPECT_NEAR(10.0, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);

   calib.add_sensor_steps(10,1.0);
   calib.add_reference_value(100,10.0);
   EXPECT_NEAR(10.0, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(100.0, calib.get_sensor_calibration(), 0.01);
   EXPECT_NEAR(10.0, calib.get_values_per_step(), 0.01);
}


/**
* tests ignoring incoming sensors data without valid reference data
*/
TEST_F(PECalibrationTest, no_reference_data_test)
{
   PE::calibration calib(0,10.0);
   calib.add_sensor_steps(10,1.0);
   calib.add_sensor_steps(10,1.0);
   calib.add_sensor_steps(10,1.0);
   calib.add_sensor_steps(10,1.0);
   calib.add_sensor_steps(10,1.0);
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);

   calib.add_reference_value(500,10.0);
   EXPECT_NEAR(PE::MAX_ACCURACY, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);

   calib.add_sensor_steps(10,1.0);
   calib.add_sensor_steps(10,1.0);
   calib.add_sensor_steps(10,1.0);
   calib.add_sensor_steps(10,1.0);
   calib.add_sensor_steps(10,1.0);
   calib.add_sensor_steps(10,1.0);
   calib.add_reference_value(600,10.0);
   EXPECT_NEAR(10.0, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(0.0, calib.get_sensor_calibration(), 0.01);

   calib.add_sensor_steps(10,1.0);
   calib.add_reference_value(100,10.0);
   EXPECT_NEAR(10.0, calib.get_sensor_accuracy(), 0.01);
   EXPECT_NEAR(100.0, calib.get_sensor_calibration(), 0.01);
   EXPECT_NEAR(10.0, calib.get_values_per_step(), 0.01);
}


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

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

