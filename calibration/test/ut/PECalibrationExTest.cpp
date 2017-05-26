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
#include "mock_PENormalisation.h"
#include "PECalibrationEx.h"

using ::testing::_;
using ::testing::Eq;
using ::testing::ByRef;


class PECalibrationExTest : public ::testing::Test
{
public:
   virtual void SetUp()
   {}
   virtual void TearDown()
   {}
};

/**
* tests 6 sensors 2 valid reference
*/
TEST_F(PECalibrationExTest, _6_sensors_2_valid_ref_test)
{
   PE::Mock_normalisation mock;

   PE::calibration_ex calib(mock,10); //10 meters accuracy

   calib.add_sensor(1000); //has to be ignored before first valid ref
   calib.add_sensor(1000); //has to be ignored before first valid ref

   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   calib.add_sensor(500);
   calib.add_sensor(500);
   calib.add_reference(100,5); //added 100 meters with accuracy +/-5meters 
   EXPECT_CALL(mock,add_sensor(_));

   calib.add_sensor(500);
   calib.add_sensor(500);
}

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

