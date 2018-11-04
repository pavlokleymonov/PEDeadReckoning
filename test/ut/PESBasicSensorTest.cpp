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
 * Unit test of the PESBasicSensor class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PESBasicSensor.h"

class PESBasicSensorTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }
};


//Test Default constructor
TEST_F(PESBasicSensorTest, default_constructor_test)
{
   PE::SBasicSensor sens;

   EXPECT_EQ(PE::MAX_VALUE, sens.Value);
   EXPECT_EQ(PE::MAX_ACCURACY, sens.Accuracy);
   EXPECT_FALSE(sens.IsValid());
}


//Test full constructor
TEST_F(PESBasicSensorTest, full_constructor_test)
{
   PE::SBasicSensor sens_valid = PE::SBasicSensor(11.1,0.1);

   EXPECT_TRUE(sens_valid.IsValid());
   EXPECT_EQ(11.1, sens_valid.Value);
   EXPECT_EQ( 0.1, sens_valid.Accuracy);

   PE::SBasicSensor sens_invalid_1 = PE::SBasicSensor(PE::MAX_VALUE, 0.2);
   EXPECT_FALSE(sens_invalid_1.IsValid());
   EXPECT_EQ(0.2, sens_invalid_1.Accuracy);

   PE::SBasicSensor sens_invalid_2 = PE::SBasicSensor(12.2,PE::MAX_ACCURACY);
   EXPECT_FALSE(sens_invalid_2.IsValid());
   EXPECT_EQ(12.2, sens_invalid_2.Value);
}


//Test operator==
TEST_F(PESBasicSensorTest, operator_eq_test)
{
   PE::SBasicSensor sen1;
   PE::SBasicSensor sen2;
   PE::SBasicSensor sen3(10.1111,0.2222);
   PE::SBasicSensor sen4(10.1111,0.2222);

   EXPECT_EQ(sen1,sen2);
   EXPECT_EQ(sen3,sen4);
   EXPECT_FALSE(sen1 == sen3);
   EXPECT_FALSE(sen4 == sen2);
   EXPECT_FALSE(PE::SBasicSensor(10,20) == PE::SBasicSensor(10,0));
   EXPECT_FALSE(PE::SBasicSensor( 0,20) == PE::SBasicSensor(10,20));
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
