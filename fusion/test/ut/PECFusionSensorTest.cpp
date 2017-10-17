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
 * Unit test of the PECFusionSensor class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include "PECFusionSensor.h"

class PECFusionSensorTest : public ::testing::Test
{
public:
   virtual void SetUp()
   {}
   virtual void TearDown()
   {}

   const PE::SBasicSensor& getAngSpeed(const PE::CFusionSensor& fusion);
   const PE::SBasicSensor& getSpeed(const PE::CFusionSensor& fusion);
};


/**
 * create test
 */
TEST_F(PECFusionSensorTest, test_create )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(1000.0, PE::SPosition(50.0,10.0,0.1), PE::SBasicSensor(90.0,5.0));
   EXPECT_EQ(1000.0, fusion.GetTimestamp());

   EXPECT_TRUE(fusion.GetPosition().IsValid());
   EXPECT_EQ(50.0, fusion.GetPosition().Latitude);
   EXPECT_EQ(10.0, fusion.GetPosition().Longitude);
   EXPECT_EQ(0.1, fusion.GetPosition().HorizontalAcc);

   EXPECT_TRUE(fusion.GetHeading().IsValid());
   EXPECT_EQ(90.0, fusion.GetHeading().Value);
   EXPECT_EQ(5.0, fusion.GetHeading().Accuracy);

   EXPECT_FALSE(getAngSpeed(fusion).IsValid());
   EXPECT_FALSE(getSpeed(fusion).IsValid());
}

/**
 * Adding speed test
 */
TEST_F(PECFusionSensorTest, test_add_speed )
{
   //ToDo test send data without valid position
   //test data only speed
   //test first position is valid
}

/**
 * Adding angular speed test
 */
TEST_F(PECFusionSensorTest, test_add_angular_speed )
{
   //ToDo test send data without valid position
   //test data only angular velosity
   //test first position is valid
}

/**
 * Adding position test
 */
TEST_F(PECFusionSensorTest, test_add_position )
{
   //TS       Lat[deg]    Long[deg]   Head[deg]   hAcc[deg]   horAcc[m]   AngSpeed[deg/s]     Speed km/h
   //6216331  53,640096   10,004298   89,400002   0,3         2           -3,40340540540541   71,5450012809687
   PE::CFusionSensor fusion = PE::CFusionSensor(
                                 6216.331, //TS
                                 PE::SPosition(53.640096, 10.004298, 2), //Position
                                 PE::SBasicSensor(89.400002,0.3) //Heading
                              );
   //TS       Lat[deg]    Long[deg]   Head[deg]   hAcc[deg]   horAcc[m]   AngSpeed[deg/s]     Speed km/h
   //6217332  53,640094   10,0046     93          0,3         2           -3,5964015984016    71,6042112782731
   fusion.AddPosition(
             6217.332, //TS
             PE::SPosition(53.640094, 10.0046, 2), //Position
             PE::SBasicSensor(93.0,0.3) //Heading
          );
   EXPECT_EQ(6217.332, fusion.GetTimestamp());
   EXPECT_TRUE(fusion.GetPosition().IsValid());
   EXPECT_EQ(53.640094, fusion.GetPosition().Latitude);
   EXPECT_EQ(10.0046, fusion.GetPosition().Longitude);
   EXPECT_EQ(2, fusion.GetPosition().HorizontalAcc);
   EXPECT_TRUE(fusion.GetHeading().IsValid());
   EXPECT_EQ(93.0, fusion.GetHeading().Value);
   EXPECT_EQ(0.3, fusion.GetHeading().Accuracy);
   EXPECT_TRUE(getAngSpeed(fusion).IsValid());
   EXPECT_NEAR(-3.596, getAngSpeed(fusion).Value, 0.001);
   EXPECT_NEAR(0.01, getAngSpeed(fusion).Accuracy, 0.01);
   EXPECT_TRUE(getSpeed(fusion).IsValid());
   EXPECT_NEAR(19.89, getSpeed(fusion).Value, 0.001);
   EXPECT_NEAR(0.01, getSpeed(fusion).Accuracy, 0.01);

   //TS       Lat[deg]    Long[deg]   Head[deg]   hAcc[deg]   horAcc[m]   AngSpeed[deg/s]     Speed km/h
   //6218331  53,640079   10,004898   97          0,3         1           -4,004004004004     71,0467784073282
   fusion.AddPosition(
             6218.331, //TS
             PE::SPosition(53.640079, 10.004898, 1), //Position
             PE::SBasicSensor(97.0,0.3) //Heading
          );
   EXPECT_EQ(6218.331, fusion.GetTimestamp());
   EXPECT_TRUE(fusion.GetPosition().IsValid());
   EXPECT_EQ(53.0, fusion.GetPosition().Latitude);
   EXPECT_EQ(10.0, fusion.GetPosition().Longitude);
   EXPECT_EQ(0, fusion.GetPosition().HorizontalAcc);
   EXPECT_TRUE(fusion.GetHeading().IsValid());
   EXPECT_EQ(97.0, fusion.GetHeading().Value);
   EXPECT_EQ(0.3, fusion.GetHeading().Accuracy);
   EXPECT_TRUE(getAngSpeed(fusion).IsValid());
   EXPECT_NEAR(-4.004, getAngSpeed(fusion).Value, 0.001);
   EXPECT_NEAR(0.01, getAngSpeed(fusion).Accuracy, 0.01);
   EXPECT_TRUE(getSpeed(fusion).IsValid());
   EXPECT_NEAR(19.73, getSpeed(fusion).Value, 0.001);
   EXPECT_NEAR(0.01, getSpeed(fusion).Accuracy, 0.01);
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

