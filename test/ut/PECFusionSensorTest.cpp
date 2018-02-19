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
#include "PETools.h"


class PECFusionSensorTest : public ::testing::Test
{
public:
   virtual void SetUp()
   {}
   virtual void TearDown()
   {}

   const PE::SBasicSensor& getAngSpeed(const PE::CFusionSensor& fusion)
   {
      return fusion.m_AngSpeed;
   }
   const PE::SBasicSensor& getSpeed(const PE::CFusionSensor& fusion)
   {
      return fusion.m_Speed;
   }
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
 * Test incorrect timestamp of position.
 */
TEST_F(PECFusionSensorTest, test_incorrect_timestamp_pos )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(10.0, PE::SPosition(50.0, 10.0, 1), PE::SBasicSensor( 90.0, 0.30 ));

   EXPECT_TRUE(fusion.GetPosition().IsValid());
   EXPECT_TRUE(fusion.GetHeading().IsValid());
   EXPECT_NEAR(10.0000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000000, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.000, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.300, fusion.GetHeading().Accuracy,0.001);

   //smaller timestamp - ignored
   fusion.AddPosition(9.9, PE::SPosition(50.0000000, 10.0000500, 1));
   EXPECT_NEAR(10.0000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000000, fusion.GetPosition().Longitude,0.0000001);

   //same timestamp
   fusion.AddPosition(10.0, PE::SPosition(50.0000000, 10.0001000, 1));
   EXPECT_NEAR(10.0000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000500, fusion.GetPosition().Longitude,0.0000001);

   //correct timestamp
   fusion.AddPosition(11.0, PE::SPosition(50.0000000, 10.0001500, 1));
   EXPECT_NEAR(11.0000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0001166, fusion.GetPosition().Longitude,0.0000001);
}


/**
 * Test longitudual driving with same heading and permanent speed. 
 *      input accuracy of position and speed the same
 */
// TEST_F(PECFusionSensorTest, test_longitudual_driving_same_acc_speed_and_pos )
// {
//    PE::CFusionSensor fusion = PE::CFusionSensor(10.0, PE::SPosition(50.0000000, 10.0000000, 1), PE::SBasicSensor( 90.0, 0.30 ));
//    EXPECT_NEAR(10.0000000, fusion.GetTimestamp(),0.0000001);
//    EXPECT_TRUE(fusion.GetPosition().IsValid());
//    EXPECT_TRUE(fusion.GetHeading().IsValid());
// 
//    //distance 3.574m speed 3.574m/s head 90deg
// 
//    fusion.AddPosition(11.0, PE::SPosition(50.0000000, 10.0000500, 1), PE::SBasicSensor( 90.0, 0.30 ), PE::SBasicSensor( 3.574, 1.0 ));
//    //timestamp
//    EXPECT_NEAR(11.0000000, fusion.GetTimestamp(),0.0000001);
//    //heading
//    EXPECT_NEAR(90.0000, fusion.GetHeading().Value,0.0001);
//    EXPECT_NEAR( 0.7865, fusion.GetHeading().Accuracy,0.0001);
//    //position
//    EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
//    EXPECT_NEAR(10.0000500, fusion.GetPosition().Longitude,0.0000001);
//    EXPECT_NEAR( 1.4000, fusion.GetPosition().HorizontalAcc,0.0001);
// 
//    fusion.AddPosition(12.0, PE::SPosition(50.0000000, 10.0001000, 1), PE::SBasicSensor( 90.0, 0.30 ), PE::SBasicSensor( 3.574, 1.0 ));
//    //timestamp
//    EXPECT_NEAR(12.0000000, fusion.GetTimestamp(),0.0000001);
//    //heading
//    EXPECT_NEAR(90.0000, fusion.GetHeading().Value,0.0001);
//    EXPECT_NEAR( 0.9416, fusion.GetHeading().Accuracy,0.0001);
//    //position
//    EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
//    EXPECT_NEAR(10.0001000, fusion.GetPosition().Longitude,0.0000001);
//    EXPECT_NEAR( 1.5290, fusion.GetPosition().HorizontalAcc,0.0001);
// 
//    fusion.AddPosition(13.0, PE::SPosition(50.0000000, 10.0001500, 1), PE::SBasicSensor( 90.0, 0.30 ), PE::SBasicSensor( 3.574, 1.0 ));
//    //timestamp
//    EXPECT_NEAR(13.0000000, fusion.GetTimestamp(),0.0000001);
//    //heading
//    EXPECT_NEAR(90.0000, fusion.GetHeading().Value,0.0001);
//    EXPECT_NEAR( 0.9856, fusion.GetHeading().Accuracy,0.0001);
//    //position
//    EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
//    EXPECT_NEAR(10.0001500, fusion.GetPosition().Longitude,0.0000001);
//    EXPECT_NEAR( 1.5646, fusion.GetPosition().HorizontalAcc,0.0001);
// 
//    fusion.AddPosition(14.0, PE::SPosition(50.0000000, 10.0002000, 1), PE::SBasicSensor( 90.0, 0.30 ), PE::SBasicSensor( 3.574, 1.0 ));
//    //timestamp
//    EXPECT_NEAR(14.0000000, fusion.GetTimestamp(),0.0000001);
//    //heading
//    EXPECT_NEAR(90.0000, fusion.GetHeading().Value,0.0001);
//    EXPECT_NEAR( 0.9993, fusion.GetHeading().Accuracy,0.0001);
//    //position
//    EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
//    EXPECT_NEAR(10.0002000, fusion.GetPosition().Longitude,0.0000001);
//    EXPECT_NEAR( 1.5744, fusion.GetPosition().HorizontalAcc,0.0001);
// 
//    fusion.AddPosition(15.0, PE::SPosition(50.0000000, 10.0002500, 1), PE::SBasicSensor( 90.0, 0.30 ), PE::SBasicSensor( 3.574, 1.0 ));
//    //timestamp
//    EXPECT_NEAR(15.0000000, fusion.GetTimestamp(),0.0000001);
//    //heading
//    EXPECT_NEAR(90.0000, fusion.GetHeading().Value,0.0001);
//    EXPECT_NEAR( 1.0037, fusion.GetHeading().Accuracy,0.0001);
//    //position
//    EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
//    EXPECT_NEAR(10.0002500, fusion.GetPosition().Longitude,0.0000001);
//    EXPECT_NEAR( 1.5770, fusion.GetPosition().HorizontalAcc,0.0001);
// }
// 
// 
// /**
//  * Test longitudual driving with same heading and permanent speed. 
//  *      different input accuracy of position and speed
//  */
// TEST_F(PECFusionSensorTest, test_longitudual_driving_speed_acc_1_pos_acc_1_till_5 )
// {
//    PE::CFusionSensor fusion = PE::CFusionSensor(10.0, PE::SPosition(50.0000000, 10.0000000, 1), PE::SBasicSensor( 90.0, 0.30 ));
//    EXPECT_NEAR(10.0000000, fusion.GetTimestamp(),0.0000001);
//    EXPECT_TRUE(fusion.GetPosition().IsValid());
//    EXPECT_TRUE(fusion.GetHeading().IsValid());
// 
//    //distance 3.574m speed 3.574m/s head 90deg
// 
//    fusion.AddPosition(11.0, PE::SPosition(50.00000000, 10.00005000, 1), PE::SBasicSensor( 90.0, 0.30 ), PE::SBasicSensor( 3.574, 0.001 ));
//    //timestamp
//    EXPECT_NEAR(11.0000000, fusion.GetTimestamp(),0.0000001);
//    //heading
//    EXPECT_NEAR(90.0000, fusion.GetHeading().Value,0.0001);
//    EXPECT_NEAR( 0.7865, fusion.GetHeading().Accuracy,0.0001);
//    //position
//    EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
//    EXPECT_NEAR(10.0000500, fusion.GetPosition().Longitude,0.0000001);
//    EXPECT_NEAR( 1.0009, fusion.GetPosition().HorizontalAcc,0.0001);
// 
//    fusion.AddPosition(12.0, PE::SPosition(50.00000000, 10.00010000, 2), PE::SBasicSensor( 90.0, 0.30 ), PE::SBasicSensor( 3.574, 0.001 ));
//    //timestamp
//    EXPECT_NEAR(12.0000000, fusion.GetTimestamp(),0.0000001);
//    //heading
//    EXPECT_NEAR(90.0000, fusion.GetHeading().Value,0.0001);
//    EXPECT_NEAR( 0.9436, fusion.GetHeading().Accuracy,0.0001);
//    //position
//    EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
//    EXPECT_NEAR(10.0001000, fusion.GetPosition().Longitude,0.0000001);
//    EXPECT_NEAR( 1.3365, fusion.GetPosition().HorizontalAcc,0.0001);
// 
//    fusion.AddPosition(13.0, PE::SPosition(50.00000000, 10.00015000, 3), PE::SBasicSensor( 90.0, 0.30 ), PE::SBasicSensor( 3.574, 0.001 ));
//    //timestamp
//    EXPECT_NEAR(13.0000000, fusion.GetTimestamp(),0.0000001);
//    //heading
//    EXPECT_NEAR(90.0000, fusion.GetHeading().Value,0.0001);
//    EXPECT_NEAR( 0.9901, fusion.GetHeading().Accuracy,0.0001);
//    //position
//    EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
//    EXPECT_NEAR(10.0001500, fusion.GetPosition().Longitude,0.0000001);
//    EXPECT_NEAR( 1.8520, fusion.GetPosition().HorizontalAcc,0.0001);
// 
//    fusion.AddPosition(14.0, PE::SPosition(50.00000000, 10.00020000, 4), PE::SBasicSensor( 90.0, 0.30 ), PE::SBasicSensor( 3.574, 0.001 ));
//    //timestamp
//    EXPECT_NEAR(14.0000000, fusion.GetTimestamp(),0.0000001);
//    //heading
//    EXPECT_NEAR(90.0000, fusion.GetHeading().Value,0.0001);
//    EXPECT_NEAR( 1.0055, fusion.GetHeading().Accuracy,0.0001);
//    //position
//    EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
//    EXPECT_NEAR(10.0002000, fusion.GetPosition().Longitude,0.0000001);
//    EXPECT_NEAR( 2.5346, fusion.GetPosition().HorizontalAcc,0.0001);
// 
//    fusion.AddPosition(15.0, PE::SPosition(50.00000000, 10.00025000, 5), PE::SBasicSensor( 90.0, 0.30 ), PE::SBasicSensor( 3.574, 0.001 ));
//    //timestamp
//    EXPECT_NEAR(15.0000000, fusion.GetTimestamp(),0.0000001);
//    //heading
//    EXPECT_NEAR(90.0000, fusion.GetHeading().Value,0.0001);
//    EXPECT_NEAR( 1.0109, fusion.GetHeading().Accuracy,0.0001);
//    //position
//    EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
//    EXPECT_NEAR(10.0002500, fusion.GetPosition().Longitude,0.0000001);
//    EXPECT_NEAR( 3.3665, fusion.GetPosition().HorizontalAcc,0.0001);
// }
// 
// /**
//  * Test drive straightforward no GPS signal no turns
//  */
// TEST_F(PECFusionSensorTest, test_drive_straightforward )
// {
//    PE::CFusionSensor fusion = PE::CFusionSensor(10.0, PE::SPosition(50.0000000, 10.0000000, 1), PE::SBasicSensor( 90.0, 0.30 ));
// 
//    //speed 3.574m/s gyro 0
//    fusion.AddSpeed(10.25, PE::SBasicSensor( 3.574, 0.001 ));
//    fusion.AddAngSpeed(10.25, PE::SBasicSensor(0.0, 0.001 ));
//    fusion.AddSpeed(10.50, PE::SBasicSensor( 3.574, 0.001 ));
//    fusion.AddAngSpeed(10.50, PE::SBasicSensor(0.0, 0.001 ));
//    fusion.AddSpeed(10.75, PE::SBasicSensor( 3.574, 0.001 ));
//    fusion.AddAngSpeed(10.75, PE::SBasicSensor(0.0, 0.001 ));
//    fusion.AddSpeed(11.00, PE::SBasicSensor( 3.574, 0.001 ));
//    fusion.AddAngSpeed(11.00, PE::SBasicSensor(0.0, 0.001 ));
//    //timestamp
//    EXPECT_NEAR(11.0000000, fusion.GetTimestamp(),0.0000001);
//    //heading
//    EXPECT_NEAR(90.0000, fusion.GetHeading().Value,0.0001);
//    EXPECT_NEAR( 0.3010, fusion.GetHeading().Accuracy,0.0001);
//    //position
//    EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
//    EXPECT_NEAR(10.0000500, fusion.GetPosition().Longitude,0.0000001);
//    EXPECT_NEAR( 1.0011, fusion.GetPosition().HorizontalAcc,0.0001);
// 
//    fusion.AddSpeed(11.25, PE::SBasicSensor( 3.574, 0.001 ));
//    fusion.AddAngSpeed(11.25, PE::SBasicSensor(0.0, 0.001 ));
//    fusion.AddSpeed(11.50, PE::SBasicSensor( 3.574, 0.001 ));
//    fusion.AddAngSpeed(11.50, PE::SBasicSensor(0.0, 0.001 ));
//    fusion.AddSpeed(11.75, PE::SBasicSensor( 3.574, 0.001 ));
//    fusion.AddAngSpeed(11.75, PE::SBasicSensor(0.0, 0.001 ));
//    fusion.AddSpeed(12.00, PE::SBasicSensor( 3.574, 0.001 ));
//    fusion.AddAngSpeed(12.00, PE::SBasicSensor(0.0, 0.001 ));
//    //timestamp
//    EXPECT_NEAR(12.0000000, fusion.GetTimestamp(),0.0000001);
//    //heading
//    EXPECT_NEAR(90.0000, fusion.GetHeading().Value,0.0001);
//    EXPECT_NEAR( 0.3025, fusion.GetHeading().Accuracy,0.0001);
//    //position
//    EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
//    EXPECT_NEAR(10.0001000, fusion.GetPosition().Longitude,0.0000001);
//    EXPECT_NEAR( 1.0023, fusion.GetPosition().HorizontalAcc,0.0001);
// 
//    fusion.AddSpeed(12.25, PE::SBasicSensor( 3.574, 0.001 ));
//    fusion.AddAngSpeed(12.25, PE::SBasicSensor(0.0, 0.001 ));
//    fusion.AddSpeed(12.50, PE::SBasicSensor( 3.574, 0.001 ));
//    fusion.AddAngSpeed(12.50, PE::SBasicSensor(0.0, 0.001 ));
//    fusion.AddSpeed(12.75, PE::SBasicSensor( 3.574, 0.001 ));
//    fusion.AddAngSpeed(12.75, PE::SBasicSensor(0.0, 0.001 ));
//    fusion.AddSpeed(13.00, PE::SBasicSensor( 3.574, 0.001 ));
//    fusion.AddAngSpeed(13.00, PE::SBasicSensor(0.0, 0.001 ));
//    //timestamp
//    EXPECT_NEAR(13.0000000, fusion.GetTimestamp(),0.0000001);
//    //heading
//    EXPECT_NEAR(90.0000, fusion.GetHeading().Value,0.0001);
//    EXPECT_NEAR( 0.3040, fusion.GetHeading().Accuracy,0.0001);
//    //position
//    EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
//    EXPECT_NEAR(10.0001500, fusion.GetPosition().Longitude,0.0000001);
//    EXPECT_NEAR( 1.0035, fusion.GetPosition().HorizontalAcc,0.0001);
// }
// 







/**
 * Test round drive no GPS signal
 */
/*
TEST_F(PECFusionSensorTest, test_drive_360_round_drive )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(10.0, PE::SPosition(50.0000000, 10.0000000, 1), PE::SBasicSensor( 90.0, 0.30 ));

   //speed 3.574m/s gyro 45grad/s
   fusion.AddSpeed(11.00, PE::SBasicSensor( 3.574, 0.001 ));
   fusion.AddAngSpeed(11.00, PE::SBasicSensor(45, 0.001 ));
   EXPECT_NEAR(0.0000, fusion.GetHeading().Value,0.0001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Longitude,0.0000001);

   fusion.AddSpeed(12.00, PE::SBasicSensor( 3.574, 0.001 ));
   fusion.AddAngSpeed(12.00, PE::SBasicSensor(45.0, 0.001 ));
   EXPECT_NEAR(0.0000, fusion.GetHeading().Value,0.0001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Longitude,0.0000001);

   fusion.AddSpeed(13.00, PE::SBasicSensor( 3.574, 0.001 ));
   fusion.AddAngSpeed(13.00, PE::SBasicSensor(45.0, 0.001 ));
   EXPECT_NEAR(0.0000, fusion.GetHeading().Value,0.0001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Longitude,0.0000001);

   fusion.AddSpeed(14.00, PE::SBasicSensor( 3.574, 0.001 ));
   fusion.AddAngSpeed(14.00, PE::SBasicSensor(45.0, 0.001 ));
   EXPECT_NEAR(0.0000, fusion.GetHeading().Value,0.0001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Longitude,0.0000001);

   fusion.AddSpeed(15.00, PE::SBasicSensor( 3.574, 0.001 ));
   fusion.AddAngSpeed(15.00, PE::SBasicSensor(45.0, 0.001 ));
   EXPECT_NEAR(0.0000, fusion.GetHeading().Value,0.0001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Longitude,0.0000001);

   fusion.AddSpeed(16.00, PE::SBasicSensor( 3.574, 0.001 ));
   fusion.AddAngSpeed(16.00, PE::SBasicSensor(45.0, 0.001 ));
   EXPECT_NEAR(0.0000, fusion.GetHeading().Value,0.0001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Longitude,0.0000001);

   fusion.AddSpeed(17.00, PE::SBasicSensor( 3.574, 0.001 ));
   fusion.AddAngSpeed(17.00, PE::SBasicSensor(45.0, 0.001 ));
   EXPECT_NEAR(0.0000, fusion.GetHeading().Value,0.0001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Longitude,0.0000001);

   fusion.AddSpeed(18.00, PE::SBasicSensor( 3.574, 0.001 ));
   fusion.AddAngSpeed(18.00, PE::SBasicSensor(45.0, 0.001 ));
   EXPECT_NEAR(0.0000, fusion.GetHeading().Value,0.0001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Longitude,0.0000001);

   fusion.AddSpeed(19.00, PE::SBasicSensor( 3.574, 0.001 ));
   fusion.AddAngSpeed(19.00, PE::SBasicSensor(45.0, 0.001 ));
   EXPECT_NEAR(0.0000, fusion.GetHeading().Value,0.0001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(0.0000000, fusion.GetPosition().Longitude,0.0000001);
}
*/

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

