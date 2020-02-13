/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2018 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */

 /**
  * Unit test for the class CCoreSimple.
  *
  * Code under test:
  *
  */
#include <string>
#include <gtest/gtest.h>
#include "PECore.h"


class PECoreTest : public ::testing::Test
{
public:
   virtual void SetUp()
   {}
   virtual void TearDown()
   {}
};


/**
 * test invalid position engine instance
 */
TEST_F(PECoreTest, invalid_instance_test )
{
   PECCore* invalidInstance = 0;

   //PEStop
   EXPECT_EQ(0, PEStop(invalidInstance));

   //PEClean
   EXPECT_FALSE( PEClean(invalidInstance) );

   //PECalculate
   EXPECT_FALSE( PECalculate(invalidInstance) );

   //PESendCoordinates
   EXPECT_FALSE( PESendCoordinates(invalidInstance, 0, 0, 0, 0) );

   //PESendHeading
   EXPECT_FALSE( PESendHeading(invalidInstance, 0, 0, 0) );

   //PESendSpeed
   EXPECT_FALSE( PESendSpeed(invalidInstance, 0, 0, 0) );

   //PESendGyro
   EXPECT_FALSE( PESendGyro(invalidInstance, 0, 0) );

   //PESendOdo
   EXPECT_FALSE( PESendOdo(invalidInstance, 0, 0) );

   double ts;
   double lat;
   double lon;
   double acc;
   double head;
   double headacc;
   double speed;
   double speedacc;
   //PEReceivePosition
   EXPECT_FALSE( PEReceivePosition(invalidInstance, ts, lat, lon, acc, head, headacc, speed, speedacc) );

   double dist;
   double distacc;
   //PEReceiveDistance
   EXPECT_FALSE( PEReceiveDistance(invalidInstance, dist, distacc) );

   double gyrobase;
   double gyroscale;
   double gyrorel;
   //PEReceiveGyroStatus
   EXPECT_FALSE( PEReceiveGyroStatus(invalidInstance, gyrobase, gyroscale, gyrorel) );

   double odobase;
   double odoscale;
   double odorel;
   //PEReceiveOdoStatus
   EXPECT_FALSE( PEReceiveOdoStatus(invalidInstance, odobase, odoscale, odorel) );
}


/**
 * test start/stop engine instance
 */
TEST_F(PECoreTest, start_stop_test )
{
   PECCore* pe = PEStart("test");


   //PEClean
   EXPECT_TRUE( PEClean(pe) );

   //PECalculate
   EXPECT_TRUE( PECalculate(pe) );

   //PESendCoordinates
   EXPECT_TRUE( PESendCoordinates(pe, 0, 0, 0, 0) );

   //PESendHeading
   EXPECT_TRUE( PESendHeading(pe, 0, 0, 0) );

   //PESendSpeed
   EXPECT_TRUE( PESendSpeed(pe, 0, 0, 0) );

   //PESendGyro
   EXPECT_TRUE( PESendGyro(pe, 0, 0) );

   //PESendOdo
   EXPECT_TRUE( PESendOdo(pe, 0, 0) );

   double ts;
   double lat;
   double lon;
   double acc;
   double head;
   double headacc;
   double speed;
   double speedacc;
   //PEReceivePosition
   EXPECT_TRUE( PEReceivePosition(pe, ts, lat, lon, acc, head, headacc, speed, speedacc) );

   double dist;
   double distacc;
   //PEReceiveDistance
   EXPECT_TRUE( PEReceiveDistance(pe, dist, distacc) );

   double gyrobase;
   double gyroscale;
   double gyrorel;
   //PEReceiveGyroStatus
   EXPECT_TRUE( PEReceiveGyroStatus(pe, gyrobase, gyroscale, gyrorel) );

   double odobase;
   double odoscale;
   double odorel;
   //PEReceiveOdoStatus
   EXPECT_TRUE( PEReceiveOdoStatus(pe, odobase, odoscale, odorel) );

   //PEStop
   EXPECT_EQ(std::string("test"), std::string(PEStop(pe)) );
}


/**
 * test acces to position engine instance after stop
 */
TEST_F(PECoreTest, after_stop_test )
{
   PECCore* pe = PEStart("test2");

   //PEStop
   EXPECT_EQ(std::string("test2"), std::string(PEStop(pe)) );

   //After stop access
   EXPECT_EQ(0, PEStop(pe) );

   //PEClean
   EXPECT_FALSE( PEClean(pe) );

   //PECalculate
   EXPECT_FALSE( PECalculate(pe) );

   //PESendCoordinates
   EXPECT_FALSE( PESendCoordinates(pe, 0, 0, 0, 0) );

   //PESendHeading
   EXPECT_FALSE( PESendHeading(pe, 0, 0, 0) );

   //PESendSpeed
   EXPECT_FALSE( PESendSpeed(pe, 0, 0, 0) );

   //PESendGyro
   EXPECT_FALSE( PESendGyro(pe, 0, 0) );

   //PESendOdo
   EXPECT_FALSE( PESendOdo(pe, 0, 0) );

   double ts;
   double lat;
   double lon;
   double acc;
   double head;
   double headacc;
   double speed;
   double speedacc;
   //PEReceivePosition
   EXPECT_FALSE( PEReceivePosition(pe, ts, lat, lon, acc, head, headacc, speed, speedacc) );

   double dist;
   double distacc;
   //PEReceiveDistance
   EXPECT_FALSE( PEReceiveDistance(pe, dist, distacc) );

   double gyrobase;
   double gyroscale;
   double gyrorel;
   //PEReceiveGyroStatus
   EXPECT_FALSE( PEReceiveGyroStatus(pe, gyrobase, gyroscale, gyrorel) );

   double odobase;
   double odoscale;
   double odorel;
   //PEReceiveOdoStatus
   EXPECT_FALSE( PEReceiveOdoStatus(pe, odobase, odoscale, odorel) );

}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
