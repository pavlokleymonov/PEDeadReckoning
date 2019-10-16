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
   EXPECT_EQ(0, PEStart(0));

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
   double gyroacc;
   //PEReceiveGyroStatus
   EXPECT_FALSE( PEReceiveGyroStatus(invalidInstance, gyrobase, gyroscale, gyrorel, gyroacc) );

   double odobase;
   double odoscale;
   double odorel;
   double odoacc;
   //PEReceiveOdoStatus
   EXPECT_FALSE( PEReceiveOdoStatus(invalidInstance, odobase, odoscale, odorel, odoacc) );
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
