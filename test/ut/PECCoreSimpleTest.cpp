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
#include "PECCoreSimple.h"


class PECCoreSimpleTest : public ::testing::Test
{
public:
   virtual void SetUp()
   {}
   virtual void TearDown()
   {}
};


/**
 * test
 */
TEST_F(PECCoreSimpleTest, create_test )
{
   PE::CCoreSimple core(PE::SPosition()/* invalid position */, PE::SBasicSensor()/* invalid heading */, 0.100/*100ms*/);
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
