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
  * Unit test for the class CSensorCfg.
  *
  * Code under test:
  *
  */
 
#include <gtest/gtest.h>
#include "PECSensorCfg.h"


class PECSensorCfgTest : public ::testing::Test
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
TEST_F(PECSensorCfgTest, ToSTR_test )
{
   //case default constructor
   EXPECT_EQ("CFGSENSOR,0,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX", PE::CSensorCfg::ToSTR(PE::CSensorCfg()) );

   //case: unknown sensor type
   EXPECT_EQ("CFGSENSOR,0,0.00000000,0.00000000,0.0,0,0.00000000,0.00000000,0.0,0,99.5,XX", PE::CSensorCfg::ToSTR(PE::CSensorCfg(PE::SENSOR_UNKNOWN,PE::CNormCfg(),PE::CNormCfg())) );

   //case: all value different
   EXPECT_EQ("CFGSENSOR,1,1.01000000,20.01200000,300.0,4000,50000.01234500,600000.01234560,7000000.0,80000000,99.3,XX", PE::CSensorCfg::ToSTR(PE::CSensorCfg(PE::SENSOR_LATITUDE,PE::CNormCfg(1.01,20.012,300.0123,4000.01234),PE::CNormCfg(50000.012345,600000.0123456,7000000.01234567,80000000.012345678),99.345)));
}

/**
 * test
 */
TEST_F(PECSensorCfgTest, ToCFG_test )
{
   //case load serialisation from default constructor
   EXPECT_FALSE( PE::CSensorCfg::ToCFG("CFGSENSOR,0,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX").IsValid() );
   EXPECT_EQ( PE::SENSOR_UNKNOWN, PE::CSensorCfg::ToCFG("CFGSENSOR,0,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX").GetType() );
   EXPECT_NEAR( 1.00000000, PE::CSensorCfg::ToCFG("CFGSENSOR,0,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX").GetScale().mAccValue, 0.00000001 );
   EXPECT_NEAR( 0.00000000, PE::CSensorCfg::ToCFG("CFGSENSOR,0,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX").GetScale().mAccMld, 0.00000001 );
   EXPECT_NEAR( 100.0, PE::CSensorCfg::ToCFG("CFGSENSOR,0,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX").GetScale().mAccRel, 0.1 );
   EXPECT_NEAR( 1, PE::CSensorCfg::ToCFG("CFGSENSOR,0,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX").GetScale().mCount, 1.0 );
   EXPECT_NEAR( 0.00000000, PE::CSensorCfg::ToCFG("CFGSENSOR,0,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX").GetBase().mAccValue, 0.00000001 );
   EXPECT_NEAR( 0.00000000, PE::CSensorCfg::ToCFG("CFGSENSOR,0,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX").GetBase().mAccMld, 0.00000001 );
   EXPECT_NEAR( 100.0, PE::CSensorCfg::ToCFG("CFGSENSOR,0,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX").GetBase().mAccRel, 0.1 );
   EXPECT_NEAR( 1, PE::CSensorCfg::ToCFG("CFGSENSOR,0,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX").GetBase().mCount, 1.0 );
   EXPECT_NEAR( 99.5, PE::CSensorCfg::ToCFG("CFGSENSOR,0,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX").GetLimit(), 0.1 );

   //case: unknown sensor type
//    EXPECT_EQ("CFGSENSOR,0,0.00000000,0.00000000,0.0,0,0.00000000,0.00000000,0.0,0,99.5,XX", PE::CSensorCfg::ToSTR(PE::CSensorCfg(PE::SENSOR_UNKNOWN,PE::CNormCfg(),PE::CNormCfg())) );

   //case: all value different
//    EXPECT_EQ("CFGSENSOR,1,1.01000000,20.01200000,300.0,4000,50000.01234500,600000.01234560,7000000.0,80000000,99.3,XX", PE::CSensorCfg::ToSTR(PE::CSensorCfg(PE::SENSOR_LATITUDE,PE::CNormCfg(1.01,20.012,300.0123,4000.01234),PE::CNormCfg(50000.012345,600000.0123456,7000000.01234567,80000000.012345678),99.345)));
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
