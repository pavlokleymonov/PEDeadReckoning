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
TEST_F(PECSensorCfgTest, create_test )
{
   PE::CSensorCfg cfg_default;
   EXPECT_FALSE(cfg_default.IsValid());
   EXPECT_EQ("CFGSENSOR,0,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX", PE::CSensorCfg::ToSTR(cfg_default) );

   PE::CSensorCfg cfg_default_limit(PE::SENSOR_GYRO_Z, PE::CNormCfg(123.456,1.23,85.5,1000),PE::CNormCfg(0.123456,0.0123,85.5,1000));
   EXPECT_TRUE(cfg_default_limit.IsValid());
   EXPECT_EQ("CFGSENSOR,6,123.45600000,1.23000000,85.5,1000,0.12345600,0.01230000,85.5,1000,99.5,XX", PE::CSensorCfg::ToSTR(cfg_default_limit) );

   PE::CSensorCfg cfg(PE::SENSOR_HEADING, PE::CNormCfg(123.456,1.23,85.5,1000),PE::CNormCfg(0.123456,0.0123,85.5,1000), 33.33);
   EXPECT_TRUE(cfg.IsValid());
   EXPECT_EQ("CFGSENSOR,3,123.45600000,1.23000000,85.5,1000,0.12345600,0.01230000,85.5,1000,33.3,XX", PE::CSensorCfg::ToSTR(cfg) );
}


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
   //case empty string
   EXPECT_FALSE( PE::CSensorCfg::ToCFG("").IsValid() );

   //case wrong format string
   EXPECT_FALSE( PE::CSensorCfg::ToCFG("1234567890").IsValid() );
   
   //case wrong marker and length string
   EXPECT_FALSE( PE::CSensorCfg::ToCFG("CFGSENSOR_,1,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,").IsValid() );

   //case load serialisation from default constructor
   std::string cfg_str = "CFGSENSOR,0,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX";
   EXPECT_FALSE( PE::CSensorCfg::ToCFG(cfg_str).IsValid() );
   EXPECT_EQ( PE::SENSOR_UNKNOWN, PE::CSensorCfg::ToCFG(cfg_str).GetType() );
   EXPECT_NEAR( 1.00000000, PE::CSensorCfg::ToCFG(cfg_str).GetScale().mAccValue, 0.00000001 );
   EXPECT_NEAR( 0.00000000, PE::CSensorCfg::ToCFG(cfg_str).GetScale().mAccMld  , 0.00000001 );
   EXPECT_NEAR(      100.0, PE::CSensorCfg::ToCFG(cfg_str).GetScale().mAccRel  , 0.1 );
   EXPECT_NEAR(          1, PE::CSensorCfg::ToCFG(cfg_str).GetScale().mCount   , 1.0 );
   EXPECT_NEAR( 0.00000000, PE::CSensorCfg::ToCFG(cfg_str).GetBase().mAccValue , 0.00000001 );
   EXPECT_NEAR( 0.00000000, PE::CSensorCfg::ToCFG(cfg_str).GetBase().mAccMld   , 0.00000001 );
   EXPECT_NEAR(      100.0, PE::CSensorCfg::ToCFG(cfg_str).GetBase().mAccRel   , 0.1 );
   EXPECT_NEAR(          1, PE::CSensorCfg::ToCFG(cfg_str).GetBase().mCount    , 1.0 );
   EXPECT_NEAR(       99.5, PE::CSensorCfg::ToCFG(cfg_str).GetLimit()          , 0.1 );

   //case: unknown sensor type
   cfg_str = "CFGSENSOR,1,1.01000000,20.01200000,300.0,4000,50000.01234500,600000.01234560,7000000.0,80000000,99.3,XX";
   EXPECT_TRUE( PE::CSensorCfg::ToCFG(cfg_str).IsValid() );
   EXPECT_EQ( PE::SENSOR_LATITUDE, PE::CSensorCfg::ToCFG(cfg_str).GetType() );
   EXPECT_NEAR(    1.01000000, PE::CSensorCfg::ToCFG(cfg_str).GetScale().mAccValue, 0.00000001 );
   EXPECT_NEAR(   20.01200000, PE::CSensorCfg::ToCFG(cfg_str).GetScale().mAccMld  , 0.00000001 );
   EXPECT_NEAR(         300.0, PE::CSensorCfg::ToCFG(cfg_str).GetScale().mAccRel  , 0.1 );
   EXPECT_NEAR(          4000, PE::CSensorCfg::ToCFG(cfg_str).GetScale().mCount   , 1.0 );
   EXPECT_NEAR(  50000.012345, PE::CSensorCfg::ToCFG(cfg_str).GetBase().mAccValue , 0.00000001 );
   EXPECT_NEAR(600000.0123456, PE::CSensorCfg::ToCFG(cfg_str).GetBase().mAccMld   , 0.00000001 );
   EXPECT_NEAR(     7000000.0, PE::CSensorCfg::ToCFG(cfg_str).GetBase().mAccRel   , 0.1 );
   EXPECT_NEAR(      80000000, PE::CSensorCfg::ToCFG(cfg_str).GetBase().mCount    , 1.0 );
   EXPECT_NEAR(          99.3, PE::CSensorCfg::ToCFG(cfg_str).GetLimit()          , 0.1 );
}


/**
 * test
 */
TEST_F(PECSensorCfgTest, ToCFG_to_ToSTR_test )
{
   std::string cfg_str = "CFGSENSOR,1,1.01000000,20.01200000,300.0,4000,50000.01234500,600000.01234560,7000000.0,80000000,99.3,XX";
   PE::CSensorCfg cfg = PE::CSensorCfg::ToCFG(cfg_str);
   EXPECT_EQ(cfg_str, PE::CSensorCfg::ToSTR(cfg));
}


/**
 * test
 */
TEST_F(PECSensorCfgTest, Wrong_cfg_marker_test )
{
   std::string cfg_str = "CFGSENSOR_,1,1.01000000,20.01200000,300.0,4000,50000.01234500,600000.01234560,7000000.0,80000000,99.3,XX";
   EXPECT_FALSE(PE::CSensorCfg::ToCFG(cfg_str).IsValid());
}


/**
 * test
 */
TEST_F(PECSensorCfgTest, Wrong_cfg_length_test )
{
   std::string cfg_str = "CFGSENSOR,1,1.01000000,20.01200000,300.0,4000,50000.01234500,600000.01234560,7000000.0,80000000,99.3,,XX";
   EXPECT_FALSE(PE::CSensorCfg::ToCFG(cfg_str).IsValid());
}


/**
 * test
 */
TEST_F(PECSensorCfgTest, ToNormalisation_test )
{
   std::string cfg_str = "CFGSENSOR,1,1.01000000,20.01200000,300.0,4000,50000.01234500,600000.01234560,7000000.0,80000000,99.3,XX";
   
   EXPECT_TRUE(PE::CSensorCfg::ToCFG(cfg_str).IsValid());
   EXPECT_NEAR( 1.01000000,PE::CSensorCfg::ToNormalisation(PE::CSensorCfg::ToCFG(cfg_str).GetScale()).GetAccumulatedValue()   , 0.00000001);
   EXPECT_NEAR(20.01200000,PE::CSensorCfg::ToNormalisation(PE::CSensorCfg::ToCFG(cfg_str).GetScale()).GetAccumulatedMld()     , 0.00000001);
   EXPECT_NEAR(      300.0,PE::CSensorCfg::ToNormalisation(PE::CSensorCfg::ToCFG(cfg_str).GetScale()).GetAccumulatedReliable(), 0.00000001);
   EXPECT_NEAR(     4000.0,PE::CSensorCfg::ToNormalisation(PE::CSensorCfg::ToCFG(cfg_str).GetScale()).GetSampleCount()        , 0.00000001);
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
