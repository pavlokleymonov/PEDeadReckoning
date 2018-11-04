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
   PE::CCoreSimple core = PE::CCoreSimple(PE::SPosition(), PE::SBasicSensor());
   EXPECT_FALSE(core.GetOdoCfg().IsValid());
   EXPECT_FALSE(core.GetGyroCfg().IsValid());
   EXPECT_FALSE(core.GetPosition().IsValid());
   EXPECT_FALSE(core.GetHeading().IsValid());
   EXPECT_FALSE(core.GetSpeed().IsValid());
}


/**
 * test
 */
TEST_F(PECCoreSimpleTest, no_valid_sensors_cfg_test )
{
   PE::CCoreSimple core = PE::CCoreSimple(PE::SPosition(), PE::SBasicSensor());

   //no set configuration
   EXPECT_FALSE(core.GetOdoCfg().IsValid());
   EXPECT_FALSE(core.GetGyroCfg().IsValid());

   //set invalid config
   PE::CSensorCfg cfgOdo = PE::CSensorCfg();
   core.SetOdoCfg(cfgOdo,2,1);
   PE::CSensorCfg cfgGyro = PE::CSensorCfg();
   core.SetGyroCfg(cfgGyro,2,1);
   EXPECT_FALSE(core.GetOdoCfg().IsValid());
   EXPECT_FALSE(core.GetGyroCfg().IsValid());

   core.AddOdo(1.000, PE::SBasicSensor(1000, 0.1));
   core.AddGyro(1.001, PE::SBasicSensor(1, 0.1));
   core.AddOdo(2.000, PE::SBasicSensor(2000, 0.1));
   core.AddGyro(2.001, PE::SBasicSensor(2, 0.1));
   core.AddOdo(3.000, PE::SBasicSensor(3000, 0.1));
   core.AddGyro(3.001, PE::SBasicSensor(3, 0.1));

   EXPECT_FALSE(core.GetPosition().IsValid());
   EXPECT_FALSE(core.GetHeading().IsValid());
   EXPECT_FALSE(core.GetSpeed().IsValid());
}


/**
 * test
 */
TEST_F(PECCoreSimpleTest, valid_odo_cfg_test )
{
   PE::CCoreSimple core = PE::CCoreSimple(PE::SPosition(), PE::SBasicSensor());
   PE::CSensorCfg cfgOdo1 = PE::CSensorCfg(PE::CSensorCfg::ToCFG("CFGSENSOR,5,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX"));
   core.SetOdoCfg(cfgOdo1, 2, 1); //SENSOR_ODOMETER_AXIS = 5
   EXPECT_TRUE(core.GetOdoCfg().IsValid());
   EXPECT_EQ(99.5, core.GetOdoCfg().GetLimit());

   //rewrite new config
   PE::CSensorCfg cfgOdo2 = PE::CSensorCfg(PE::CSensorCfg::ToCFG("CFGSENSOR,5,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,90.0,XX"));
   core.SetOdoCfg(cfgOdo2, 2, 1); //SENSOR_ODOMETER_AXIS = 5
   EXPECT_TRUE(core.GetOdoCfg().IsValid());
   EXPECT_EQ(90.0, core.GetOdoCfg().GetLimit());

}


/**
 * test
 */
TEST_F(PECCoreSimpleTest, valid_gyro_cfg_test )
{
   PE::CCoreSimple core = PE::CCoreSimple(PE::SPosition(), PE::SBasicSensor());

   PE::CSensorCfg cfgGyro1 = PE::CSensorCfg(PE::CSensorCfg::ToCFG("CFGSENSOR,6,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,99.5,XX"));
   core.SetGyroCfg(cfgGyro1, 2, 1); //SENSOR_GYRO_Z = 6
   EXPECT_TRUE(core.GetGyroCfg().IsValid());
   EXPECT_EQ(99.5, core.GetOdoCfg().GetLimit());

   //rewrite new config
   PE::CSensorCfg cfgGyro2 = PE::CSensorCfg(PE::CSensorCfg::ToCFG("CFGSENSOR,6,1.00000000,0.00000000,100.0,1,0.00000000,0.00000000,100.0,1,90.1,XX"));
   core.SetGyroCfg(cfgGyro2, 2, 1); //SENSOR_GYRO_Z = 6
   EXPECT_TRUE(core.GetGyroCfg().IsValid());
   EXPECT_EQ(90.1, core.GetGyroCfg().GetLimit());
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
