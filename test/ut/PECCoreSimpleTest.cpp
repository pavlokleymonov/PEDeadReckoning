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

   PE::TValue CalibratedScaleTo(const PE::CCoreSimple& core, PE::TSensorTypeID typeId)
   {
      return core.CalibratedScaleTo(typeId);
   }

   PE::TValue GetScale(const PE::CCoreSimple& core, PE::TSensorTypeID typeId)
   {
      return core.GetScale(typeId);
   }

   PE::TValue CalibratedBaseTo(const PE::CCoreSimple& core, PE::TSensorTypeID typeId)
   {
      return core.CalibratedBaseTo(typeId);
   }

   PE::TValue GetBase(const PE::CCoreSimple& core, PE::TSensorTypeID typeId)
   {
      return core.GetBase(typeId);
   }

   PE::SBasicSensor GetVale(const PE::CCoreSimple& core, PE::TSensorTypeID typeId, const PE::SBasicSensor& raw)
   {
      return core.GetVale(typeId, raw);
   }
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


/**
 * test odo calibration
 */
TEST_F(PECCoreSimpleTest, odo_calibration_test )
{
/*
   PE::CCoreSimple core = PE::CCoreSimple(PE::SPosition(50.0,10.0,1.0), PE::SBasicSensor());
   PE::CSensorCfg cfgOdo = PE::CSensorCfg(PE::CSensorCfg::ToCFG("CFGSENSOR,5,0.0,0.0,0.0,0,0.0,0.0,0.0,0,90.0,XX"));
   core.SetOdoCfg(cfgOdo, 2, 1); //SENSOR_ODOMETER_AXIS = 5 2 times +/- 1
   EXPECT_TRUE(core.GetOdoCfg().IsValid());

   core.AddOdo (1.0, PE::SBasicSensor( 2,0.1)); //2 ticks
   core.AddGnss(1.0, PE::SPosition(50.00000636,10.00000989,1.0), PE::SBasicSensor(45.0,0.1), PE::SBasicSensor(1.0,0.1)); //1m/s
   core.UpdatePosition();
   EXPECT_NEAR(0.0, core.GetSpeed().Value,0.000001);
   EXPECT_NEAR(0.0, core.GetSpeed().Accuracy,0.000001);

   core.AddOdo (2.0, PE::SBasicSensor( 4,0.1)); //4 ticks
   core.AddGnss(2.0, PE::SPosition(50.00001907,10.00002967,1.0), PE::SBasicSensor(45.0,0.1), PE::SBasicSensor(2.0,0.1)); //2m/s
   core.UpdatePosition();
   EXPECT_NEAR(0.0, core.GetSpeed().Value,0.000001);
   EXPECT_NEAR(0.0, core.GetSpeed().Accuracy,0.000001);


   core.AddOdo (3.0, PE::SBasicSensor( 6,0.1)); //6 ticks
   core.AddGnss(3.0, PE::SPosition(50.00003815,10.00005935,1.0), PE::SBasicSensor(45.0,0.1), PE::SBasicSensor(3.0,0.1)); //3m/s
   core.UpdatePosition();
   EXPECT_NEAR(0.0, core.GetSpeed().Value,0.000001);
   EXPECT_NEAR(0.0, core.GetSpeed().Accuracy,0.000001);


   core.AddOdo (4.0, PE::SBasicSensor( 4,0.1)); //8 ticks
   core.AddGnss(4.0, PE::SPosition(50.00006359,10.00009893,1.0), PE::SBasicSensor(45.0,0.1), PE::SBasicSensor(4.0,0.1)); //4m/s
   core.UpdatePosition();
   EXPECT_NEAR(0.0, core.GetSpeed().Value,0.000001);
   EXPECT_NEAR(0.0, core.GetSpeed().Accuracy,0.000001);


   core.AddOdo (5.0, PE::SBasicSensor( 5,0.1)); //10 ticks
   core.AddGnss(5.0, PE::SPosition(50.00009538,10.00014839,1.0), PE::SBasicSensor(45.0,0.1), PE::SBasicSensor(5.0,0.1)); //5m/s
   core.UpdatePosition();
   EXPECT_NEAR(0.0, core.GetSpeed().Value,0.000001);
   EXPECT_NEAR(0.0, core.GetSpeed().Accuracy,0.000001);


   core.AddOdo (6.0, PE::SBasicSensor( 6,0.1)); //12 ticks
   core.AddGnss(6.0, PE::SPosition(50.00013354,10.00020775,1.0), PE::SBasicSensor(45.0,0.1), PE::SBasicSensor(6.0,0.1)); //6m/s
   core.UpdatePosition();
   EXPECT_NEAR(0.0, core.GetSpeed().Value,0.000001);
   EXPECT_NEAR(0.0, core.GetSpeed().Accuracy,0.000001);


   core.AddOdo (7.0, PE::SBasicSensor( 7,0.1)); //14 ticks
   core.AddGnss(7.0, PE::SPosition(50.00017805,10.00027700,1.0), PE::SBasicSensor(45.0,0.1), PE::SBasicSensor(7.0,0.1)); //7m/s
   core.UpdatePosition();
   EXPECT_NEAR(0.0, core.GetSpeed().Value,0.000001);
   EXPECT_NEAR(0.0, core.GetSpeed().Accuracy,0.000001);


   core.AddOdo (8.0, PE::SBasicSensor( 8,0.1)); //16 ticks
   core.AddGnss(8.0, PE::SPosition(50.00022892,10.00035615,1.0), PE::SBasicSensor(45.0,0.1), PE::SBasicSensor(8.0,0.1)); //8m/s
   core.UpdatePosition();
   EXPECT_NEAR(0.0, core.GetSpeed().Value,0.000001);
   EXPECT_NEAR(0.0, core.GetSpeed().Accuracy,0.000001);


   core.AddOdo (9.0, PE::SBasicSensor( 9,0.1)); //18 ticks
   core.AddGnss(9.0, PE::SPosition(50.00028616,10.00044519,1.0), PE::SBasicSensor(45.0,0.1), PE::SBasicSensor(9.0,0.1)); //9m/s
   core.UpdatePosition();
   EXPECT_NEAR(0.0, core.GetSpeed().Value,0.000001);
   EXPECT_NEAR(0.0, core.GetSpeed().Accuracy,0.000001);


   core.AddOdo (10.0, PE::SBasicSensor(10,0.1)); //20 ticks
   core.AddGnss(10.0, PE::SPosition(50.00034975,10.00054412,1.0), PE::SBasicSensor(45.0,0.1), PE::SBasicSensor(10.0,0.1)); //10m/s
   core.UpdatePosition();
   EXPECT_NEAR(0.0, core.GetSpeed().Value,0.000001);
   EXPECT_NEAR(0.0, core.GetSpeed().Accuracy,0.000001);


   core.AddOdo (11.0, PE::SBasicSensor(8,0.1)); //16 ticks
   core.AddGnss(11.0, PE::SPosition(50.00040062,10.00062327,1.0), PE::SBasicSensor(45.0,0.1), PE::SBasicSensor(8.0,0.1)); //8m/s
   core.UpdatePosition();
   EXPECT_NEAR(0.0, core.GetSpeed().Value,0.000001);
   EXPECT_NEAR(0.0, core.GetSpeed().Accuracy,0.000001);


   core.AddOdo (12.0, PE::SBasicSensor(4,0.1)); //8 ticks
   core.AddGnss(12.0, PE::SPosition(50.00042606,10.00066284,1.0), PE::SBasicSensor(45.0,0.1), PE::SBasicSensor(4.0,0.1)); //4m/s
   core.UpdatePosition();
   EXPECT_NEAR(0.0, core.GetSpeed().Value,0.000001);
   EXPECT_NEAR(0.0, core.GetSpeed().Accuracy,0.000001);


   core.AddOdo (13.0, PE::SBasicSensor(9,0.1)); //18 ticks
   core.AddGnss(13.0, PE::SPosition(50.00048329,10.00075188,1.0), PE::SBasicSensor(45.0,0.1), PE::SBasicSensor(9.0,0.1)); //9m/s
   core.UpdatePosition();
   EXPECT_NEAR(0.0, core.GetSpeed().Value,0.000001);
   EXPECT_NEAR(0.0, core.GetSpeed().Accuracy,0.000001);

//    EXPECT_NEAR( 0.00, CalibratedScaleTo(core, PE::SENSOR_ODOMETER_AXIS),0.0000001);
//    EXPECT_NEAR( 0.00, GetScale(core, PE::SENSOR_ODOMETER_AXIS), 0.0000001 );
//    EXPECT_EQ("", PE::CSensorCfg::ToSTR(core.GetOdoCfg()));
*/
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
