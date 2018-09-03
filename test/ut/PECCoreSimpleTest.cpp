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


/**
 * test circle driving with the speed~98[km/h] distance 2[km]
 */
TEST_F(PECCoreSimpleTest, ideal_circle_driving_test )
{
   PE::CCoreSimple core = PE::CCoreSimple(PE::SPosition(), PE::SBasicSensor());

   PE::CSensorCfg cfgOdo =  PE::CSensorCfg::ToCFG("CFGSENSOR,5,0.0,0.0,0.0,0,0.0,0.0,0.0,0,99.5,XX");
   core.SetOdoCfg(cfgOdo, 2, 1); //SENSOR_ODOMETER_AXIS = 5, 2 times per one reference (5 sec)
   PE::CSensorCfg cfgGyro =  PE::CSensorCfg::ToCFG("CFGSENSOR,6,0.0,0.0,0.0,0,0.0,0.0,0.0,0,99.5,XX");
   core.SetGyroCfg(cfgGyro, 3, 1); //SENSOR_GYRO_Z = 6,        3 times per one reference (3.33 sec)

   //#0
   core.AddGnss( 1.00, PE::SPosition(49.99863456730125,9.996082730449409,0.1), PE::SBasicSensor(332.00, 0.1), PE::SBasicSensor(27.78,0.1));

   //1X
   core.AddOdo ( 1.10, PE::SBasicSensor(1000,1));
   core.AddGyro( 1.33, PE::SBasicSensor( 100,1));
   core.AddGyro( 4.66, PE::SBasicSensor( 100,1));
   core.AddOdo ( 6.10, PE::SBasicSensor(1000,1));
   core.AddGyro( 8.00, PE::SBasicSensor( 100,1));
   //#1
   core.AddGnss(10.00, PE::SPosition(50.00081217782849,9.995745334369854,0.1), PE::SBasicSensor(17.0, 0.1), PE::SBasicSensor(27.78,0.1));
   EXPECT_EQ(10.0, core.GetTimestamp());
   EXPECT_NEAR( 50.00081, core.GetPosition().Latitude, 0.00001);
   EXPECT_NEAR(  9.99574, core.GetPosition().Longitude, 0.00001);
   EXPECT_NEAR( 17.00, core.GetHeading().Value, 0.5);
   EXPECT_NEAR( 27.78, core.GetSpeed().Value, 0.5);
   EXPECT_NEAR(  0.00, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR(  0.00, core.GyroCalibratedTo(), 0.01);

   //1x
   core.AddOdo (11.10, PE::SBasicSensor(1000,1));
   core.AddGyro(11.33, PE::SBasicSensor( 100,1));
   core.AddGyro(14.66, PE::SBasicSensor( 100,1));
   core.AddOdo (16.10, PE::SBasicSensor(1000,1));
   core.AddGyro(18.00, PE::SBasicSensor( 100,1));
   //#2
   core.AddGnss(19.00, PE::SPosition(50.00250531810699,9.997902372885799,0.1), PE::SBasicSensor(62.00,0.1), PE::SBasicSensor(27.78,0.1));

   //2X
   core.AddOdo (21.10, PE::SBasicSensor(2000,1));
   core.AddGyro(21.33, PE::SBasicSensor( 200,1));
   core.AddGyro(24.66, PE::SBasicSensor( 200,1));
   core.AddOdo (26.10, PE::SBasicSensor(2000,1));
   core.AddGyro(28.00, PE::SBasicSensor( 200,1));
   //#3
   core.AddGnss(28.00, PE::SPosition(50.00133548545172,10.00392433729531,0.1), PE::SBasicSensor(152.00,0.1), PE::SBasicSensor(55.56,0.1));
   EXPECT_NEAR( 50.001335, core.GetPosition().Latitude, 0.00001);
   EXPECT_NEAR( 10.003924, core.GetPosition().Longitude, 0.00001);
   EXPECT_NEAR(  0.10, core.GetPosition().HorizontalAcc, 0.01);
   EXPECT_NEAR(152.00, core.GetHeading().Value, 0.1);
   EXPECT_NEAR(  0.12, core.GetHeading().Accuracy, 0.1);
   EXPECT_NEAR( 55.56, core.GetSpeed().Value, 0.2);
   EXPECT_NEAR(  0.12, core.GetSpeed().Accuracy, 0.1);
   EXPECT_NEAR(  0.00, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR(  0.00, core.GyroCalibratedTo(), 0.01);

   //1X
   core.AddOdo (31.10, PE::SBasicSensor(1000,1));
   core.AddGyro(31.33, PE::SBasicSensor( 100,1));
   core.AddGyro(34.66, PE::SBasicSensor( 100,1));
   core.AddOdo (36.10, PE::SBasicSensor(1000,1));
   //#4
   core.AddGnss(37.00, PE::SPosition(49.99915785122201,10.00426136658914,0.1), PE::SBasicSensor(197.00,0.1), PE::SBasicSensor(27.78,0.1));

   //1X
   core.AddGyro(38.00, PE::SBasicSensor( 100,1));
   core.AddOdo (41.10, PE::SBasicSensor(1000,1));
   core.AddGyro(41.33, PE::SBasicSensor( 100,1));
   core.AddGyro(44.66, PE::SBasicSensor( 100,1));
   //#5
   core.AddGnss(46.00, PE::SPosition(49.99746482882622,10.00210425432427,0.1), PE::SBasicSensor(242.00,0.1), PE::SBasicSensor(27.78,0.1));

   //1X
   core.AddOdo (46.10, PE::SBasicSensor(1000,1));
   core.AddGyro(48.00, PE::SBasicSensor( 100,1));
   core.AddOdo (51.10, PE::SBasicSensor(1000,1));
   core.AddGyro(51.33, PE::SBasicSensor( 100,1));
   core.AddGyro(54.66, PE::SBasicSensor( 100,1));
   //#6
   core.AddGnss(55.00, PE::SPosition(49.99724808447059,9.998716659729791,0.1), PE::SBasicSensor(287.00,0.1), PE::SBasicSensor(27.78,0.1));

   //Circle #1
   //1X
   core.AddOdo (56.10, PE::SBasicSensor(1000,1));
   core.AddGyro(58.00, PE::SBasicSensor( 100,1));
   core.AddOdo (61.10, PE::SBasicSensor(1000,1));
   core.AddGyro(61.33, PE::SBasicSensor( 100,1));
   //#0
   core.AddGnss(64.00, PE::SPosition(49.99863456730125,9.996082730449409,0.1), PE::SBasicSensor(332.00, 0.1), PE::SBasicSensor(27.78,0.1));

   //2X
   core.AddGyro(64.66, PE::SBasicSensor( 200,1));
   core.AddOdo (66.10, PE::SBasicSensor(2000,1));
   core.AddGyro(68.00, PE::SBasicSensor( 200,1));
   core.AddOdo (71.10, PE::SBasicSensor(2000,1));
   core.AddGyro(71.33, PE::SBasicSensor( 200,1));
   //#2
   core.AddGnss(73.00, PE::SPosition(50.00250531810699,9.997902372885799,0.1), PE::SBasicSensor(62.00,0.1), PE::SBasicSensor(55.56,0.1));

   //1X
   core.AddGyro(74.66, PE::SBasicSensor( 100,1));
   core.AddOdo (76.10, PE::SBasicSensor(1000,1));
   core.AddGyro(78.00, PE::SBasicSensor( 100,1));
   core.AddOdo (81.10, PE::SBasicSensor(1000,1));
   core.AddGyro(81.33, PE::SBasicSensor( 100,1));
   //#7
   core.AddGnss(82.00, PE::SPosition(50.00272208616509,10.00129033426604,0.1), PE::SBasicSensor(107.00,0.1), PE::SBasicSensor(27.78,0.1));

   //2X
   core.AddGyro(84.66, PE::SBasicSensor( 200,1));
   core.AddOdo (86.10, PE::SBasicSensor(2000,1));
   core.AddGyro(88.00, PE::SBasicSensor( 200,1));
   //#4
   core.AddGnss(91.00, PE::SPosition(49.99915785122201,10.00426136658914,0.1), PE::SBasicSensor(197.00,0.1), PE::SBasicSensor(27.78,0.1));

   //1X
   core.AddOdo (91.10, PE::SBasicSensor(1000,1));
   core.AddGyro(91.33, PE::SBasicSensor( 100,1));
   core.AddGyro(94.66, PE::SBasicSensor( 100,1));
   core.AddOdo (96.10, PE::SBasicSensor(1000,1));
   core.AddGyro(98.00, PE::SBasicSensor( 100,1));
   //#5
   core.AddGnss(100.00, PE::SPosition(49.99746482882622,10.00210425432427,0.1), PE::SBasicSensor(242.00,0.1), PE::SBasicSensor(27.78,0.1));
   EXPECT_NEAR( 49.997464, core.GetPosition().Latitude, 0.000011);
   EXPECT_NEAR( 10.002104, core.GetPosition().Longitude, 0.00001);
   EXPECT_NEAR(  0.10, core.GetPosition().HorizontalAcc, 0.01);
   EXPECT_NEAR(242.00, core.GetHeading().Value, 0.1);
   EXPECT_NEAR(  0.12, core.GetHeading().Accuracy, 0.1);
   EXPECT_NEAR( 27.78, core.GetSpeed().Value, 0.1);
   EXPECT_NEAR(  0.12, core.GetSpeed().Accuracy, 0.1);
   EXPECT_NEAR(  0.00, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR(  0.00, core.GyroCalibratedTo(), 0.01);


   
   //    core.AddOdo (101.10, PE::SBasicSensor(1000,1));
   //    core.AddOdo (106.10, PE::SBasicSensor(1000,1));
   //    core.AddOdo (111.10, PE::SBasicSensor(1000,1));
   //    core.AddOdo (116.10, PE::SBasicSensor(1000,1));
// 
   //    core.AddGyro(101.33, PE::SBasicSensor( 100,1));
   //    core.AddGyro(104.66, PE::SBasicSensor( 100,1));
   //    core.AddGyro(108.00, PE::SBasicSensor( 100,1));
   //    core.AddGyro(111.33, PE::SBasicSensor( 100,1));
   //    core.AddGyro(114.66, PE::SBasicSensor( 100,1));
   //    core.AddGyro(118.00, PE::SBasicSensor( 100,1));

//    printf("X1_0 == ts:%.2f lat:%.5f lon:%.5f head:%.2f speed:%.2f odo->scale:%.2f%% base:%.2f%% gyro->scale:%.2f%% base:%.2f%%\n",
//       core.GetTimestamp(),
//       core.GetPosition().Latitude,
//       core.GetPosition().Longitude,
//       core.GetHeading().Value,
//       core.GetSpeed().Value,
//       core.GetOdoScaleCalib(),
//       core.GetOdoBaseCalib(),
//       core.GetGyroScaleCalib(),
//       core.GetGyroBaseCalib());

//    core.AddGnss(1.000, PE::SPosition(49.99863456730125,9.996082730449409,1), PE::SBasicSensor(), PE::SBasicSensor());
//    core.AddGnss(10.000, PE::SPosition(50.00081217782849,9.995745334369854,1), PE::SBasicSensor(), PE::SBasicSensor());
//    core.AddGnss(19.000, PE::SPosition(50.00250531810699,9.997902372885799,1), PE::SBasicSensor(), PE::SBasicSensor());
//    core.AddGnss(28.000, PE::SPosition(50.00272208616509,10.00129033426604,1), PE::SBasicSensor(), PE::SBasicSensor());
//    core.AddGnss(37.000, PE::SPosition(50.00133548545172,10.00392433729531,1), PE::SBasicSensor(), PE::SBasicSensor());
//    core.AddGnss(46.000, PE::SPosition(49.99915785122201,10.00426136658914,1), PE::SBasicSensor(), PE::SBasicSensor());
//    core.AddGnss(55.000, PE::SPosition(49.99746482882622,10.00210425432427,1), PE::SBasicSensor(), PE::SBasicSensor());
//    core.AddGnss(64.000, PE::SPosition(49.99724808447059,9.998716659729791,1), PE::SBasicSensor(), PE::SBasicSensor());
//   EXPECT_TRUE(core.AddGnss(73.000, PE::SPosition(49.99863456730125,9.996082730449409,1), PE::SBasicSensor(), PE::SBasicSensor()));
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
