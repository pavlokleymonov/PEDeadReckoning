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


typedef enum {
   ODO     =  0,
   GYRO    =  1,
   GNSS_P0 =  2,
   GNSS_P1 =  3,
   GNSS_P2 =  4,
   GNSS_P3 =  5,
   GNSS_P4 =  6,
   GNSS_P5 =  7,
   GNSS_P6 =  8,
   GNSS_P7 =  9
} TSenID;

typedef enum {
   GNSS_ON  = 0,
   GNSS_OFF = 1,
   ODO_ON   = 2,
   ODO_OFF  = 3,
   GYRO_ON  = 4,
   GYRO_OFF = 5
} TSenAddFlag;

int getIdx(int last, int max, bool is1X)
{
   last += (is1X ? 1 : 2);
   return last % max;
}

PE::TTimestamp AddSensors(int& gnssIdx, PE::TTimestamp beginTs, PE::TTimestamp endTs, PE::CCoreSimple& core, PE::SPosition& last_gnss_pos, TSenAddFlag addGnss, TSenAddFlag addOdo, TSenAddFlag addGyro)
{
   PE::SBasicSensor s1x(27.78,1.0);
   PE::SBasicSensor odo1x(1000,1);
   PE::SBasicSensor gyro1x(-100,1);

   PE::SBasicSensor s2x(55.56,1.0);
   PE::SBasicSensor odo2x(2000,1);
   PE::SBasicSensor gyro2x(-200,1);

   std::vector<PE::SPosition> pos;
   pos.push_back(PE::SPosition(49.99863456730125,9.996082730449409,0.1));
   pos.push_back(PE::SPosition(50.00081217782849,9.995745334369854,0.1));
   pos.push_back(PE::SPosition(50.00250531810699,9.997902372885799,0.1));
   pos.push_back(PE::SPosition(50.00272208616509,10.00129033426604,0.1));
   pos.push_back(PE::SPosition(50.00133548545172,10.00392433729531,0.1));
   pos.push_back(PE::SPosition(49.99915785122201,10.00426136658914,0.1));
   pos.push_back(PE::SPosition(49.99746482882622,10.00210425432427,0.1));
   pos.push_back(PE::SPosition(49.99724808447059,9.998716659729791,0.1));

   std::vector<PE::SBasicSensor> head;
   head.push_back( PE::SBasicSensor(332.00, 0.1));
   head.push_back( PE::SBasicSensor( 17.00, 0.1));
   head.push_back( PE::SBasicSensor( 62.00, 0.1));
   head.push_back( PE::SBasicSensor(107.00, 0.1));
   head.push_back( PE::SBasicSensor(152.00, 0.1));
   head.push_back( PE::SBasicSensor(197.00, 0.1));
   head.push_back( PE::SBasicSensor(242.00, 0.1));
   head.push_back( PE::SBasicSensor(287.00, 0.1));

   PE::TTimestamp gnssTs = beginTs;
   PE::TTimestamp  odoTs = gnssTs + 0.10;
   PE::TTimestamp gyroTs = gnssTs + 0.20;

   std::multimap<PE::TTimestamp, std::pair<TSenID,bool> > list;

   PE::SBasicSensor invalid;

   bool is1x = true;
   while (gnssTs <= endTs)
   {
      while (odoTs <= gnssTs+9.0)
      {
         if ( ODO_ON == addOdo )
         {
            list.insert(std::make_pair(odoTs, std::make_pair(ODO,is1x)));
         }
         odoTs += 4.5; //4.5 seconds
      }
      while (gyroTs <= gnssTs+9.0)
      {
         if ( GYRO_ON == addGyro )
         {
            list.insert(std::make_pair(gyroTs, std::make_pair(GYRO,is1x)));
         }
         gyroTs += 3.00; //3.0 seconds
      }
      gnssTs += 9.0;
      gnssIdx = getIdx(gnssIdx, 8, is1x);
      if ( GNSS_ON == addGnss )
      {
         list.insert(std::make_pair(gnssTs, std::make_pair(TSenID(gnssIdx+2),is1x)));
      }
      is1x ^= true;
   }
   for (std::multimap<PE::TTimestamp, std::pair<TSenID,bool> >::iterator it = list.begin(); it!=list.end(); ++it)
   {
      PE::TTimestamp ts = it->first;
      bool is_1x = it->second.second;
      TSenID ID = it->second.first;
      switch ( ID )
      {
         case ODO:
            //printf("ts=%.3f %s ODO:%.2f\n", ts, is_1x ? "1X" : "2X", is_1x ? odo1x.Value: odo2x.Value);
            core.AddOdo( ts, is_1x ? odo1x : odo2x);
            break;
         case GYRO:
            //printf("ts=%.3f %s GYRO:%.2f\n", ts, is_1x ? "1X" : "2X", is_1x ? gyro1x.Value : gyro2x.Value);
            core.AddGyro( ts, is_1x ? gyro1x : gyro2x);
            break;
         default:
            //printf("ts=%.3f %s GNSS_%u lat=%.6f lon=%.6f head=%.2f speed=%.2f\n", ts, is_1x ? "1X" : "2X", ID-2, pos[ID-2].Latitude, pos[ID-2].Longitude, head[ID-2].Value, is_1x ? s1x.Value : s2x.Value);
            core.AddGnss( ts, pos[ID-2], head[ID-2], invalid);
            //core.AddGnss( ts, pos[ID-2], head[ID-2], is_1x ? s1x : s2x);
            break;
      }
   }
   last_gnss_pos = pos[gnssIdx];
   return gnssTs;
}


/**
 * test circle driving with the speed~98[km/h] circle size 2[km]
 */
TEST_F(PECCoreSimpleTest, ideal_circle_driving_test )
{
   PE::CCoreSimple core = PE::CCoreSimple(PE::SPosition(), PE::SBasicSensor());

   PE::CSensorCfg cfgOdo =  PE::CSensorCfg::ToCFG("CFGSENSOR,5,0.0,0.0,0.0,0,0.0,0.0,0.0,0,50.0,XX");
   EXPECT_TRUE(cfgOdo.IsValid());
   EXPECT_EQ(PE::SENSOR_ODOMETER_AXIS, cfgOdo.GetType());
   EXPECT_EQ(50.0, cfgOdo.GetLimit());
   core.SetOdoCfg(cfgOdo, 2, 1); //SENSOR_ODOMETER_AXIS = 5, 2 times per one reference (4.5 sec)

   PE::CSensorCfg cfgGyro =  PE::CSensorCfg::ToCFG("CFGSENSOR,6,0.0,0.0,0.0,0,0.0,0.0,0.0,0,50.0,XX");
   EXPECT_TRUE(cfgGyro.IsValid());
   EXPECT_EQ(PE::SENSOR_GYRO_Z, cfgGyro.GetType());
   EXPECT_EQ(50.0, cfgGyro.GetLimit());
   core.SetGyroCfg(cfgGyro, 3, 1); //SENSOR_GYRO_Z = 6,        3 times per one reference (3.00 sec)

   int gnssIdx = 0;
   PE::TTimestamp beginTs = 1;
   PE::SPosition last_gnss_pos;
   beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core,last_gnss_pos, GNSS_ON, ODO_ON, GYRO_ON);
   EXPECT_NEAR( 15.80, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR(  0.00, core.GyroCalibratedTo(), 0.01);
   EXPECT_NEAR( beginTs, core.GetTimestamp(), 0.01);
   EXPECT_NEAR( last_gnss_pos.Latitude,  core.GetPosition().Latitude,  0.0000001);
   EXPECT_NEAR( last_gnss_pos.Longitude, core.GetPosition().Longitude, 0.0000001);

   beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core,last_gnss_pos, GNSS_ON, ODO_ON, GYRO_ON);
   EXPECT_NEAR( 75.24, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR( 63.89, core.GyroCalibratedTo(), 0.01);
   EXPECT_NEAR( beginTs, core.GetTimestamp(), 0.01);
   EXPECT_NEAR( last_gnss_pos.Latitude,  core.GetPosition().Latitude,  0.0000001);
   EXPECT_NEAR( last_gnss_pos.Longitude, core.GetPosition().Longitude, 0.0000001);

   beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core,last_gnss_pos, GNSS_ON, ODO_ON, GYRO_ON);
   EXPECT_NEAR( 85.73, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR( 80.49, core.GyroCalibratedTo(), 0.01);

   beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core,last_gnss_pos, GNSS_ON, ODO_ON, GYRO_ON);
   EXPECT_NEAR( 89.90, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR( 86.74, core.GyroCalibratedTo(), 0.01);

   beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core,last_gnss_pos, GNSS_ON, ODO_ON, GYRO_ON);
   EXPECT_NEAR( 92.08, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR( 89.97, core.GyroCalibratedTo(), 0.01);

   beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core,last_gnss_pos, GNSS_ON, ODO_ON, GYRO_ON);
   EXPECT_NEAR( 93.44, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR( 91.95, core.GyroCalibratedTo(), 0.01);

   beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core,last_gnss_pos, GNSS_ON, ODO_ON, GYRO_ON);
   EXPECT_NEAR( 94.37, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR( 93.27, core.GyroCalibratedTo(), 0.01);

   beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core,last_gnss_pos, GNSS_ON, ODO_ON, GYRO_ON);
   EXPECT_NEAR( 95.05, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR( 94.23, core.GyroCalibratedTo(), 0.01);

   beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core,last_gnss_pos, GNSS_ON, ODO_ON, GYRO_ON);
   EXPECT_NEAR( 95.57, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR( 94.95, core.GyroCalibratedTo(), 0.01);

   beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core,last_gnss_pos, GNSS_ON, ODO_ON, GYRO_ON);
   EXPECT_NEAR( 95.99, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR( 95.51, core.GyroCalibratedTo(), 0.01);
   EXPECT_NEAR( beginTs, core.GetTimestamp(), 0.01);
   EXPECT_NEAR( last_gnss_pos.Latitude,  core.GetPosition().Latitude,  0.0000001);
   EXPECT_NEAR( last_gnss_pos.Longitude, core.GetPosition().Longitude, 0.0000001);
}


/**
 * test circle driving with the speed~98[km/h] circle size 2[km]
 */
TEST_F(PECCoreSimpleTest, ideal_circle_driving_deadreckoning_test )
{
   PE::CCoreSimple core = PE::CCoreSimple(PE::SPosition(), PE::SBasicSensor());

   PE::CSensorCfg cfgOdo =  PE::CSensorCfg::ToCFG("CFGSENSOR,5,0.0,0.0,0.0,0,0.0,0.0,0.0,0,90.0,XX");
   core.SetOdoCfg(cfgOdo, 2, 1); //SENSOR_ODOMETER_AXIS = 5, 2 times per one reference (4.5 sec)

   PE::CSensorCfg cfgGyro =  PE::CSensorCfg::ToCFG("CFGSENSOR,6,0.0,0.0,0.0,0,0.0,0.0,0.0,0,90.0,XX");
   core.SetGyroCfg(cfgGyro, 3, 1); //SENSOR_GYRO_Z = 6,        3 times per one reference (3.00 sec)

   int gnssIdx = 0;
   PE::TTimestamp beginTs = 1;
   PE::SPosition last_gnss_pos;
   //Calibartion 0%
   beginTs = AddSensors(gnssIdx, beginTs, beginTs+300,core,last_gnss_pos, GNSS_ON, ODO_ON, GYRO_ON);
   EXPECT_NEAR(  0.00, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR(  0.00, core.GyroCalibratedTo(), 0.01);

   //Calibartion ~75%
   beginTs = AddSensors(gnssIdx, beginTs, beginTs+300,core,last_gnss_pos, GNSS_ON, ODO_ON, GYRO_ON);
   EXPECT_NEAR( 78.55, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR( 43.62, core.GyroCalibratedTo(), 0.01);

   //Calibartion ~90%
   beginTs = AddSensors(gnssIdx, beginTs, beginTs+300,core,last_gnss_pos, GNSS_ON, ODO_ON, GYRO_ON);
//    EXPECT_NEAR( 919.00, beginTs, 0.01);
//    EXPECT_NEAR( 919.00, core.GetTimestamp(), 0.01);
//    EXPECT_NEAR(50.0008121, last_gnss_pos.Latitude,       0.0000001);
//    EXPECT_NEAR( 9.9957453, last_gnss_pos.Longitude,      0.0000001);
//    EXPECT_NEAR(50.0008121, core.GetPosition().Latitude , 0.0000001);
//    EXPECT_NEAR( 9.9957453, core.GetPosition().Longitude, 0.0000001);
//    EXPECT_NEAR(      0.10, core.GetPosition().HorizontalAcc, 0.01);
   //Last valid calibartion status
//    EXPECT_NEAR( 90.48, core.OdoCalibratedTo(), 0.01);
//    EXPECT_NEAR( 91.88, core.GyroCalibratedTo(), 0.01);


   //DeadReckoning mode
   beginTs = AddSensors(gnssIdx, beginTs, beginTs+5,core,last_gnss_pos, GNSS_OFF, ODO_ON, GYRO_ON);
   //update sensor only position
   core.UpdatePosition();
//    EXPECT_NEAR( 946.00, beginTs, 0.01);
//    EXPECT_NEAR( 943.20, core.GetTimestamp(), 0.01);
//    EXPECT_NEAR(49.9991578, last_gnss_pos.Latitude,       0.0000001);
//    EXPECT_NEAR(10.0042613, last_gnss_pos.Longitude,      0.0000001);
//    EXPECT_EQ(-1, gnssIdx);
//    EXPECT_NEAR(00.0000000, core.GetPosition().Latitude , 0.0000001);
//    EXPECT_NEAR(00.0000000, core.GetPosition().Longitude, 0.0000001);
//    EXPECT_NEAR(00.0000000, core.GetSpeed().Value, 0.0000001);
//    EXPECT_NEAR(00.0000000, core.GetHeading().Value, 0.0000001);
   //Calibration was not improved - OK
//    EXPECT_NEAR( 90.48, core.OdoCalibratedTo(), 0.01);
//    EXPECT_NEAR( 91.88, core.GyroCalibratedTo(), 0.01);

   beginTs = AddSensors(gnssIdx, beginTs, beginTs+5,core,last_gnss_pos, GNSS_OFF, ODO_ON, GYRO_ON);
   //update sensor only position
   core.UpdatePosition();
//    EXPECT_NEAR( 946.00, beginTs, 0.01);
//    EXPECT_NEAR( 943.20, core.GetTimestamp(), 0.01);
//    EXPECT_NEAR(49.9991578, last_gnss_pos.Latitude,       0.0000001);
//    EXPECT_NEAR(10.0042613, last_gnss_pos.Longitude,      0.0000001);
//    EXPECT_EQ(-1, gnssIdx);
//    EXPECT_NEAR(00.0000000, core.GetPosition().Latitude , 0.0000001);
//    EXPECT_NEAR(00.0000000, core.GetPosition().Longitude, 0.0000001);
//    EXPECT_NEAR(00.0000000, core.GetSpeed().Value, 0.0000001);
//    EXPECT_NEAR(00.0000000, core.GetHeading().Value, 0.0000001);
   //Calibration was not improved - OK
//    EXPECT_NEAR( 90.48, core.OdoCalibratedTo(), 0.01);
//    EXPECT_NEAR( 91.88, core.GyroCalibratedTo(), 0.01);
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
