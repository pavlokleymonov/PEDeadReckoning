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

int getIdx(int last, int max, bool is1X)
{
   last += (is1X ? 1 : 2);
   return last % max;
}

PE::TTimestamp AddSensors(int& gnssIdx, PE::TTimestamp beginTs, PE::TTimestamp endTs, PE::CCoreSimple& core, PE::SPosition& last_gnss_pos)
{
   PE::SBasicSensor s1x(27.78,0.1);
   PE::SBasicSensor odo1x(1000,1);
   PE::SBasicSensor gyro1x(100,1);

   PE::SBasicSensor s2x(55.56,0.1);
   PE::SBasicSensor odo2x(2000,1);
   PE::SBasicSensor gyro2x(200,1);

   std::vector<PE::SPosition> pos;
   pos.push_back(PE::SPosition(49.99863456730125,9.996082730449409,1));
   pos.push_back(PE::SPosition(50.00081217782849,9.995745334369854,1));
   pos.push_back(PE::SPosition(50.00250531810699,9.997902372885799,1));
   pos.push_back(PE::SPosition(50.00272208616509,10.00129033426604,1));
   pos.push_back(PE::SPosition(50.00133548545172,10.00392433729531,1));
   pos.push_back(PE::SPosition(49.99915785122201,10.00426136658914,1));
   pos.push_back(PE::SPosition(49.99746482882622,10.00210425432427,1));
   pos.push_back(PE::SPosition(49.99724808447059,9.998716659729791,1));

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
         list.insert(std::make_pair(odoTs, std::make_pair(ODO,is1x)));
         odoTs += 4.5; //4.5 seconds
      }
      while (gyroTs <= gnssTs+9.0)
      {
         list.insert(std::make_pair(gyroTs, std::make_pair(GYRO,is1x)));
         gyroTs += 3.00; //3.0 seconds
      }
      gnssTs += 9.0;
      gnssIdx = getIdx(gnssIdx, 8, is1x);
      list.insert(std::make_pair(gnssTs, std::make_pair(TSenID(gnssIdx+2),is1x)));
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
            core.AddGnss( ts, pos[ID-2], head[ID-2], is_1x ? s1x : s2x);
            last_gnss_pos = pos[ID-2];
            //core.AddGnss( ts, pos[ID-2], invalid, invalid);
            break;
      }
   }
   return gnssTs;
}


/**
 * test circle driving with the speed~98[km/h] distance 2[km]
 */
TEST_F(PECCoreSimpleTest, ideal_circle_driving_test )
{
   PE::CCoreSimple core = PE::CCoreSimple(PE::SPosition(), PE::SBasicSensor());

   PE::CSensorCfg cfgOdo =  PE::CSensorCfg::ToCFG("CFGSENSOR,5,0.0,0.0,0.0,0,0.0,0.0,0.0,0,10.1,XX");
   EXPECT_TRUE(cfgOdo.IsValid());
   EXPECT_EQ(PE::SENSOR_ODOMETER_AXIS, cfgOdo.GetType());
   EXPECT_EQ(10.1, cfgOdo.GetLimit());
   core.SetOdoCfg(cfgOdo, 2, 1); //SENSOR_ODOMETER_AXIS = 5, 2 times per one reference (4.5 sec)

   PE::CSensorCfg cfgGyro =  PE::CSensorCfg::ToCFG("CFGSENSOR,6,0.0,0.0,0.0,0,0.0,0.0,0.0,0,10.2,XX");
   EXPECT_TRUE(cfgGyro.IsValid());
   EXPECT_EQ(PE::SENSOR_GYRO_Z, cfgGyro.GetType());
   EXPECT_EQ(10.2, cfgGyro.GetLimit());
   core.SetGyroCfg(cfgGyro, 3, 1); //SENSOR_GYRO_Z = 6,        3 times per one reference (3.00 sec)

   int gnssIdx = 0;
   PE::TTimestamp beginTs = 1;
   PE::SPosition last_gnss_pos;
   beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core,last_gnss_pos);
   EXPECT_NEAR( 59.19, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR( 51.68, core.GyroCalibratedTo(), 0.01);
//    EXPECT_NEAR( beginTs, core.GetTimestamp(), 0.01);
//    EXPECT_TRUE( last_gnss_pos.IsValid() );
//    EXPECT_NEAR( last_gnss_pos.Latitude,  core.GetPosition().Latitude,  0.000001);
//    EXPECT_NEAR( last_gnss_pos.Longitude, core.GetPosition().Longitude, 0.000001);
//    EXPECT_NEAR( 1.00, last_gnss_pos.HorizontalAcc, 0.01);
//    EXPECT_NEAR( 1.00, core.GetPosition().HorizontalAcc, 0.01);


   beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core,last_gnss_pos);
   EXPECT_NEAR( 79.40, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR( 82.77, core.GyroCalibratedTo(), 0.01);

   beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core,last_gnss_pos);
   EXPECT_NEAR( 86.00, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR( 89.63, core.GyroCalibratedTo(), 0.01);

   beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core,last_gnss_pos);
   EXPECT_NEAR( 89.33, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR( 92.56, core.GyroCalibratedTo(), 0.01);

   beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core,last_gnss_pos);
   EXPECT_NEAR( 91.35, core.OdoCalibratedTo(), 0.01);
   EXPECT_NEAR( 94.19, core.GyroCalibratedTo(), 0.01);

   EXPECT_NEAR( beginTs, core.GetTimestamp(), 0.01);
   EXPECT_TRUE( last_gnss_pos.IsValid() );
   EXPECT_NEAR( last_gnss_pos.Latitude,  core.GetPosition().Latitude,  0.00001);
   EXPECT_NEAR( last_gnss_pos.Longitude, core.GetPosition().Longitude, 0.00001);
   EXPECT_NEAR( 1.00, last_gnss_pos.HorizontalAcc, 0.01);
   EXPECT_NEAR( 1.08, core.GetPosition().HorizontalAcc, 0.01);

//    beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core);
//    printf("Odo=%.2f%% Gyro=%.2f%%\n",core.OdoCalibratedTo(),core.GyroCalibratedTo());
//    EXPECT_NEAR(  0.00, core.OdoCalibratedTo(), 0.01);
//    EXPECT_NEAR(  0.00, core.GyroCalibratedTo(), 0.01);
// 
//    beginTs = AddSensors(gnssIdx, beginTs, beginTs+100,core);
//    printf("Odo=%.2f%% Gyro=%.2f%%\n",core.OdoCalibratedTo(),core.GyroCalibratedTo());
//    EXPECT_NEAR(  0.00, core.OdoCalibratedTo(), 0.01);
//    EXPECT_NEAR(  0.00, core.GyroCalibratedTo(), 0.01);


//    EXPECT_NEAR( 49.997464, core.GetPosition().Latitude, 0.000011);
//    EXPECT_NEAR( 10.002104, core.GetPosition().Longitude, 0.00001);
//    EXPECT_NEAR(  0.10, core.GetPosition().HorizontalAcc, 0.01);
//    EXPECT_NEAR(242.00, core.GetHeading().Value, 0.1);
//    EXPECT_NEAR(  0.12, core.GetHeading().Accuracy, 0.1);
//    EXPECT_NEAR( 27.78, core.GetSpeed().Value, 0.1);
//    EXPECT_NEAR(  0.12, core.GetSpeed().Accuracy, 0.1);
//    EXPECT_NEAR(  0.00, core.OdoCalibratedTo(), 0.01);
//    EXPECT_NEAR(  0.00, core.GyroCalibratedTo(), 0.01);



//    core.AddGnss( 1.000, PE::SPosition(49.99863456730125,9.996082730449409,1), PE::SBasicSensor(), PE::SBasicSensor());
//    core.AddGnss(10.000, PE::SPosition(50.00081217782849,9.995745334369854,1), PE::SBasicSensor(), PE::SBasicSensor());
//    core.AddGnss(19.000, PE::SPosition(50.00250531810699,9.997902372885799,1), PE::SBasicSensor(), PE::SBasicSensor());
//    core.AddGnss(28.000, PE::SPosition(50.00272208616509,10.00129033426604,1), PE::SBasicSensor(), PE::SBasicSensor());
//    core.AddGnss(37.000, PE::SPosition(50.00133548545172,10.00392433729531,1), PE::SBasicSensor(), PE::SBasicSensor());
//    core.AddGnss(46.000, PE::SPosition(49.99915785122201,10.00426136658914,1), PE::SBasicSensor(), PE::SBasicSensor());
//    core.AddGnss(55.000, PE::SPosition(49.99746482882622,10.00210425432427,1), PE::SBasicSensor(), PE::SBasicSensor());
//    core.AddGnss(64.000, PE::SPosition(49.99724808447059,9.998716659729791,1), PE::SBasicSensor(), PE::SBasicSensor());
//    core.AddGnss(73.000, PE::SPosition(49.99863456730125,9.996082730449409,1), PE::SBasicSensor(), PE::SBasicSensor());
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
