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
  * Unit test for the class CSensorEntity.
  *
  * Code under test:
  *
  */
 
#include <gtest/gtest.h>
#include "PECSensorEntity.h"


class PECSensorEntityTest : public ::testing::Test
{
public:
   virtual void SetUp()
   {}
   virtual void TearDown()
   {}

   PE::TValue GetScaleCalibStatus(const PE::CSensorEntity& entity)
   {
      return entity.mScaleCalib.CalibratedTo();
   }

   PE::TValue GetBaseCalibStatus(const PE::CSensorEntity& entity)
   {
      return entity.mBaseCalib.CalibratedTo();
   }
};


/**
 * test
 */
TEST_F(PECSensorEntityTest, create_test )
{
   PE::CSensorEntity entity(PE::CSensorCfg(PE::SENSOR_GYRO_Z, // Gyroscope Z-axe
                                           PE::CNormCfg(123.456,12.3456,1.23456,6.0), // scale
                                           PE::CNormCfg(654.321,65.4321,6.54321,12.0), // base
                                           10.0 // 10.0% calibration limit
                                           ),
                            3, // Ratio 3 sensors per 1 reference
                            1 // Threshold +/- 1
                           );
   EXPECT_EQ(3.0, entity.GetRatio());
   EXPECT_EQ(1.0, entity.GetThreshold());

   //build configuration
   PE::CSensorCfg cfg(PE::SENSOR_GYRO_Z,
                      PE::CSensorCfg::ToNormCfg(entity.GetScale()),
                      PE::CSensorCfg::ToNormCfg(entity.GetBase()),
                      entity.GetLimit());

   EXPECT_EQ("CFGSENSOR,6,123.45600000,12.34560000,1.2,6,654.32100000,65.43210000,6.5,12,10.0,XX",PE::CSensorCfg::ToSTR(cfg));
}


/**
 * test
 */
TEST_F(PECSensorEntityTest, simple_test )
{
   PE::CSensorEntity entity(PE::CSensorCfg(PE::SENSOR_GYRO_Z, // Gyroscope Z-axe
                                           PE::CNormCfg(), // Empty scale
                                           PE::CNormCfg(), // Empty base
                                           10.0 // 10.0% calibration limit
                                           ),
                            3, // Ratio 3 sensors per 1 reference
                            1 // Threshold +/- 1
                           );

   EXPECT_NEAR(0.00, GetScaleCalibStatus(entity), 0.01);
   EXPECT_NEAR(0.00, GetBaseCalibStatus(entity), 0.01);
   EXPECT_NEAR(0.00, entity.CalibratedTo(), 0.01);

   entity.AddSensor(PE::SBasicSensor(1000+100,0.1));
   entity.AddSensor(PE::SBasicSensor(1000+100,0.1));
   entity.AddSensor(PE::SBasicSensor(1000+100,0.1));
   entity.AddReference(PE::SBasicSensor(10,0.01));
   EXPECT_NEAR(0.00, GetScaleCalibStatus(entity), 0.01);
   EXPECT_NEAR(0.00, GetBaseCalibStatus(entity), 0.01);
   EXPECT_NEAR(0.00, entity.CalibratedTo(), 0.01);

   entity.AddSensor(PE::SBasicSensor(2000+100,0.2));
   entity.AddSensor(PE::SBasicSensor(2000+100,0.2));
   entity.AddSensor(PE::SBasicSensor(2000+100,0.2));
   entity.AddReference(PE::SBasicSensor(20,0.02));
   EXPECT_NEAR(0.00, GetScaleCalibStatus(entity), 0.01);
   EXPECT_NEAR(0.00, GetBaseCalibStatus(entity), 0.01);
   EXPECT_NEAR(0.00, entity.CalibratedTo(), 0.01);

   entity.AddSensor(PE::SBasicSensor(1000+100,0.1));
   entity.AddSensor(PE::SBasicSensor(1000+100,0.1));
   entity.AddSensor(PE::SBasicSensor(1000+100,0.1));
   entity.AddReference(PE::SBasicSensor(10,0.01));
   EXPECT_NEAR(50.00, GetScaleCalibStatus(entity), 0.01);
   EXPECT_NEAR(0.00, GetBaseCalibStatus(entity), 0.01);
   EXPECT_NEAR(0.00, entity.CalibratedTo(), 0.01);

   entity.AddSensor(PE::SBasicSensor(100+100,0.3));
   entity.AddSensor(PE::SBasicSensor(100+100,0.3));
   entity.AddSensor(PE::SBasicSensor(100+100,0.3));
   entity.AddReference(PE::SBasicSensor(1,0.03));
   EXPECT_NEAR(66.67, GetScaleCalibStatus(entity), 0.01);
   EXPECT_NEAR(0.00, GetBaseCalibStatus(entity), 0.01);
   EXPECT_NEAR(0.00, entity.CalibratedTo(), 0.01);

   entity.AddSensor(PE::SBasicSensor(200+100,0.1));
   entity.AddSensor(PE::SBasicSensor(200+100,0.1));
   entity.AddSensor(PE::SBasicSensor(200+100,0.1));
   entity.AddReference(PE::SBasicSensor(2,0.01));
   EXPECT_NEAR(75.00, GetScaleCalibStatus(entity), 0.01);
   EXPECT_NEAR(50.00, GetBaseCalibStatus(entity), 0.01);
   EXPECT_NEAR(50.00, entity.CalibratedTo(), 0.01);

   entity.AddSensor(PE::SBasicSensor(300+100,0.1));
   entity.AddSensor(PE::SBasicSensor(300+100,0.1));
   entity.AddSensor(PE::SBasicSensor(300+100,0.1));
   entity.AddReference(PE::SBasicSensor(3,0.01));
   EXPECT_NEAR(80.00, GetScaleCalibStatus(entity), 0.01);
   EXPECT_NEAR(66.67, GetBaseCalibStatus(entity), 0.01);
   EXPECT_NEAR(66.67, entity.CalibratedTo(), 0.01);
   //Calibration is more then 0.00% check value
   EXPECT_NEAR(10.00, entity.GetSensor(PE::SBasicSensor(1000+100,0.1)).Value, 0.01);
   EXPECT_NEAR(0.00, entity.GetSensor(PE::SBasicSensor(1000+100,0.1)).Accuracy, 0.01);

   EXPECT_NEAR(20.00, entity.GetSensor(PE::SBasicSensor(2000+100,0.1)).Value, 0.01);
   EXPECT_NEAR(0.00, entity.GetSensor(PE::SBasicSensor(2000+100,0.1)).Accuracy, 0.01);

   EXPECT_NEAR(1.00, entity.GetSensor(PE::SBasicSensor(100+100,0.1)).Value, 0.01);
   EXPECT_NEAR(0.00, entity.GetSensor(PE::SBasicSensor(100+100,0.1)).Accuracy, 0.01);

   EXPECT_NEAR(-1.00, entity.GetSensor(PE::SBasicSensor(0,0.1)).Value, 0.01);
   EXPECT_NEAR(0.00, entity.GetSensor(PE::SBasicSensor(0,0.1)).Accuracy, 0.01);
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
