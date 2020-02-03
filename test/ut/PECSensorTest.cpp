/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2020 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */


/**
 * Unit test of the PE::CSensor class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "PECSensor.h"
#include "PETools.h"

class PECSensorTest : public ::testing::Test
{
public:
   virtual void SetUp() {
   }
   virtual void TearDown() {
   }
};


class PECSensorStub : public PE::ISensorAdjuster
{
private:
   PE::TValue m_refV;
   PE::TValue m_senV;
public:
   PECSensorStub()
      : m_refV(std::numeric_limits<PE::TValue>::quiet_NaN())
      , m_senV(std::numeric_limits<PE::TValue>::quiet_NaN())
      {
      }
   bool SetRefValue( const PE::TTimestamp& oldRefTimestamp, 
                     const PE::TTimestamp& newRefTimestamp, 
                     const PE::TValue& refValue, 
                     const PE::TAccuracy& refAccuracy)
      {
         if ( 0.0 == refAccuracy )
         {
            m_refV = refValue;
            return true;
         }
         else
         {
            m_refV = std::numeric_limits<PE::TValue>::quiet_NaN();
            return false;
         }
      }
   const PE::TValue& GetRefValue() const
      {
         return m_refV;
      }
   bool SetSenValue( const PE::TTimestamp& oldRefTimestamp, 
                     const PE::TTimestamp& oldSenTimestamp, 
                     const PE::TTimestamp& newSenTimestamp, 
                     const PE::TValue& senValue, 
                     bool IsValid)
      {
         if ( IsValid )
         {
            m_senV = senValue;
            return true;
         }
         else
         {
            m_senV = std::numeric_limits<PE::TValue>::quiet_NaN();
            return false;
         }
      }
   const PE::TValue& GetSenValue() const
      {
         return m_senV;
      }
};

/**
 * checks initialization staff
 */
TEST_F(PECSensorTest, test_init)
{
   PECSensorStub adjuster_stub;
   PE::CSensor sensor(adjuster_stub);

   EXPECT_EQ( 0.0, sensor.GetRefTimeStamp() );
   EXPECT_EQ( 0.0, sensor.GetSenTimeStamp() );
   EXPECT_EQ( 0.0, sensor.GetBias().GetReliable());
   EXPECT_EQ( 0.0, sensor.GetScale().GetReliable());
}


/**
 * checks all valid values
 */
TEST_F(PECSensorTest, test_all_values_are_valid_check)
{
   PECSensorStub adjuster_stub;
   PE::CSensor sensor(adjuster_stub);

   sensor.AddRef(1.000, 100,  0.0);
   sensor.AddSen(1.001,   5+2, true);
   sensor.AddRef(1.002, 200,  0.0);
   sensor.AddSen(1.003,  10+2, true);
   sensor.AddRef(1.004, 300,  0.0);
   sensor.AddSen(1.005,  15+2, true);
   sensor.AddRef(1.006, 100,  0.0);
   sensor.AddSen(1.007,   5+2, true);
   sensor.AddRef(1.008, 200,  0.0);
   sensor.AddSen(1.009,  10+2, true);

   EXPECT_NEAR  ( 1.008, sensor.GetRefTimeStamp(), 0.0001 );
   EXPECT_NEAR  ( 1.009, sensor.GetSenTimeStamp(), 0.0001 );
   EXPECT_NEAR  ( 50.00, sensor.GetBias().GetReliable(), 0.01 );
   EXPECT_NEAR  (  2.00, sensor.GetBias().GetMean(), 0.01 );
   EXPECT_NEAR  ( 50.00, sensor.GetScale().GetReliable(), 0.01 );
   EXPECT_NEAR  ( 20.00, sensor.GetScale().GetMean(), 0.01 );
}


/**
 * checks adding first reference and sensor values
 */
TEST_F(PECSensorTest, test_adding_first_ref_and_sen_values)
{
   PECSensorStub adjuster_stub;
   PE::CSensor sensor(adjuster_stub);

   EXPECT_FALSE(sensor.AddSen(1.001, 0, true));
   EXPECT_FALSE(sensor.AddSen(1.002, 0, true));
   EXPECT_FALSE(sensor.AddSen(1.003, 0, true));
   EXPECT_EQ(0.0, sensor.GetSenTimeStamp());
   EXPECT_FALSE(sensor.AddRef(1.111, 0,  0.0));
   EXPECT_EQ(0.000, sensor.GetSenTimeStamp());
   EXPECT_EQ(1.111, sensor.GetRefTimeStamp());
   EXPECT_FALSE(sensor.AddSen(1.234, 0, true));
   EXPECT_EQ(1.234, sensor.GetSenTimeStamp());
   EXPECT_EQ(1.111, sensor.GetRefTimeStamp());
   EXPECT_TRUE (sensor.AddSen(2.000, 0, true));
   EXPECT_TRUE (sensor.AddRef(2.222, 0,  0.0));
   EXPECT_EQ(2.000, sensor.GetSenTimeStamp());
   EXPECT_EQ(2.222, sensor.GetRefTimeStamp());
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
