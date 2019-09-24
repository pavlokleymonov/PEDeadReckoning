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
 * Unit test of the PECCalibrationSummary class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PECCalibrationSummary.h"


class PECCalibrationSummaryTest : public ::testing::Test
{
public:
   virtual void SetUp()
   {}
   virtual void TearDown()
   {}
};


/**
 * tests creation 
 */
TEST_F(PECCalibrationSummaryTest, create_destroy_test)
{
   PE::ICalibration* p_calib = new PE::CCalibrationSummary();

   EXPECT_TRUE( PE::isnan( p_calib->GetBias() ) );
   EXPECT_TRUE( PE::isnan( p_calib->GetScale() ) );

   delete p_calib;
}


/**
 * tests first iteration
 */
TEST_F(PECCalibrationSummaryTest, first_iteration_test)
{
   PE::ICalibration* p_calib = new PE::CCalibrationSummary();

   EXPECT_TRUE( PE::isnan( p_calib->GetBias() ) );
   EXPECT_TRUE( PE::isnan( p_calib->GetScale() ) );

   p_calib->AddRaw(11);
   p_calib->AddRef(1);

   EXPECT_TRUE( PE::isnan( p_calib->GetBias() ) );
   EXPECT_TRUE( PE::isnan( p_calib->GetScale() ) );

   p_calib->Recalculate();

   EXPECT_TRUE( PE::isnan( p_calib->GetBias() ) );
   EXPECT_TRUE( PE::isnan( p_calib->GetScale() ) );

   delete p_calib;
}


/**
 * tests  n; n+1; base=1 and scale=0.1
 */
TEST_F(PECCalibrationSummaryTest, n_and_n_plus_1_base_1_scale_0_dot_1_test)
{
   PE::ICalibration* p_calib = new PE::CCalibrationSummary();

   p_calib->AddRaw(11);
   p_calib->AddRef(1);
   p_calib->Recalculate();

   EXPECT_TRUE( PE::isnan( p_calib->GetBias() ) );
   EXPECT_TRUE( PE::isnan( p_calib->GetScale() ) );

   p_calib->AddRaw(12);
   p_calib->AddRef(1.1);
   p_calib->Recalculate();

   EXPECT_NEAR( 1.0, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR( 0.1, p_calib->GetScale(), PE::EPSILON);

   p_calib->AddRaw(13);
   p_calib->AddRef(1.2);
   p_calib->Recalculate();

   EXPECT_NEAR( 1.0, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR( 0.1, p_calib->GetScale(), PE::EPSILON);

   delete p_calib;
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
