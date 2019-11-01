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
 * tests same value at start-up
 */
TEST_F(PECCalibrationSummaryTest, same_value_at_start_up_test)
{
   PE::ICalibration* p_calib = new PE::CCalibrationSummary();

   p_calib->AddRaw(11);
   p_calib->AddRef(1);
   p_calib->Recalculate();
   EXPECT_TRUE( PE::isnan( p_calib->GetBias() ) );
   EXPECT_TRUE( PE::isnan( p_calib->GetScale() ) );

   p_calib->AddRaw(11);
   p_calib->AddRef(1);
   p_calib->Recalculate();
   EXPECT_TRUE( PE::isnan( p_calib->GetBias() ) );
   EXPECT_TRUE( PE::isnan( p_calib->GetScale() ) );

   p_calib->AddRaw(11);
   p_calib->AddRef(1);
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

   //RAW=11 REF=1.0
   p_calib->AddRaw(11);
   p_calib->AddRef(1);
   p_calib->Recalculate();
   EXPECT_TRUE( PE::isnan( p_calib->GetBias() ) );
   EXPECT_TRUE( PE::isnan( p_calib->GetScale() ) );

   //RAW=12 REF=1.1
   p_calib->AddRaw(12);
   p_calib->AddRef(1.1);
   p_calib->Recalculate();
   EXPECT_NEAR( 1.0, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR( 0.1, p_calib->GetScale(), PE::EPSILON);

   //RAW=13 REF=1.2
   p_calib->AddRaw(13);
   p_calib->AddRef(1.2);
   p_calib->Recalculate();
   EXPECT_NEAR( 1.0, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR( 0.1, p_calib->GetScale(), PE::EPSILON);

   delete p_calib;
}


/**
 * tests  n; n+1; base=-12 and scale=0.1
 */
TEST_F(PECCalibrationSummaryTest, n_and_n_plus_1_base_minus_12_scale_0_dot_1_test)
{
   PE::ICalibration* p_calib = new PE::CCalibrationSummary();

   //RAW=-2 REF=1.0
   p_calib->AddRaw(-2);
   p_calib->AddRef(1);
   p_calib->Recalculate();
   EXPECT_TRUE( PE::isnan( p_calib->GetBias() ) );
   EXPECT_TRUE( PE::isnan( p_calib->GetScale() ) );

   //RAW=-1 REF=1.1
   p_calib->AddRaw(-1);
   p_calib->AddRef(1.1);
   p_calib->Recalculate();
   EXPECT_NEAR( -12.0, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR( 0.1, p_calib->GetScale(), PE::EPSILON);

   //RAW=0 REF=1.2
   p_calib->AddRaw(0);
   p_calib->AddRef(1.2);
   p_calib->Recalculate();
   EXPECT_NEAR( -12.0, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR( 0.1, p_calib->GetScale(), PE::EPSILON);

   delete p_calib;
}


/**
 * tests  n; n+1; base=0 and scale=0.11
 */
TEST_F(PECCalibrationSummaryTest, n_and_n_plus_1_base_0_scale_0_dot_11_test)
{
   PE::ICalibration* p_calib = new PE::CCalibrationSummary();

   //RAW=9.090909090909 REF=1.0
   p_calib->AddRaw(9.090909090909);
   p_calib->AddRef(1.0);
   p_calib->Recalculate();
   EXPECT_TRUE( PE::isnan( p_calib->GetBias() ) );
   EXPECT_TRUE( PE::isnan( p_calib->GetScale() ) );

   //RAW=10 REF=1.1
   p_calib->AddRaw(10);
   p_calib->AddRef(1.1);
   p_calib->Recalculate();
   EXPECT_NEAR( 0.0, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR( 0.11, p_calib->GetScale(), PE::EPSILON);

   //RAW=10.90909090909 REF=1.2
   p_calib->AddRaw(10.90909090909);
   p_calib->AddRef(1.2);
   p_calib->Recalculate();
   EXPECT_NEAR( 0.0, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR( 0.11, p_calib->GetScale(), PE::EPSILON);

   //RAW=11.81818181818 REF=1.3
   p_calib->AddRaw(11.81818181818);
   p_calib->AddRef(1.3);
   p_calib->Recalculate();
   EXPECT_NEAR( 0.0, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR( 0.11, p_calib->GetScale(), PE::EPSILON);

   delete p_calib;
}


/**
 * tests  n; n+1; base=-0.1234 and scale=1.2
 */
TEST_F(PECCalibrationSummaryTest, n_and_n_plus_1_base_minus_0_dot_1234_scale_1_dot_2_test)
{
   PE::ICalibration* p_calib = new PE::CCalibrationSummary();

   //RAW=0.70993333333   REF=1.0
   p_calib->AddRaw(0.70993333333);
   p_calib->AddRef(1.0);
   p_calib->Recalculate();
   EXPECT_TRUE( PE::isnan( p_calib->GetBias() ) );
   EXPECT_TRUE( PE::isnan( p_calib->GetScale() ) );

   //RAW=0.79326666667   REF=1.1
   p_calib->AddRaw(0.79326666667);
   p_calib->AddRef(1.1);
   p_calib->Recalculate();
   EXPECT_NEAR( -0.1234, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR( 1.2, p_calib->GetScale(), PE::EPSILON);

   //RAW=0.87660000000   REF=1.2
   p_calib->AddRaw(0.87660000000);
   p_calib->AddRef(1.2);
   p_calib->Recalculate();
   EXPECT_NEAR( -0.1234, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR( 1.2, p_calib->GetScale(), PE::EPSILON);

   delete p_calib;
}


/**
 * tests  n; n+1; base=-123 and scale=-5.4
 */
TEST_F(PECCalibrationSummaryTest, n_and_n_plus_1_base_minus_123_scale_minus_5_dot_4_test)
{
   PE::ICalibration* p_calib = new PE::CCalibrationSummary();

   //RAW=-123.1851851851851   REF=1.0
   p_calib->AddRaw(-123.1851851851851);
   p_calib->AddRef(1.0);
   p_calib->Recalculate();
   EXPECT_TRUE( PE::isnan( p_calib->GetBias() ) );
   EXPECT_TRUE( PE::isnan( p_calib->GetScale() ) );

   //RAW=-123.20370370370370   REF=1.1
   p_calib->AddRaw(-123.20370370370370);
   p_calib->AddRef(1.1);
   p_calib->Recalculate();
   EXPECT_NEAR( -123, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR(-5.4, p_calib->GetScale(), PE::EPSILON);

   //RAW=-123.22222222222222   REF=1.2
   p_calib->AddRaw(-123.22222222222222);
   p_calib->AddRef(1.2);
   p_calib->Recalculate();
   EXPECT_NEAR( -123, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR( -5.4, p_calib->GetScale(), PE::EPSILON);

   delete p_calib;
}


/**
 * tests  CleanLastStep bias=11 scale=0.1
 */
TEST_F(PECCalibrationSummaryTest, clean_last_step_test)
{
   PE::ICalibration* p_calib = new PE::CCalibrationSummary();

   //RAW=10+11   REF=1.0
   p_calib->AddRaw(10.0 + 11);
   p_calib->AddRef(1.0);
   p_calib->Recalculate();
   EXPECT_TRUE( PE::isnan( p_calib->GetBias() ) );
   EXPECT_TRUE( PE::isnan( p_calib->GetScale() ) );

   //RAW=20+11   REF=2.0
   p_calib->AddRaw(20 + 11);
   p_calib->AddRef(2.0);
   p_calib->Recalculate();
   EXPECT_NEAR( 11, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR( 0.1, p_calib->GetScale(), PE::EPSILON);

   //RAW=20+11   REF=2.0 deviation +0.01
   p_calib->AddRaw(20 + 0.01 + 11);
   p_calib->AddRef(2.0);
   //reject last data has to be previouse scale and base
   p_calib->CleanLastStep();
   //clean up will not change the bias and scale value
   EXPECT_NEAR( 11, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR( 0.1, p_calib->GetScale(), PE::EPSILON);

   //but recalculate on same values has to provide NaN
   p_calib->Recalculate();
   EXPECT_TRUE( PE::isnan( p_calib->GetBias() ) );
   EXPECT_TRUE( PE::isnan( p_calib->GetScale() ) );

   //RAW=20+11   REF=2.0 deviation +0.01
   p_calib->AddRaw(20 + 0.01 + 11);
   p_calib->AddRef(2.0);
   p_calib->Recalculate();
   EXPECT_NEAR( 10.9699999999999, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR(  0.0998003992016, p_calib->GetScale(), PE::EPSILON);

   //RAW=20+11   REF=2.0 deviation -0.01
   p_calib->AddRaw(20 - 0.01 + 11);
   p_calib->AddRef(2.0);
   //retry last data has to be considered
   p_calib->Recalculate();
   EXPECT_NEAR( 11.07000000000005, p_calib->GetBias(), PE::EPSILON);
   EXPECT_NEAR(  0.10040160642570, p_calib->GetScale(), PE::EPSILON);

   delete p_calib;
}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
