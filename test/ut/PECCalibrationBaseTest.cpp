/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2017 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */


/**
 * Unit test of the PECCalibrationBase class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PECCalibrationBase.h"


class PECCalibrationBaseTest : public ::testing::Test
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
TEST_F(PECCalibrationBaseTest, test_create)
{
   PE::CNormalisation base;
   PE::CCalibrationBase   calib(base);

   EXPECT_NEAR(0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR(0.000000, base.GetMean(), 0.000001);
   EXPECT_NEAR(0.000000, base.GetMld(), 0.000001);
   EXPECT_NEAR(0.000000, base.GetReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationBaseTest, test_not_empty_init_normalisation)
{
   PE::CNormalisation base;

   base.AddSensor(10);
   base.AddSensor(10);
   base.AddSensor(10);
   base.AddSensor(10);
   EXPECT_NEAR(10.000000, base.GetMean(), 0.000001);

   PE::CCalibrationBase   calib(base);
   EXPECT_NEAR( 0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR(10.000000, base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetMld(), 0.000001);
   EXPECT_NEAR(75.000000, base.GetReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationBaseTest, test_do_calibration_no_sensors_no_reference_no_scale)
{
   PE::CNormalisation base;

   base.AddSensor(10);
   base.AddSensor(10);
   base.AddSensor(10);
   base.AddSensor(10);
   EXPECT_NEAR(10.000000, base.GetMean(), 0.000001);

   PE::CCalibrationBase   calib(base);
   calib.DoCalibration();
   calib.DoCalibration();
   calib.DoCalibration();
   calib.DoCalibration();
   calib.DoCalibration();
   EXPECT_NEAR( 0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR(10.000000, base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetMld(), 0.000001);
   EXPECT_NEAR(75.000000, base.GetReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationBaseTest, test_do_calibration_no_sensors_no_reference_with_scale)
{
   PE::CNormalisation base;

   base.AddSensor(10);
   base.AddSensor(10);
   base.AddSensor(10);
   base.AddSensor(10);
   EXPECT_NEAR(10.000000, base.GetMean(), 0.000001);

   PE::CCalibrationBase   calib(base);
   calib.AddScale(10000);
   calib.DoCalibration();
   calib.AddScale(10000);
   calib.DoCalibration();
   calib.AddScale(10000);
   calib.DoCalibration();
   calib.AddScale(10000);
   calib.DoCalibration();
   calib.AddScale(10000);
   calib.DoCalibration();
   //no effect since no valid sensor and reference values
   EXPECT_NEAR( 0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR(10.000000, base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetMld(), 0.000001);
   EXPECT_NEAR(75.000000, base.GetReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationBaseTest, test_do_calibration_no_sensors_but_valid_reference_and_scale)
{
   PE::CNormalisation base;

   base.AddSensor(10);
   base.AddSensor(10);
   base.AddSensor(10);
   base.AddSensor(10);
   EXPECT_NEAR(10.000000, base.GetMean(), 0.000001);

   PE::CCalibrationBase   calib(base);
   calib.AddScale(10000);
   calib.AddReference(10);
   calib.DoCalibration();
   calib.AddReference(20);
   calib.AddScale(10000);
   calib.DoCalibration();
   calib.AddReference(30);
   calib.AddScale(10000);
   calib.DoCalibration();
   EXPECT_NEAR( 0.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR(10.000000, base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetMld(), 0.000001);
   EXPECT_NEAR(75.000000, base.GetReliable(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationBaseTest, test_do_calibration_sensors_reference_and_scale_are_valid)
{
   PE::CNormalisation base;
   PE::CCalibrationBase   calib(base);

   calib.AddScale(2);

   calib.AddReference(20);
   calib.AddSensor(11);
   calib.DoCalibration();
   EXPECT_NEAR( 2.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetMld(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetReliable(), 0.000001);

   calib.AddReference(40);
   calib.AddSensor(21);
   calib.DoCalibration();
   EXPECT_NEAR( 2.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 2.000000, base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetMld(), 0.000001);
   EXPECT_NEAR(50.000000, base.GetReliable(), 0.000001);

   calib.AddReference(2);
   calib.AddSensor(2);
   calib.DoCalibration();
   EXPECT_NEAR( 2.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 2.000000, base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetMld(), 0.000001);
   EXPECT_NEAR(66.666667, base.GetReliable(), 0.000001);

   //general check
   EXPECT_NEAR(40.000000 , 21 * 2 - base.GetMean(), 0.000001);
}


/**
 * tests  
 */
TEST_F(PECCalibrationBaseTest, test_reset)
{
   PE::CNormalisation base;
   PE::CCalibrationBase   calib(base);

   calib.AddScale(2);

   calib.AddReference(20);
   calib.AddSensor(11);
   calib.DoCalibration();
   EXPECT_NEAR( 2.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetMld(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetReliable(), 0.000001);

   calib.AddReference(40);
   calib.AddSensor(21);
   calib.DoCalibration();
   EXPECT_NEAR( 2.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 2.000000, base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetMld(), 0.000001);
   EXPECT_NEAR(50.000000, base.GetReliable(), 0.000001);

   calib.AddReference(2);
   calib.AddSensor(2);
   calib.Reset();
   //values same as before
   EXPECT_NEAR( 2.000000, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 2.000000, base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetMld(), 0.000001);
   EXPECT_NEAR(50.000000, base.GetReliable(), 0.000001);
}


TEST_F(PECCalibrationBaseTest, test_gyro_hamburg_airport_b433_18_plus_9sec_before)
{
   PE::CNormalisation base;
   PE::CCalibrationBase   calib(base);

   calib.AddScale(0.009590);

   calib.AddSensor( -1100);
   calib.AddSensor( -1124);
   calib.AddSensor( -1136);
   calib.AddSensor( -1157);
   calib.AddSensor( -1172);
   calib.AddSensor( -1192);
   calib.AddSensor( -1201);
   calib.AddSensor( -1199);
   calib.AddSensor( -1200);
   calib.AddSensor( -1205);
   calib.AddSensor( -1206);
   calib.AddSensor( -1182);
   calib.AddSensor( -1156);
   calib.AddSensor( -1134);
   calib.AddSensor( -1111);
   calib.AddSensor( -1116);
   calib.AddSensor( -1129);
   calib.AddReference(-11.38860539); //hAcc=0.3[deg]
   calib.DoCalibration();
   EXPECT_NEAR( 0.264205, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base.GetMld(), 0.000001);
   EXPECT_NEAR( 0.00, base.GetReliable(), 0.01);

   calib.AddSensor( -1118);
   calib.AddSensor( -1101);
   calib.AddSensor( -1082);
   calib.AddSensor( -1045);
   calib.AddSensor( -988);
   calib.AddSensor( -935);
   calib.AddSensor( -887);
   calib.AddSensor( -868);
   calib.AddSensor( -850);
   calib.AddSensor( -846);
   calib.AddSensor( -840);
   calib.AddSensor( -855);
   calib.AddSensor( -839);
   calib.AddSensor( -836);
   calib.AddSensor( -823);
   calib.AddSensor( -827);
   calib.AddReference(-10.61061662); //hAcc=0.3[deg]
   calib.DoCalibration();
//EXPECT_NEAR( 0.985326, calib.GetBase(), 0.000001);
//EXPECT_NEAR( 0.624766, base.GetMean(), 0.000001);
//EXPECT_NEAR( 0.360560, base.GetMld(), 0.000001);
   EXPECT_NEAR( 1.020017, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.642111, base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.377905, base.GetMld(), 0.000001);
   EXPECT_NEAR( 0.00, base.GetReliable(), 0.01);

   calib.AddSensor( -830);
   calib.AddSensor( -849);
   calib.AddSensor( -843);
   calib.AddSensor( -849);
   calib.AddSensor( -859);
   calib.AddSensor( -846);
   calib.AddSensor( -880);
   calib.AddSensor( -866);
   calib.AddSensor( -907);
   calib.AddSensor( -926);
   calib.AddSensor( -966);
   calib.AddSensor( -964);
   calib.AddSensor( -945);
   calib.AddSensor( -954);
   calib.AddSensor( -967);
   calib.AddSensor( -1002);
   calib.AddSensor( -1032);
   calib.AddReference(-8.699997); //hAcc=0.3[deg]
   calib.DoCalibration();
//EXPECT_NEAR( 0.653622, calib.GetBase(), 0.000001);
//EXPECT_NEAR( 0.634385, base.GetMean(), 0.000001);
//EXPECT_NEAR( 0.189898, base.GetMld(), 0.000001);
//EXPECT_NEAR(31.65, base.GetReliable(), 0.01);
   EXPECT_NEAR( 0.668223, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.650815, base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.197656, base.GetMld(), 0.000001);
   EXPECT_NEAR(31.86, base.GetReliable(), 0.01);

   calib.AddSensor( -1047);
   calib.AddSensor( -1043);
   calib.AddSensor( -1034);
   calib.AddSensor( -1046);
   calib.AddSensor( -1065);
   calib.AddSensor( -1090);
   calib.AddSensor( -1109);
   calib.AddSensor( -1118);
   calib.AddSensor( -1122);
   calib.AddSensor( -1147);
   calib.AddSensor( -1129);
   calib.AddSensor( -1130);
   calib.AddSensor( -1111);
   calib.AddSensor( -1109);
   calib.AddSensor( -1096);
   calib.AddSensor( -1079);
   calib.AddSensor( -1073);
   calib.AddReference(-10.599991); //hAcc=0.3[deg]
   calib.DoCalibration();
//EXPECT_NEAR( 0.521103, calib.GetBase(), 0.000001);
//EXPECT_NEAR( 0.606064, base.GetMean(), 0.000001);
//EXPECT_NEAR( 0.154920, base.GetMld(), 0.000001);
//EXPECT_NEAR(44.16, base.GetReliable(), 0.01);
   EXPECT_NEAR( 0.535351, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.621949, base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.160637, base.GetMld(), 0.000001);
   EXPECT_NEAR(44.40, base.GetReliable(), 0.01);

   calib.AddSensor( -1052);
   calib.AddSensor( -1021);
   calib.AddSensor( -1003);
   calib.AddSensor( -961);
   calib.AddSensor( -947);
   calib.AddSensor( -943);
   calib.AddSensor( -936);
   calib.AddSensor( -951);
   calib.AddSensor( -930);
   calib.AddSensor( -927);
   calib.AddSensor( -920);
   calib.AddSensor( -931);
   calib.AddSensor( -922);
   calib.AddSensor( -928);
   calib.AddSensor( -926);
   calib.AddSensor( -922);
   calib.AddReference(-10.400009); //hAcc=0.3[deg]
   calib.DoCalibration();
//    EXPECT_NEAR( 0.667462, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.618344, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.128469, base.GetMld(), 0.000001);
//    EXPECT_NEAR(53.42, base.GetReliable(), 0.01);
   EXPECT_NEAR( 0.683785, calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.634316, base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.132845, base.GetMld(), 0.000001);
   EXPECT_NEAR(53.66, base.GetReliable(), 0.01);

   calib.AddSensor( -910);
   calib.AddSensor( -914);
   calib.AddSensor( -931);
   calib.AddSensor( -910);
   calib.AddSensor( -924);
   calib.AddSensor( -905);
   calib.AddSensor( -907);
   calib.AddSensor( -897);
   calib.AddSensor( -881);
   calib.AddSensor( -854);
   calib.AddSensor( -816);
   calib.AddSensor( -771);
   calib.AddSensor( -711);
   calib.AddSensor( -655);
   calib.AddSensor( -607);
   calib.AddSensor( -561);
   calib.AddSensor( -512);
   calib.AddReference(-8.999985); //hAcc=0.3[deg]
   calib.DoCalibration();
//    EXPECT_NEAR( 0.777888, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.644934, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.129366, base.GetMld(), 0.000001);
//    EXPECT_NEAR(57.76, base.GetReliable(), 0.01);

   EXPECT_NEAR( 0.784946,  calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.659421,  base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.131381,  base.GetMld(), 0.000001);
   EXPECT_NEAR( 58.20,  base.GetReliable(), 0.01);


   calib.AddSensor( -469);
   calib.AddSensor( -399);
   calib.AddSensor( -296);
   calib.AddSensor( -222);
   calib.AddSensor( -156);
   calib.AddSensor( -133);
   calib.AddSensor( -94);
   calib.AddSensor( -60);
   calib.AddSensor( -27);
   calib.AddSensor( 5);
   calib.AddSensor( 3);
   calib.AddSensor( 4);
   calib.AddSensor( 30);
   calib.AddSensor( 53);
   calib.AddSensor( 95);
   calib.AddSensor( 111);
   calib.AddSensor( 125);
   calib.AddReference( -4.100006); //hAcc=0.3[deg]
   calib.DoCalibration();
//    EXPECT_NEAR( 1.158070, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.718240, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.181110, base.GetMld(), 0.000001);
//    EXPECT_NEAR(58.01, base.GetReliable(), 0.01);

   EXPECT_NEAR( 1.143285,  calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.728545,  base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.178607,  base.GetMld(), 0.000001);
   EXPECT_NEAR( 58.64,  base.GetReliable(), 0.01);

   calib.AddSensor( 127);
   calib.AddSensor( 126);
   calib.AddSensor( 134);
   calib.AddSensor( 123);
   calib.AddSensor( 120);
   calib.AddSensor( 121);
   calib.AddSensor( 127);
   calib.AddSensor( 126);
   calib.AddSensor( 121);
   calib.AddSensor( 123);
   calib.AddSensor( 128);
   calib.AddSensor( 130);
   calib.AddSensor( 150);
   calib.AddSensor( 158);
   calib.AddSensor( 168);
   calib.AddSensor( 190);
   calib.AddReference( 1.299988); //hAcc=0.3[deg]
   calib.DoCalibration();
//    EXPECT_NEAR( 0.969366, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.749630, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.186628, base.GetMld(), 0.000001);
//    EXPECT_NEAR(61.16, base.GetReliable(), 0.01);

   EXPECT_NEAR( 1.000606,  calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.762552,  base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.187100,  base.GetMld(), 0.000001);
   EXPECT_NEAR( 61.54,  base.GetReliable(), 0.01);

   calib.AddSensor( 198);
   calib.AddSensor( 190);
   calib.AddSensor( 186);
   calib.AddSensor( 190);
   calib.AddSensor( 176);
   calib.AddSensor( 178);
   calib.AddSensor( 159);
   calib.AddSensor( 169);
   calib.AddSensor( 168);
   calib.AddSensor( 168);
   calib.AddSensor( 176);
   calib.AddSensor( 187);
   calib.AddSensor( 201);
   calib.AddSensor( 210);
   calib.AddSensor( 223);
   calib.AddSensor( 241);
   calib.AddSensor( 250);
   calib.AddReference( 1.600006); //hAcc=0.3[deg]
   calib.DoCalibration();
//    EXPECT_NEAR( 0.908427, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.767274, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.180944, base.GetMld(), 0.000001);
//    EXPECT_NEAR(64.39, base.GetReliable(), 0.01);

   EXPECT_NEAR( 0.916612,  calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.779670,  base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.180830,  base.GetMld(), 0.000001);
   EXPECT_NEAR( 64.76,  base.GetReliable(), 0.01);

   calib.AddSensor( 258 );
   calib.AddSensor( 243 );
   calib.AddSensor( 252 );
   calib.AddSensor( 240 );
   calib.AddSensor( 247 );
   calib.AddSensor( 253 );
   calib.AddSensor( 265 );
   calib.AddSensor( 274 );
   calib.AddSensor( 297 );
   calib.AddSensor( 330 );
   calib.AddSensor( 365 );
   calib.AddSensor( 402 );
   calib.AddSensor( 408 );
   calib.AddSensor( 410 );
   calib.AddSensor( 423 );
   calib.AddSensor( 419 );
   calib.AddSensor( 427 );
   calib.AddReference( 2.497502498 ); //hAcc=0.15[deg]
   calib.DoCalibration();
//    EXPECT_NEAR( 0.895142, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.780061, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.173626, base.GetMld(), 0.000001);
//    EXPECT_NEAR(67.21, base.GetReliable(), 0.01);
   EXPECT_NEAR( 0.886199,  calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.790323,  base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.171390,  base.GetMld(), 0.000001);
   EXPECT_NEAR( 67.66,  base.GetReliable(), 0.01);

   calib.AddSensor( 428 );
   calib.AddSensor( 438 );
   calib.AddSensor( 464 );
   calib.AddSensor( 491 );
   calib.AddSensor( 508 );
   calib.AddSensor( 513 );
   calib.AddSensor( 503 );
   calib.AddSensor( 514 );
   calib.AddSensor( 498 );
   calib.AddSensor( 537 );
   calib.AddSensor( 557 );
   calib.AddSensor( 595 );
   calib.AddSensor( 621 );
   calib.AddSensor( 640 );
   calib.AddSensor( 651 );
   calib.AddSensor( 645 );
   calib.AddReference( 4.704701702 ); //hAcc=0.15[deg]
   calib.DoCalibration();
//    EXPECT_NEAR( 0.819357, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.783634, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.159835, base.GetMld(), 0.000001);
//    EXPECT_NEAR(69.99, base.GetReliable(), 0.01);
   EXPECT_NEAR( 0.846701,  calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.795448,  base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.159377,  base.GetMld(), 0.000001);
   EXPECT_NEAR( 70.31,  base.GetReliable(), 0.01);

   calib.AddSensor( 652 );
   calib.AddSensor( 669 );
   calib.AddSensor( 686 );
   calib.AddSensor( 703 );
   calib.AddSensor( 721 );
   calib.AddSensor( 737 );
   calib.AddSensor( 723 );
   calib.AddSensor( 731 );
   calib.AddSensor( 730 );
   calib.AddSensor( 733 );
   calib.AddSensor( 714 );
   calib.AddSensor( 706 );
   calib.AddSensor( 700 );
   calib.AddSensor( 701 );
   calib.AddSensor( 704 );
   calib.AddSensor( 707 );
   calib.AddSensor( 705 );
   calib.AddReference( 6.900009 ); //hAcc=0.15[deg]
   calib.DoCalibration();
//    EXPECT_NEAR( 0.759453, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.781619, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.147320, base.GetMld(), 0.000001);
//    EXPECT_NEAR(72.38, base.GetReliable(), 0.01);
   EXPECT_NEAR( 0.766293,  calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.793019,  base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.147317,  base.GetMld(), 0.000001);
   EXPECT_NEAR( 72.64,  base.GetReliable(), 0.01);

   calib.AddSensor( 714 );
   calib.AddSensor( 719 );
   calib.AddSensor( 738 );
   calib.AddSensor( 756 );
   calib.AddSensor( 764 );
   calib.AddSensor( 768 );
   calib.AddSensor( 764 );
   calib.AddSensor( 752 );
   calib.AddSensor( 751 );
   calib.AddSensor( 756 );
   calib.AddSensor( 757 );
   calib.AddSensor( 779 );
   calib.AddSensor( 794 );
   calib.AddSensor( 791 );
   calib.AddSensor( 796 );
   calib.AddSensor( 787 );
   calib.AddSensor( 798 );
   calib.AddReference( 7.399994 ); //hAcc=0.15[deg]
   calib.DoCalibration();
//    EXPECT_NEAR( 0.710183, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.776124, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.140538, base.GetMld(), 0.000001);
//    EXPECT_NEAR(74.20, base.GetReliable(), 0.01);
   EXPECT_NEAR( 0.701541,  calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.785982,  base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.142078,  base.GetMld(), 0.000001);
   EXPECT_NEAR( 74.37,  base.GetReliable(), 0.01);

   calib.AddSensor( 808 );
   calib.AddSensor( 837 );
   calib.AddSensor( 852 );
   calib.AddSensor( 861 );
   calib.AddSensor( 860 );
   calib.AddSensor( 848 );
   calib.AddSensor( 846 );
   calib.AddSensor( 848 );
   calib.AddSensor( 851 );
   calib.AddSensor( 856 );
   calib.AddSensor( 863 );
   calib.AddSensor( 843 );
   calib.AddSensor( 811 );
   calib.AddSensor( 790 );
   calib.AddSensor( 751 );
   calib.AddSensor( 738 );
   calib.AddReference( 8.399994 ); //hAcc=0.3[deg]
   calib.DoCalibration();
//    EXPECT_NEAR( 0.598731, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.763453, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.142399, base.GetMld(), 0.000001);
//    EXPECT_NEAR(75.41, base.GetReliable(), 0.01);
   EXPECT_NEAR( 0.619253,  calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.774073,  base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.143058,  base.GetMld(), 0.000001);
   EXPECT_NEAR( 75.60,  base.GetReliable(), 0.01);

   calib.AddSensor( 726 );
   calib.AddSensor( 716 );
   calib.AddSensor( 729 );
   calib.AddSensor( 750 );
   calib.AddSensor( 795 );
   calib.AddSensor( 796 );
   calib.AddSensor( 781 );
   calib.AddSensor( 780 );
   calib.AddSensor( 763 );
   calib.AddSensor( 762 );
   calib.AddSensor( 773 );
   calib.AddSensor( 773 );
   calib.AddSensor( 783 );
   calib.AddSensor( 781 );
   calib.AddSensor( 765 );
   calib.AddSensor( 748 );
   calib.AddSensor( 731 );
   calib.AddReference( 7.800003 ); //hAcc=0.3[deg]
   calib.DoCalibration();
//    EXPECT_NEAR( 0.537903, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.748416, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.147264, base.GetMld(), 0.000001);
//    EXPECT_NEAR(76.37, base.GetReliable(), 0.01);
   EXPECT_NEAR( 0.545066,  calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.758806,  base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.148106,  base.GetMld(), 0.000001);
   EXPECT_NEAR( 76.54,  base.GetReliable(), 0.01);

   calib.AddSensor( 725 );
   calib.AddSensor( 727 );
   calib.AddSensor( 731 );
   calib.AddSensor( 719 );
   calib.AddSensor( 693 );
   calib.AddSensor( 676 );
   calib.AddSensor( 665 );
   calib.AddSensor( 671 );
   calib.AddSensor( 677 );
   calib.AddSensor( 690 );
   calib.AddSensor( 694 );
   calib.AddSensor( 708 );
   calib.AddSensor( 716 );
   calib.AddSensor( 716 );
   calib.AddSensor( 724 );
   calib.AddSensor( 715 );
   calib.AddSensor( 710 );
   calib.AddReference( 7.400009 ); //hAcc=0.3[deg]
   calib.DoCalibration();
//    EXPECT_NEAR( 0.472509, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.731172, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.154690, base.GetMld(), 0.000001);
//    EXPECT_NEAR(77.15, base.GetReliable(), 0.01);
   EXPECT_NEAR( 0.470071,  calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.740760,  base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.156278,  base.GetMld(), 0.000001);
   EXPECT_NEAR( 77.29,  base.GetReliable(), 0.01);

   calib.AddSensor( 714 );
   calib.AddSensor( 699 );
   calib.AddSensor( 682 );
   calib.AddSensor( 656 );
   calib.AddSensor( 636 );
   calib.AddSensor( 615 );
   calib.AddSensor( 626 );
   calib.AddSensor( 625 );
   calib.AddSensor( 644 );
   calib.AddSensor( 638 );
   calib.AddSensor( 643 );
   calib.AddSensor( 637 );
   calib.AddSensor( 650 );
   calib.AddSensor( 656 );
   calib.AddSensor( 659 );
   calib.AddSensor( 652 );
   calib.AddReference( 6.793194805 ); //hAcc=0.3[deg]
   calib.DoCalibration();
//    EXPECT_NEAR( 0.397303, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.711532, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.164662, base.GetMld(), 0.000001);
//    EXPECT_NEAR(77.79, base.GetReliable(), 0.01);

   EXPECT_NEAR( 0.410625,  calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.721340,  base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.165931,  base.GetMld(), 0.000001);
   EXPECT_NEAR( 77.93,  base.GetReliable(), 0.01);
   
   calib.AddSensor( 660 );
   calib.AddSensor( 658 );
   calib.AddSensor( 661 );
   calib.AddSensor( 674 );
   calib.AddSensor( 672 );
   calib.AddSensor( 692 );
   calib.AddSensor( 689 );
   calib.AddSensor( 685 );
   calib.AddSensor( 691 );
   calib.AddSensor( 679 );
   calib.AddSensor( 670 );
   calib.AddSensor( 667 );
   calib.AddSensor( 652 );
   calib.AddSensor( 657 );
   calib.AddSensor( 655 );
   calib.AddSensor( 652 );
   calib.AddSensor( 642 );
   calib.AddReference( 6.80680981 ); //hAcc=0.3[deg]
   calib.DoCalibration();
//    EXPECT_NEAR( 0.360301, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.692020, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.174489, base.GetMld(), 0.000001);
//    EXPECT_NEAR(78.40, base.GetReliable(), 0.01);
   EXPECT_NEAR( 0.365552,  calib.GetBase(), 0.000001);
   EXPECT_NEAR( 0.701574,  base.GetMean(), 0.000001);
   EXPECT_NEAR( 0.175936,  base.GetMld(), 0.000001);
   EXPECT_NEAR( 78.53,  base.GetReliable(), 0.01);

   calib.AddSensor( 16 );
   calib.AddSensor( 5 );
   calib.AddSensor( -6 );
   calib.AddSensor( -28 );
   calib.AddSensor( -67 );
   calib.AddSensor( -84 );
   calib.AddSensor( -83 );
   calib.AddSensor( -103 );
   calib.AddSensor( -103 );
   calib.AddSensor( -99 );
   calib.AddSensor( -100 );
   calib.AddSensor( -95 );
   calib.AddSensor( -93 );
   calib.AddSensor( -102 );
   calib.AddSensor( -93 );
   calib.AddSensor( -100 );
   calib.AddSensor( -108 );
   calib.AddReference( -0.300003 );
   calib.DoCalibration();
//    EXPECT_NEAR( 0.319341, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.672405, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.184409, base.GetMld(), 0.000001);
//    EXPECT_NEAR(78.98, base.GetReliable(), 0.01);
// 
//    calib.AddSensor( -86 );
//    calib.AddSensor( -81 );
//    calib.AddSensor( -89 );
//    calib.AddSensor( -97 );
//    calib.AddSensor( -105 );
//    calib.AddSensor( -102 );
//    calib.AddSensor( -90 );
//    calib.AddSensor( -91 );
//    calib.AddSensor( -81 );
//    calib.AddSensor( -84 );
//    calib.AddSensor( -82 );
//    calib.AddSensor( -73 );
//    calib.AddSensor( -103 );
//    calib.AddSensor( -119 );
//    calib.AddSensor( -92 );
//    calib.AddSensor( -65 );
//    calib.AddReference( -0.800003 );
//    calib.DoCalibration();
//    EXPECT_NEAR( 0.302168, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.653893, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.193215, base.GetMld(), 0.000001);
//    EXPECT_NEAR(79.55, base.GetReliable(), 0.01);
// 
//    calib.AddSensor( -51 );
//    calib.AddSensor( -81 );
//    calib.AddSensor( -111 );
//    calib.AddSensor( -116 );
//    calib.AddSensor( -115 );
//    calib.AddSensor( -102 );
//    calib.AddSensor( -91 );
//    calib.AddSensor( -99 );
//    calib.AddSensor( -83 );
//    calib.AddSensor( -85 );
//    calib.AddSensor( -70 );
//    calib.AddSensor( -68 );
//    calib.AddSensor( -69 );
//    calib.AddSensor( -70 );
//    calib.AddSensor( -71 );
//    calib.AddSensor( -81 );
//    calib.AddSensor( -84 );
//    calib.AddReference( -1.300003 );
//    calib.DoCalibration();
//    EXPECT_NEAR( 0.309953, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.637515, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.199932, base.GetMld(), 0.000001);
//    EXPECT_NEAR(80.14, base.GetReliable(), 0.01);
// 
//    calib.AddSensor( -84 );
//    calib.AddSensor( -89 );
//    calib.AddSensor( -87 );
//    calib.AddSensor( -78 );
//    calib.AddSensor( -77 );
//    calib.AddSensor( -62 );
//    calib.AddSensor( -70 );
//    calib.AddSensor( -75 );
//    calib.AddSensor( -72 );
//    calib.AddSensor( -57 );
//    calib.AddSensor( -70 );
//    calib.AddSensor( -40 );
//    calib.AddSensor( -34 );
//    calib.AddSensor( -43 );
//    calib.AddSensor( -42 );
//    calib.AddSensor( -38 );
//    calib.AddSensor( -40 );
//    calib.AddReference( -0.799988 );
//    calib.DoCalibration();
//    EXPECT_NEAR( 0.304543, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.622380, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.205547, base.GetMld(), 0.000001);
//    EXPECT_NEAR(80.70, base.GetReliable(), 0.01);
// 
//    calib.AddSensor( -37 );
//    calib.AddSensor( -42 );
//    calib.AddSensor( -33 );
//    calib.AddSensor( -20 );
//    calib.AddSensor( -14 );
//    calib.AddSensor( 2 );
//    calib.AddSensor( 7 );
//    calib.AddSensor( 8 );
//    calib.AddSensor( 12 );
//    calib.AddSensor( 19 );
//    calib.AddSensor( 22 );
//    calib.AddSensor( 17 );
//    calib.AddSensor( 20 );
//    calib.AddSensor( 11 );
//    calib.AddSensor( 17 );
//    calib.AddSensor( 11 );
//    calib.AddReference( -0.400009 );
//    EXPECT_NEAR( 0.304543, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.622380, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.205547, base.GetMld(), 0.000001);
//    EXPECT_NEAR(80.70, base.GetReliable(), 0.01);
// 
//    calib.DoCalibration();
//    calib.AddSensor( 14 );
//    calib.AddSensor( 15 );
//    calib.AddSensor( 8 );
//    calib.AddSensor( 15 );
//    calib.AddSensor( 10 );
//    calib.AddSensor( 8 );
//    calib.AddSensor( 2 );
//    calib.AddSensor( -3 );
//    calib.AddSensor( -2 );
//    calib.AddSensor( 0 );
//    calib.AddSensor( -8 );
//    calib.AddSensor( -18 );
//    calib.AddSensor( -40 );
//    calib.AddSensor( -51 );
//    calib.AddSensor( -51 );
//    calib.AddSensor( -73 );
//    calib.AddSensor( -79 );
//    calib.AddReference( 0.199997 );
//    EXPECT_NEAR( 0.308717, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.608742, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.209842, base.GetMld(), 0.000001);
//    EXPECT_NEAR(81.26, base.GetReliable(), 0.01);
// 
//    calib.DoCalibration();
//    calib.AddSensor( -111 );
//    calib.AddSensor( -154 );
//    calib.AddSensor( -187 );
//    calib.AddSensor( -219 );
//    calib.AddSensor( -244 );
//    calib.AddSensor( -280 );
//    calib.AddSensor( -327 );
//    calib.AddSensor( -390 );
//    calib.AddSensor( -457 );
//    calib.AddSensor( -505 );
//    calib.AddSensor( -544 );
//    calib.AddSensor( -580 );
//    calib.AddSensor( -628 );
//    calib.AddSensor( -639 );
//    calib.AddSensor( -672 );
//    calib.AddSensor( -659 );
//    calib.AddSensor( -677 );
//    calib.AddReference( -2.099991 );
//    EXPECT_NEAR( 0.281444, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.595105, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.214355, base.GetMld(), 0.000001);
//    EXPECT_NEAR(81.78, base.GetReliable(), 0.01);
// 
//    calib.DoCalibration();
//    calib.AddSensor( -692 );
//    calib.AddSensor( -726 );
//    calib.AddSensor( -766 );
//    calib.AddSensor( -810 );
//    calib.AddSensor( -841 );
//    calib.AddSensor( -870 );
//    calib.AddSensor( -880 );
//    calib.AddSensor( -891 );
//    calib.AddSensor( -885 );
//    calib.AddSensor( -908 );
//    calib.AddSensor( -910 );
//    calib.AddSensor( -922 );
//    calib.AddSensor( -944 );
//    calib.AddSensor( -974 );
//    calib.AddSensor( -989 );
//    calib.AddSensor( -1001 );
//    calib.AddReference( -7.199997 );
//    calib.DoCalibration();
//    EXPECT_NEAR( 0.146142, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.562138, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.229521, base.GetMld(), 0.000001);
//    EXPECT_NEAR(82.62, base.GetReliable(), 0.01);
// 
//    calib.AddSensor( -1005 );
//    calib.AddSensor( -1013 );
//    calib.AddSensor( -1028 );
//    calib.AddSensor( -1036 );
//    calib.AddSensor( -1038 );
//    calib.AddSensor( -1038 );
//    calib.AddSensor( -1035 );
//    calib.AddSensor( -1031 );
//    calib.AddSensor( -1041 );
//    calib.AddSensor( -1038 );
//    calib.AddSensor( -1050 );
//    calib.AddSensor( -1059 );
//    calib.AddSensor( -1059 );
//    calib.AddSensor( -1059 );
//    calib.AddSensor( -1073 );
//    calib.AddSensor( -1081 );
//    calib.AddSensor( -1085 );
//    calib.AddReference( -10.200012 );
//    calib.DoCalibration();
//    EXPECT_NEAR( 0.140175, calib.GetBase(), 0.000001);
//    EXPECT_NEAR( 0.546510, base.GetMean(), 0.000001);
//    EXPECT_NEAR( 0.236322, base.GetMld(), 0.000001);
//    EXPECT_NEAR(83.01, base.GetReliable(), 0.01);
}


/**
 * tests  
 */
TEST_F(PECCalibrationBaseTest, test_do_calibration_between_sensors_and_references)
{
   PE::CNormalisation base1;
   PE::CNormalisation base2;
   PE::CNormalisation base3;
   PE::CCalibrationBase   calib1(base1);
   PE::CCalibrationBase   calib2(base2);
   PE::CCalibrationBase   calib3(base3);

   calib1.AddScale(1);
   calib2.AddScale(1);
   calib3.AddScale(1);

   //sen,sen,sen,ref
   calib1.AddSensor(1);
   calib1.DoCalibration(); //ignored
   EXPECT_NEAR( 0.000000, base1.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetMld(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetReliable(), 0.000001);
   calib1.AddSensor(1);
   calib1.DoCalibration(); //ignored
   EXPECT_NEAR( 0.000000, base1.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetMld(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetReliable(), 0.000001);
   calib1.AddSensor(1);
   calib1.DoCalibration(); //ignored
   EXPECT_NEAR( 0.000000, base1.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetMld(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetReliable(), 0.000001);
   calib1.AddReference(10);
   calib1.DoCalibration(); //processed but only ONE value is not considerred
   EXPECT_NEAR( 0.000000, base1.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetMld(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetReliable(), 0.000001);

   //ref,ref,ref,sen
   calib1.AddReference(9);
   calib1.DoCalibration(); //ignored
   EXPECT_NEAR( 0.000000, base1.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetMld(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetReliable(), 0.000001);
   calib1.AddReference(9);
   calib1.DoCalibration(); //ignored
   EXPECT_NEAR( 0.000000, base1.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetMld(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetReliable(), 0.000001);
   calib1.AddReference(9);
   calib1.DoCalibration(); //ignored
   EXPECT_NEAR( 0.000000, base1.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetMld(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetReliable(), 0.000001);
   calib1.AddSensor(0);
   calib1.DoCalibration(); //processed
   EXPECT_NEAR(-9.000000, base1.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetMld(), 0.000001);
   EXPECT_NEAR(50.000000, base1.GetReliable(), 0.000001);

   //ref,sen
   calib1.AddReference(19);
   calib1.AddSensor(10);
   calib1.DoCalibration(); //processed
   EXPECT_NEAR(-9.000000, base1.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetMld(), 0.000001);
   EXPECT_NEAR(66.666667, base1.GetReliable(), 0.000001);
   //sen,ref
   calib1.AddSensor(11);
   calib1.AddReference(20);
   calib1.DoCalibration(); //processed
   EXPECT_NEAR(-9.000000, base1.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetMld(), 0.000001);
   EXPECT_NEAR(75.000000, base1.GetReliable(), 0.000001);
   //ref,sen
   calib1.AddReference(100);
   calib1.AddSensor(91);
   calib1.DoCalibration(); //processed
   EXPECT_NEAR(-9.000000, base1.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetMld(), 0.000001);
   EXPECT_NEAR(80.000000, base1.GetReliable(), 0.000001);
   //sen,ref
   calib1.AddSensor(19);
   calib1.AddReference(28);
   calib1.DoCalibration(); //processed
   EXPECT_NEAR(-9.000000, base1.GetMean(), 0.000001);
   EXPECT_NEAR( 0.000000, base1.GetMld(), 0.000001);
   EXPECT_NEAR(83.333333, base1.GetReliable(), 0.000001);


}


int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

