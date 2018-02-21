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
 * Unit test of the PECFusionSensor class.
 *
 * Code under test:
 *
 */

#include <gtest/gtest.h>
#include "PECFusionSensor.h"
#include "PETools.h"


class PECFusionSensorTest : public ::testing::Test
{
public:
   virtual void SetUp()
   {}
   virtual void TearDown()
   {}
};

//test cases:
//check skipping sensors data with outdated timesatmp
//check if all start value are invalid and add only one set of sensors. -> getters have to return same value
//test do fusion wihtout any data
//compare simple and complex fusions
//test stored position, but no new sensors data
//test with invalid init speed/angSpeed
//test with zero init speed/angSpeed


/**
 * create test
 */
TEST_F(PECFusionSensorTest, test_create )
{
   PE::SPosition pos = PE::SPosition(50.0,10.0,0.1);//lat=50 lon=10
   PE::SBasicSensor heading = PE::SBasicSensor(90.0,5.0);//90deg
   PE::SBasicSensor speed = PE::SBasicSensor(5.0,0.1); //5m/s
   PE::SBasicSensor angSpeed = PE::SBasicSensor(10.0,0.1);//10deg/s

   //all input data are valid
   PE::CFusionSensor fusion = PE::CFusionSensor(1000.0, pos, heading, angSpeed, speed);
   EXPECT_EQ(1000.0, fusion.GetTimestamp());

   EXPECT_TRUE(fusion.GetPosition().IsValid());
   EXPECT_EQ(50.0, fusion.GetPosition().Latitude);
   EXPECT_EQ(10.0, fusion.GetPosition().Longitude);
   EXPECT_EQ(0.1, fusion.GetPosition().HorizontalAcc);

   EXPECT_TRUE(fusion.GetHeading().IsValid());
   EXPECT_EQ(90.0, fusion.GetHeading().Value);
   EXPECT_EQ(5.0, fusion.GetHeading().Accuracy);

   EXPECT_TRUE(fusion.GetSpeed().IsValid());
   EXPECT_EQ(5.0,fusion.GetSpeed().Value);
   EXPECT_EQ(0.1,fusion.GetSpeed().Accuracy);

   EXPECT_TRUE(fusion.GetAngSpeed().IsValid());
   EXPECT_EQ(10.0, fusion.GetAngSpeed().Value);
   EXPECT_EQ(0.1, fusion.GetAngSpeed().Accuracy);

   //all input data are invalid
   PE::CFusionSensor fusion2 = PE::CFusionSensor(0.0,PE::SPosition(),PE::SBasicSensor(),PE::SBasicSensor(),PE::SBasicSensor());
   EXPECT_EQ(0.0, fusion2.GetTimestamp());
   EXPECT_FALSE(fusion2.GetPosition().IsValid());
   EXPECT_FALSE(fusion2.GetHeading().IsValid());
   EXPECT_FALSE(fusion2.GetSpeed().IsValid());
   EXPECT_FALSE(fusion2.GetAngSpeed().IsValid());
}


/**
 * Test incorrect timestamp of position.
 */
TEST_F(PECFusionSensorTest, test_incorrect_timestamp_pos )
{
   PE::SPosition pos = PE::SPosition(50.0, 10.0, 1);//lat=50 lon=10
   PE::SBasicSensor heading = PE::SBasicSensor(90.0,0.3);//90deg
   PE::SBasicSensor angSpeedInvalid;
   PE::SBasicSensor speedInvalid;
   PE::CFusionSensor fusion = PE::CFusionSensor(10.0, pos, heading,angSpeedInvalid,speedInvalid);

   EXPECT_TRUE(fusion.GetPosition().IsValid());
   EXPECT_TRUE(fusion.GetHeading().IsValid());
   EXPECT_NEAR(10.0000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000000, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.000, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.300, fusion.GetHeading().Accuracy,0.001);

   //smaller timestamp - ignored
   fusion.AddPosition(9.9, PE::SPosition(50.0000000, 10.0000500, 1));
   fusion.DoComplexFusion();
   EXPECT_NEAR(10.0000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000000, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.000, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.300, fusion.GetHeading().Accuracy,0.001);

   //same timestamp
   fusion.AddPosition(10.0, PE::SPosition(50.0000000, 10.0001000, 1));
   fusion.DoComplexFusion();
   EXPECT_NEAR(10.0000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000500, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.000, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.300, fusion.GetHeading().Accuracy,0.001);

   //correct timestamp
   fusion.AddPosition(11.0, PE::SPosition(50.0000000, 10.0001500, 1));
   fusion.DoComplexFusion();
   EXPECT_NEAR(11.0000000, fusion.GetTimestamp(),0.0000001);
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0001166, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.333, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    89.999, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     1.147, fusion.GetHeading().Accuracy,0.001);
}


/**
 * Test position only fusion driving on a circle.
 */
TEST_F(PECFusionSensorTest, test_position_only_fusion_circle_driving )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(10.0, PE::SPosition(50.0, 10.0, 1), PE::SBasicSensor( 90.0, 0.10 ),PE::SBasicSensor(),PE::SBasicSensor());

   fusion.AddPosition(11.0, PE::SPosition(50.0000039, 10.0000696, 1));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0000026, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000464, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.333, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    89.943, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.395, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(12.0, PE::SPosition(50.0000155, 10.0001370, 1));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0000127, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0001275, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.571, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    89.193, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     1.668, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(13.0, PE::SPosition(50.0000345, 10.0002004, 1));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0000310, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0001997, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.675, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    85.523, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     5.019, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(14.0, PE::SPosition(50.0000602, 10.0002576, 1));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0000567, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0002599, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.723, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    74.621, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(    11.559, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(15.0, PE::SPosition(50.0000920, 10.0003070, 1));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0000894, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003097, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.757, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    55.774, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(    19.310, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(16.0, PE::SPosition(50.0001288, 10.0003471, 1));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0001278, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003487, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.821, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    36.637, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(    25.102, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(17.0, PE::SPosition(50.0001695, 10.0003766, 1));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0001694, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003768, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.932, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    23.144, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(    29.243, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(18.0, PE::SPosition(50.0002128, 10.0003947, 1));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0002100, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003935, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.872, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    13.887, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(    32.657, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(19.0, PE::SPosition(50.0002576, 10.0004008, 1));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0002545, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0004003, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.871, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(     5.258, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(    31.966, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(20.0, PE::SPosition(50.0003030, 10.0003970, 1));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0002999, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003972, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.872, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(   357.286, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(    31.919, fusion.GetHeading().Accuracy,0.001);
}

/**
 * Test position only fusion straight driving.
 */
TEST_F(PECFusionSensorTest, test_position_only_fusion_straight_driving )
{
   PE::SPosition    prevPos  = PE::SPosition(50.0, 10.0, 1);
   PE::SBasicSensor prevHead = PE::SBasicSensor( 90.0, 0.10);
   PE::TValue       distance = 5; //5[m]

   PE::CFusionSensor fusion = PE::CFusionSensor(10.0, prevPos, prevHead,PE::SBasicSensor(), PE::SBasicSensor());

   fusion.AddPosition(11.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoComplexFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0000466, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.333, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(     0.395, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(12.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoComplexFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0001115, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.571, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(     1.688, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(13.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoComplexFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0001795, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.675, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(     5.138, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(14.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoComplexFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0002488, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.723, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    11.820, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(15.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoComplexFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0003185, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.757, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    19.637, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(16.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoComplexFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0003884, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.823, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    25.524, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(17.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoComplexFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0004584, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.936, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    29.508, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(18.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoComplexFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0005238, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.872, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    32.697, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(19.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoComplexFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0005893, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.871, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    33.249, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(20.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoComplexFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0006548, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.871, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    33.612, prevHead.Accuracy,0.001);

   prevPos.HorizontalAcc = 1;
   fusion.AddPosition(21.0, PE::TOOLS::ToPosition(prevPos, distance, prevHead.Value));
   fusion.DoComplexFusion();
   prevPos  = fusion.GetPosition();
   prevHead = fusion.GetHeading();
   EXPECT_NEAR(50.0000000, prevPos.Latitude,0.0000001);
   EXPECT_NEAR(10.0007202, prevPos.Longitude,0.0000001);
   EXPECT_NEAR(     1.870, prevPos.HorizontalAcc,0.001);
   EXPECT_NEAR(    90.000, prevHead.Value,0.001);
   EXPECT_NEAR(    33.743, prevHead.Accuracy,0.001);
}

/**
 * Test position and heading fusion driving on a circle.
 */
TEST_F(PECFusionSensorTest, test_position_heading_fusion_circle_driving )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(10.0, PE::SPosition(50.0, 10.0, 1), PE::SBasicSensor( 90.0, 0.10 ),PE::SBasicSensor(),PE::SBasicSensor());

   fusion.AddPosition(11.0, PE::SPosition(50.0000039, 10.0000696, 1));
   fusion.AddHeading (11.0, PE::SBasicSensor( 75.0, 0.10 ));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0000026, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000464, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.333, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    78.015, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.159, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(12.0, PE::SPosition(50.0000155, 10.0001370, 1));
   fusion.AddHeading (12.0, PE::SBasicSensor( 65.0, 0.10 ));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0000140, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0001273, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.571, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    65.960, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.185, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(13.0, PE::SPosition(50.0000345, 10.0002004, 1));
   fusion.AddHeading (13.0, PE::SBasicSensor( 55.0, 0.10 ));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0000341, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0001987, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.675, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    55.424, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.192, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(14.0, PE::SPosition(50.0000602, 10.0002576, 1));
   fusion.AddHeading (14.0, PE::SBasicSensor( 45.0, 0.10 ));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0000603, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0002580, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.720, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    45.220, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.195, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(15.0, PE::SPosition(50.0000920, 10.0003070, 1));
   fusion.AddHeading (15.0, PE::SBasicSensor( 35.0, 0.10 ));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0000921, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003073, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.739, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    35.127, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.197, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(16.0, PE::SPosition(50.0001288, 10.0003471, 1));
   fusion.AddHeading (16.0, PE::SBasicSensor( 25.0, 0.10 ));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0001288, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003472, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.749, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    25.083, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.198, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(17.0, PE::SPosition(50.0001695, 10.0003766, 1));
   fusion.AddHeading (17.0, PE::SBasicSensor( 15.0, 0.10 ));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0001695, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003766, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.759, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    15.062, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.198, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(18.0, PE::SPosition(50.0002128, 10.0003947, 1));
   fusion.AddHeading (18.0, PE::SBasicSensor( 5.0, 0.10 ));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0002128, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003946, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.773, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(     5.051, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.199, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(19.0, PE::SPosition(50.0002576, 10.0004008, 1));
   fusion.AddHeading (19.0, PE::SBasicSensor( 0.0, 0.10 ));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0002575, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0004007, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.789, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(     0.023, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.199, fusion.GetHeading().Accuracy,0.001);

   fusion.AddPosition(20.0, PE::SPosition(50.0003030, 10.0003970, 1));
   fusion.AddHeading (20.0, PE::SBasicSensor( 355.0, 0.10 ));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0003029, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0003973, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.803, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(   355.014, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.199, fusion.GetHeading().Accuracy,0.001);
}

/**
 * Test invariance between order of adding position and heading.
 */
TEST_F(PECFusionSensorTest, test_position_heading_fusion_invariance )
{
   PE::CFusionSensor fusion = PE::CFusionSensor(10.0, PE::SPosition(50.0, 10.0, 1), PE::SBasicSensor( 90.0, 0.10 ),PE::SBasicSensor(),PE::SBasicSensor());

   fusion.AddPosition(11.0, PE::SPosition(50.0000039, 10.0000696, 1));
   fusion.AddHeading (11.0, PE::SBasicSensor( 85.0, 0.10 ));
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0000026, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000464, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.333, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    85.997, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.159, fusion.GetHeading().Accuracy,0.001);

   PE::CFusionSensor fusion2 = PE::CFusionSensor(10.0, PE::SPosition(50.0, 10.0, 1), PE::SBasicSensor( 90.0, 0.10 ),PE::SBasicSensor(),PE::SBasicSensor());

   fusion2.AddHeading (11.0, PE::SBasicSensor( 85.0, 0.10 ));
   fusion2.AddPosition(11.0, PE::SPosition(50.0000039, 10.0000696, 1));
   fusion2.DoComplexFusion();
   EXPECT_NEAR(50.0000026, fusion2.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000464, fusion2.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     1.333, fusion2.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(    85.997, fusion2.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.159, fusion2.GetHeading().Accuracy,0.001);
}

//test adding several same sensors inside one fusion
/**
 * test several speeds and angular speeds inside one fusion 
 */
TEST_F(PECFusionSensorTest, test_several_speed_ang_speed_in_one_fusion )
{
   PE::SPosition pos = PE::SPosition(50.0,10.0,1.0);//lat=50 lon=10
   PE::SBasicSensor heading = PE::SBasicSensor(90.0,5.0);//90deg
   PE::SBasicSensor speed = PE::SBasicSensor(3.08625,0.1); //3,08m/s
   PE::SBasicSensor angSpeed = PE::SBasicSensor(10.0,0.1);//turn left 10deg/s

   //PE::CFusionSensor fusion = PE::CFusionSensor(10.0, pos, heading, speed, angSpeed);
   PE::CFusionSensor fusion = PE::CFusionSensor(10.0, pos, heading,PE::SBasicSensor(0.0,0.1),PE::SBasicSensor(0.0,0.1));

   fusion.AddSpeed(11.0, speed);
   fusion.AddAngSpeed(11.0, angSpeed);
   fusion.DoSimpleFusion();
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000000, fusion.GetPosition().Longitude,0.0000001);


   /*

   fusion.AddSpeed(12.0, speed);
   fusion.AddAngSpeed(12.0, angSpeed);
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000000, fusion.GetPosition().Longitude,0.0000001);

   fusion.AddSpeed(13.0, speed);
   fusion.AddAngSpeed(13.0, angSpeed);
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000000, fusion.GetPosition().Longitude,0.0000001);

   fusion.AddSpeed(14.0, speed);
   fusion.AddAngSpeed(14.0, angSpeed);
   fusion.DoComplexFusion();
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000000, fusion.GetPosition().Longitude,0.0000001);

   fusion.AddSpeed(15.0, speed);
   fusion.AddAngSpeed(15.0, angSpeed);
   fusion.AddSpeed(16.0, speed);
   fusion.AddAngSpeed(16.0, angSpeed);
   fusion.AddSpeed(17.0, speed);
   fusion.AddAngSpeed(17.0, angSpeed);
   fusion.AddSpeed(18.0, speed);
   fusion.AddAngSpeed(18.0, angSpeed);
   fusion.AddSpeed(19.0, speed);
   fusion.AddAngSpeed(19.0, angSpeed);
   fusion.AddSpeed(20.0, speed);
   fusion.AddAngSpeed(20.0, angSpeed);
   fusion.DoComplexFusion();
   //fusion.DoSimpleFusion();
   EXPECT_NEAR(50.0000000, fusion.GetPosition().Latitude,0.0000001);
   EXPECT_NEAR(10.0000000, fusion.GetPosition().Longitude,0.0000001);
   EXPECT_NEAR(     0.000, fusion.GetPosition().HorizontalAcc,0.001);
   EXPECT_NEAR(     0.000, fusion.GetHeading().Value,0.001);
   EXPECT_NEAR(     0.000, fusion.GetHeading().Accuracy,0.001);
   EXPECT_NEAR(     0.000, fusion.GetSpeed().Value,0.001);
   EXPECT_NEAR(     0.000, fusion.GetSpeed().Accuracy,0.001);
   EXPECT_NEAR(     0.000, fusion.GetAngSpeed().Value,0.001);
   EXPECT_NEAR(     0.000, fusion.GetAngSpeed().Accuracy,0.001);
   */
}



int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

