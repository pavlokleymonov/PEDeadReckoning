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

#include "PECFusionSensor.h"
#include "PETools.h"
#include <math.h>



using namespace PE;

PE::CFusionSensor::CFusionSensor(const TTimestamp& timestamp, const SPosition& position, const SBasicSensor& heading)
: m_Timestamp(timestamp)
, m_Position(position)
, m_Heading(heading)
, m_AngSpeed() //invalid 
, m_Speed() //invalid
{
}


PE::CFusionSensor::~CFusionSensor()
{
}


void PE::CFusionSensor::AddPosition(const TTimestamp& timestamp, const SPosition& position, const SBasicSensor& heading)
{
   m_Speed     = MergeSensor(
                    PredictSensorAccuracy(m_Timestamp, m_Speed, timestamp),
                    PredictSpeed(m_Timestamp, m_Position, timestamp, position)
                 );
   m_AngSpeed  = MergeSensor(
                    PredictSensorAccuracy(m_Timestamp, m_AngSpeed, timestamp),
                    PredictAngSpeed(m_Timestamp, m_Heading, timestamp, heading)
                 );
   m_Position  = MergePosition(
                    PredictPosition(m_Timestamp, m_Position, m_Heading, timestamp, m_Speed),
                    position
                 );
   m_Heading   = MergeSensor(
                    PredictHeading(m_Timestamp, m_Heading, timestamp, m_AngSpeed),
                    heading
                 );
   m_Timestamp = timestamp;
}


void PE::CFusionSensor::AddVelocity(const TTimestamp& timestamp, const SBasicSensor& speed)
{
   m_Speed     = MergeSensor(
                    PredictSensorAccuracy(m_Timestamp, m_Speed, timestamp),
                    speed
                 );
   m_AngSpeed  = PredictSensorAccuracy(m_Timestamp, m_AngSpeed, timestamp);
   m_Position  = PredictPosition(m_Timestamp, m_Position, m_Heading, timestamp, m_Speed);
   m_Heading   = PredictHeading(m_Timestamp, m_Heading, timestamp, m_AngSpeed);
   m_Timestamp = timestamp;
}


void PE::CFusionSensor::AddAngularVelocity(const TTimestamp& timestamp, const SBasicSensor& angSpeed)
{
   m_Speed     = PredictSensorAccuracy(m_Timestamp, m_Speed, timestamp);
   m_AngSpeed  = MergeSensor(
                    PredictSensorAccuracy(m_Timestamp, m_AngSpeed, timestamp),
                    angSpeed
                 );
   m_Position  = PredictPosition(m_Timestamp, m_Position, m_Heading, timestamp, m_Speed);
   m_Heading   = PredictHeading(m_Timestamp, m_Heading, timestamp, m_AngSpeed);
   m_Timestamp = timestamp;
}


const TTimestamp& PE::CFusionSensor::GetTimestamp() const
{
   return m_Timestamp;
}


const SBasicSensor& PE::CFusionSensor::GetHeading() const
{
   return m_Heading;
}


const SPosition& PE::CFusionSensor::GetPosition() const
{
   return m_Position;
}


SBasicSensor PE::CFusionSensor::PredictSensorAccuracy(const TTimestamp& timestampFirst, const SBasicSensor& sensor, const TTimestamp& timestampLast)
{
   SBasicSensor resultSensor = sensor;
   if ( timestampLast > timestampFirst && 
        sensor.IsValid())
   {
      resultSensor.Accuracy = resultSensor.Accuracy + resultSensor.Accuracy*(timestampLast-timestampFirst);
   }
   return resultSensor;
}


SBasicSensor PE::CFusionSensor::PredictHeading(const TTimestamp& timestampFirst, const SBasicSensor& heading, const TTimestamp& timestampLast, const SBasicSensor& angSpeed)
{
   SBasicSensor resultHeading = heading;
   if ( timestampLast > timestampFirst && 
        heading.IsValid() && 
        angSpeed.IsValid())
   {
      TValue deltaTS = timestampLast - timestampFirst;
      resultHeading.Value    = TOOLS::ToHeading(heading.Value, deltaTS, angSpeed.Value);
      resultHeading.Accuracy = resultHeading.Accuracy + angSpeed.Accuracy*(deltaTS);
   }
   return resultHeading;
}


SPosition PE::CFusionSensor::PredictPosition(const TTimestamp& timestampFirst, const SPosition& position, const SBasicSensor& heading, const TTimestamp& timestampLast, const SBasicSensor& speed)
{
   SPosition resultPosition = position;
   if ( timestampLast > timestampFirst && 
        position.IsValid() &&
         heading.IsValid() &&
           speed.IsValid())
   {
      TValue deltaTS              = timestampLast - timestampFirst;
      TValue distance             = speed.Value * deltaTS;
      TValue additionlaInAccuracy = speed.Accuracy * deltaTS / cos(TOOLS::ToRadians(heading.Accuracy));
      resultPosition              = TOOLS::ToPosition(position, distance, heading.Value);
      resultPosition.HorizontalAcc= position.HorizontalAcc + additionlaInAccuracy;
   }
   return resultPosition;
}


SBasicSensor PE::CFusionSensor::PredictSpeed(const TTimestamp& timestampFirst, const SPosition& positionFirst, const TTimestamp& timestampLast, const SPosition& positionLast)
{
   SBasicSensor resutlSpeed;
   if ( timestampLast > timestampFirst &&
        positionFirst.IsValid() &&
        positionLast.IsValid())
   {
      TValue deltaTS       = timestampLast - timestampFirst;
      TValue distance      = TOOLS::ToDistancePrecise(positionFirst,positionLast);
      resutlSpeed.Value    = distance / deltaTS;
      resutlSpeed.Accuracy = (positionFirst.HorizontalAcc + positionLast.HorizontalAcc) / deltaTS;
   }
   return resutlSpeed;
}


SBasicSensor PE::CFusionSensor::PredictAngSpeed(const TTimestamp& timestampFirst, const SBasicSensor& headingFirst, const TTimestamp& timestampLast, const SBasicSensor& headingLast)
{
   SBasicSensor resultAngSpeed;
   if ( timestampLast > timestampFirst &&
        headingFirst.IsValid() &&
        headingLast.IsValid())
   {
      TValue deltaTS          = timestampLast - timestampFirst;
      //=IF(AND(O11>270;O12<90);O11-360-O12;O11-O12)
      TValue deltaAng = (270 < headingFirst.Value && 90 > headingLast.Value) ? headingFirst.Value - 360 - headingLast.Value : headingFirst.Value - headingLast.Value;
      resultAngSpeed.Value    = deltaAng / deltaTS;
      resultAngSpeed.Accuracy = (headingFirst.Accuracy + headingLast.Accuracy) / deltaTS;
   }
   return resultAngSpeed;
}


SBasicSensor PE::CFusionSensor::MergeSensor(const SBasicSensor& sen1, const SBasicSensor& sen2)
{
   if ( false == sen1.IsValid() )
      return sen2;
   if ( false == sen2.IsValid() )
      return sen1;
   TAccuracy   K = sen1.Accuracy + sen2.Accuracy;
   TValue    val = (sen1.Value * ( K - sen1.Accuracy ) + sen2.Value * ( K - sen2.Accuracy )) / K;
   TAccuracy acc = (sen1.Accuracy * ( K - sen1.Accuracy ) + sen2.Accuracy * ( K - sen2.Accuracy )) / K;
   return SBasicSensor(val,acc);
}

SPosition PE::CFusionSensor::MergePosition(const SPosition& pos1, const SPosition& pos2)
{
   if ( false == pos1.IsValid() )
      return pos2;
   if ( false == pos2.IsValid() )
      return pos1;
   TAccuracy   K = pos1.HorizontalAcc + pos2.HorizontalAcc;
   TValue lat = (pos1.Latitude * ( K - pos1.HorizontalAcc ) + pos2.Latitude * ( K - pos2.HorizontalAcc )) / K;
   TValue lon = (pos1.Longitude * ( K - pos1.HorizontalAcc ) + pos2.Longitude * ( K - pos2.HorizontalAcc )) / K;
   TAccuracy horizontalAcc = (pos1.HorizontalAcc * ( K - pos1.HorizontalAcc ) + pos2.HorizontalAcc * ( K - pos2.HorizontalAcc )) / K;
   return SPosition(lat,lon,horizontalAcc);
}



