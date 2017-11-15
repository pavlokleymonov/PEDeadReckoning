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




PE::TValue ToAngDistance(const PE::TValue& firstHeading, const PE::TValue& secondHeading)
{
   if   ( 180 < (firstHeading - secondHeading))
   {
      return firstHeading - (secondHeading+360.0);
   }
   else if ( -180 > (firstHeading - secondHeading))
   {
      return firstHeading+360.0 - secondHeading;
   }
   else
   {
      return firstHeading - secondHeading;
   }
}


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


void PE::CFusionSensor::AddPosition(const TTimestamp& timestamp, const SPosition& position, const SBasicSensor& heading, const SBasicSensor& speed)
{
   if ( m_Timestamp > timestamp )
   {
      return;
   }
   SBasicSensor posSpeed   = MergeSensor(
                                PredictSpeed(m_Timestamp, m_Position, timestamp, position),
                                speed
                             );
   SBasicSensor posHeading = MergeHeading(
                                PredictHeading(m_Position, position),
                                heading
                             );
   SBasicSensor startHeading = m_Heading;

   m_Speed     = MergeSensor(
                    PredictSensorAccuracy(m_Timestamp, m_Speed, timestamp),
                    posSpeed
                 );
   m_AngSpeed  = MergeSensor(
                    PredictSensorAccuracy(m_Timestamp, m_AngSpeed, timestamp),
                    PredictAngSpeed(m_Timestamp,m_Heading,timestamp,posHeading)
                 );
   m_Heading   = MergeHeading(
                    PredictHeading(m_Timestamp, m_Heading, timestamp, m_AngSpeed),
                    posHeading
                 );
   m_Position  = MergePosition(
                    PredictPosition(m_Timestamp, startHeading, timestamp, m_Heading, m_Position, m_Speed),
                    position
                 );
   m_Timestamp = timestamp;
}


void PE::CFusionSensor::AddSpeed(const TTimestamp& timestamp, const SBasicSensor& speed)
{
   if ( m_Timestamp > timestamp )
   {
      return;
   }
   SBasicSensor startHeading = m_Heading;
   m_Speed     = MergeSensor(
                    PredictSensorAccuracy(m_Timestamp, m_Speed, timestamp),
                    speed
                 );
   m_AngSpeed  = PredictSensorAccuracy(m_Timestamp, m_AngSpeed, timestamp);
   m_Heading   = PredictHeading(m_Timestamp, m_Heading, timestamp, m_AngSpeed);
   m_Position  = PredictPosition(m_Timestamp, startHeading, timestamp, m_Heading, m_Position, m_Speed);
   m_Timestamp = timestamp;
}


void PE::CFusionSensor::AddAngSpeed(const TTimestamp& timestamp, const SBasicSensor& angSpeed)
{
   if ( m_Timestamp > timestamp )
   {
      return;
   }
   SBasicSensor startHeading = m_Heading;
   m_Speed     = PredictSensorAccuracy(m_Timestamp, m_Speed, timestamp);
   m_AngSpeed  = MergeSensor(
                    PredictSensorAccuracy(m_Timestamp, m_AngSpeed, timestamp),
                    angSpeed
                 );
   m_Heading   = PredictHeading(m_Timestamp, m_Heading, timestamp, m_AngSpeed);
   m_Position  = PredictPosition(m_Timestamp, startHeading, timestamp, m_Heading, m_Position, m_Speed);
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


SBasicSensor PE::CFusionSensor::PredictHeading(const SPosition& positionFirst, const SPosition& positionLast)
{
   SBasicSensor resultHeading;
   if ( positionFirst.IsValid() && 
        positionLast.IsValid())
   {
      TValue distance        = TOOLS::ToDistancePrecise(positionFirst, positionLast);
      if ( 0.0 < distance )
      {
         TValue deviation       = positionFirst.HorizontalAcc + positionLast.HorizontalAcc;
         resultHeading.Value    = TOOLS::ToHeading(positionFirst, positionLast);
         resultHeading.Accuracy = TOOLS::ToDegrees(atan(deviation / distance)) / 2;
      }
   }
   return resultHeading;
}


SPosition PE::CFusionSensor::PredictPosition(const TTimestamp& timestampFirst, const SBasicSensor& headingFirst, const TTimestamp& timestampLast, const SBasicSensor& headingLast, const SPosition& position, const SBasicSensor& speed)
{
   SPosition resultPosition = position;

   if ( timestampLast > timestampFirst &&
          headingFirst.IsValid() &&
           headingLast.IsValid() &&
              position.IsValid() &&
                 speed.IsValid())
   {
      TValue deltaTS              = timestampLast - timestampFirst;
      TValue omega                = TOOLS::ToRadians(fabs(ToAngDistance(headingFirst.Value, headingLast.Value)));
      TValue arch                 = speed.Value * deltaTS;
      TValue horda                = arch * ( 0 < omega ? sin(omega) / omega : 1 );
      TValue additionlaInAccuracy = speed.Accuracy * deltaTS / cos(TOOLS::ToRadians(headingFirst.Accuracy + headingLast.Accuracy)/2);
      resultPosition              = TOOLS::ToPosition(position, horda, headingLast.Value);
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
      TValue deltaAng         = ToAngDistance(headingFirst.Value, headingLast.Value);
      resultAngSpeed.Value    = deltaAng / deltaTS;
      resultAngSpeed.Accuracy = (headingFirst.Accuracy + headingLast.Accuracy) / deltaTS;
   }
   return resultAngSpeed;
}


SBasicSensor PE::CFusionSensor::MergeSensor(const SBasicSensor& sen1, const SBasicSensor& sen2)
{
   if ( false == sen1.IsValid() )
   {
      return sen2;
   }
   if ( false == sen2.IsValid() )
   {
      return sen1;
   }
   TAccuracy   K = sen1.Accuracy + sen2.Accuracy;
   TValue    val = (sen1.Value * ( K - sen1.Accuracy ) + sen2.Value * ( K - sen2.Accuracy )) / K;
   TAccuracy acc = (sen1.Accuracy * ( K - sen1.Accuracy ) + sen2.Accuracy * ( K - sen2.Accuracy )) / K;
   return SBasicSensor(val,acc);
}


SBasicSensor PE::CFusionSensor::MergeHeading(const SBasicSensor& head1, const SBasicSensor& head2)
{
   SBasicSensor headExt1 = head1;
   SBasicSensor headExt2 = head2;

   if      ( 180 < (head1.Value - head2.Value) )
   {
      headExt2.Value += 360.0;
   }

   else if ( -180 > (head1.Value - head2.Value) )
   {
      headExt1.Value += 360.0;
   }
   
   SBasicSensor result = MergeSensor(headExt1, headExt2);

   if ( 360.0 <= result.Value )
   {
      result.Value -= 360.0;
   }
   
   return result;
}

SPosition PE::CFusionSensor::MergePosition(const SPosition& pos1, const SPosition& pos2)
{
   if ( false == pos1.IsValid() )
   {
      return pos2;
   }
   if ( false == pos2.IsValid() )
   {
      return pos1;
   }
   TValue lon1 = pos1.Longitude;
   TValue lon2 = pos2.Longitude;

   if      ( 180 < (lon1 - lon2) )
   {
      lon2 += 360.0;
   }

   else if ( -180 > (lon1 - lon2) )
   {
      lon1 += 360.0;
   }

   TAccuracy   K = pos1.HorizontalAcc + pos2.HorizontalAcc;
   TValue lat = (pos1.Latitude * ( K - pos1.HorizontalAcc ) + pos2.Latitude * ( K - pos2.HorizontalAcc )) / K;
   TValue lon = (lon1 * ( K - pos1.HorizontalAcc ) + lon2 * ( K - pos2.HorizontalAcc )) / K;
   TAccuracy horizontalAcc = (pos1.HorizontalAcc * ( K - pos1.HorizontalAcc ) + pos2.HorizontalAcc * ( K - pos2.HorizontalAcc )) / K;

   if ( 180.0 <= lon )
   {
      lon -= 360.0;
   }

   return SPosition(lat,lon,horizontalAcc);
}


