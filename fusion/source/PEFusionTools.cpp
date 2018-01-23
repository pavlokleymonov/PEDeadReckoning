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

#include "PEFusionTools.h"
#include "PETools.h"
#include <math.h>

using namespace PE;


TValue ToAngDistance(const TValue& firstHeading, const TValue& secondHeading)
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


SBasicSensor PE::FUSION::PredictSensorAccuracy(const TTimestamp& timestampFirst, const SBasicSensor& sensor, const TTimestamp& timestampLast)
{
   SBasicSensor resultSensor = sensor;
   if ( timestampLast > timestampFirst && 
        sensor.IsValid())
   {
      resultSensor.Accuracy = resultSensor.Accuracy + resultSensor.Accuracy*(timestampLast-timestampFirst);
   }
   return resultSensor;
}


SBasicSensor PE::FUSION::PredictHeading(const TTimestamp& timestampFirst, const SBasicSensor& heading, const TTimestamp& timestampLast, const SBasicSensor& angSpeed)
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


SBasicSensor PE::FUSION::PredictHeading(const SPosition& positionFirst, const SPosition& positionLast)
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


SPosition PE::FUSION::PredictPosition(const TTimestamp& timestampFirst, const SBasicSensor& headingFirst, const TTimestamp& timestampLast, const SBasicSensor& headingLast, const SPosition& position, const SBasicSensor& speed)
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


SBasicSensor PE::FUSION::PredictSpeed(const TTimestamp& timestampFirst, const SPosition& positionFirst, const TTimestamp& timestampLast, const SPosition& positionLast)
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


SBasicSensor PE::FUSION::PredictAngSpeed(const TTimestamp& timestampFirst, const SBasicSensor& headingFirst, const TTimestamp& timestampLast, const SBasicSensor& headingLast)
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


SBasicSensor PE::FUSION::MergeSensor(const SBasicSensor& sen1, const SBasicSensor& sen2)
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


SBasicSensor PE::FUSION::MergeHeading(const SBasicSensor& head1, const SBasicSensor& head2)
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


SPosition PE::FUSION::MergePosition(const SPosition& pos1, const SPosition& pos2)
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


