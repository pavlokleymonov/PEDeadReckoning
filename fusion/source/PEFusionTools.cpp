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

/*
SBasicSensor PE::FUSION::GetSensor(const TTimestamp& deltaTimestamp, const SBasicSensor& sensor)
{
   SBasicSensor resultSensor = sensor;
   if ( EPSILON < deltaTimestamp && sensor.IsValid() )
   {
      resultSensor.Accuracy = sensor.Accuracy * (1 + deltaTimestamp);
   }
   return resultSensor;
}


SBasicSensor PE::FUSION::GetDistanceOrAngle(const TTimestamp& deltaTimestamp, const SBasicSensor& speed)
{
   SBasicSensor distance;
   TValue value = EPSILON;
   TValue accuracy = EPSILON;
   if ( EPSILON < deltaTimestamp && speed.IsValid() && EPSILON < speed.Value )
   {
      value = speed.Value * deltaTimestamp;
      accuracy = speed.Accuracy * deltaTimestamp;
      if ( EPSILON < accuracy && accuracy < value )
      {
         distance.Value = value;
         distance.Accuracy = accuracy;
      }
   }
   return distance;
}


SBasicSensor PE::FUSION::GetHeading(const SBasicSensor& firstHeading, const SBasicSensor& angle)
{
   SBasicSensor lastHeading = firstHeading;
   if ( firstHeading.IsValid() && angle.IsValid() )
   {
      lastHeading.Value = TOOLS::ToHeading(firstHeading.Value, angle.Value);
      lastHeading.Accuracy = firstHeading.Accuracy + angle.Accuracy;
   }
   return lastHeading;
}


SBasicSensor PE::FUSION::GetHeading(const SBasicSensor& firstHeading, const SPosition& firstPos, const SPosition& lastPos)
{
   SBasicSensor lastHeading;
   TValue distance = EPSILON;
   TValue accuracy = MAX_VALUE;
   TValue angle    = EPSILON;
   if ( firstPos.IsValid() && lastPos.IsValid() )
   {
      distance = TOOLS::ToDistance(firstPos.Latitude, firstPos.Longitude,lastPos.Latitude, lastPos.Longitude);
      accuracy = firstPos.HorizontalAcc + lastPos.HorizontalAcc;
      if ( EPSILON < accuracy && accuracy < distance )
      {
         lastHeading.Value    = TOOLS::ToHeading(firstPos.Latitude, firstPos.Longitude, lastPos.Latitude, lastPos.Longitude);
         lastHeading.Accuracy = TOOLS::ToDegrees(atan( accuracy / distance ));
         if ( firstHeading.IsValid() && firstHeading.Accuracy < lastHeading.Accuracy )
         {
            angle = TOOLS::ToAngle(firstHeading.Value, lastHeading.Value) * 2;  //real heading two times bigger then delta between position heading and start heading
            distance = TOOLS::ToDistance(firstHeading.Value, firstPos.Latitude, firstPos.Longitude,lastPos.Latitude, lastPos.Longitude); //distance with considering turning driving
            accuracy = lastPos.HorizontalAcc;
            lastHeading.Value = TOOLS::ToHeading(firstHeading.Value, angle );
            lastHeading.Accuracy = TOOLS::ToDegrees(atan( accuracy / distance ));
         }
      }
   }
   return lastHeading;
}


SPosition PE::FUSION::GetPosition(const SBasicSensor& heading, const SPosition& pos, const SBasicSensor& distance, const SBasicSensor& angle)
{
   SPosition lastPos = pos;
   if ( pos.IsValid() && heading.IsValid() && distance.IsValid() )
   {
      //if ( EPSILON < distance.Value ) //if distance and heading will have own classes derived from SBasicSensor and returns false if out of accuracy range limits
      if ( EPSILON < distance.Value && distance.Accuracy < distance.Value && 45 > heading.Accuracy )
      {
         std::pair<TValue, TValue> latlon = PE::TOOLS::ToPosition(pos.Latitude, pos.Longitude, distance.Value, heading.Value); //TODO added here angle into pos calculation
         lastPos.Latitude = latlon.first;
         lastPos.Longitude = latlon.second;
         TValue d = distance.Value + distance.Accuracy;
         TValue q = PE::TOOLS::ToRadians(heading.Accuracy);
         lastPos.HorizontalAcc = 2 * d * sin(q / 2);
         
         ///CONSIDERING angle
      }
   
   }
   return lastPos;
}

*/
SBasicSensor PE::FUSION::PredictSensorAccuracy(const TTimestamp& deltaTimestamp, const SBasicSensor& sensor)
{
   SBasicSensor resultSensor = sensor;
   if ( 0 < deltaTimestamp && sensor.IsValid() )
   {
      resultSensor.Accuracy = sensor.Accuracy * (1 + deltaTimestamp);
   }
   return resultSensor;
}


SBasicSensor PE::FUSION::PredictHeading(const TTimestamp& deltaTimestamp, const SBasicSensor& heading, const SBasicSensor& angSpeed)
{
   SBasicSensor resultHeading = heading;
   if ( 0 < deltaTimestamp && heading.IsValid() )
   {
      resultHeading.Accuracy = heading.Accuracy * (1 + deltaTimestamp);
      if ( angSpeed.IsValid() )
      {
         resultHeading.Value    = TOOLS::ToHeading(heading.Value, angSpeed.Value * deltaTimestamp);
         resultHeading.Accuracy = heading.Accuracy + angSpeed.Accuracy * deltaTimestamp;
      }
   }
   return resultHeading;
}

//TODO has to be reworked!!!!
SBasicSensor PE::FUSION::PredictHeading(const TTimestamp& deltaTimestamp, const SPosition& positionFirst, const SPosition& positionLast, const SBasicSensor& heading)
{
   SBasicSensor resultHeading = PredictSensorAccuracy(deltaTimestamp, heading);
   if ( 0 < deltaTimestamp && positionFirst.IsValid() && positionLast.IsValid() )
   {
      TValue distance        = TOOLS::ToDistance(positionFirst.Latitude, positionFirst.Longitude, positionLast.Latitude, positionLast.Longitude);
      if ( 0.0 < distance )
      {
         TValue deviation       = positionFirst.HorizontalAcc + positionLast.HorizontalAcc;
         resultHeading.Value    = TOOLS::ToHeading(positionFirst.Latitude, positionFirst.Longitude, positionLast.Latitude, positionLast.Longitude);
         resultHeading.Accuracy = TOOLS::ToDegrees(atan(deviation / distance)) / 2 * deltaTimestamp;
         if (heading.IsValid())
         {
            TValue omega           = TOOLS::ToAngle(heading.Value, resultHeading.Value) * 2;
            resultHeading.Value    = TOOLS::ToHeading(heading.Value,omega);
            resultHeading.Accuracy += heading.Accuracy;
         }
      }
   }
   return resultHeading;
}

//TODO has to be reworked!!!!
SPosition PE::FUSION::PredictPosition(const TTimestamp& deltaTimestamp, const SBasicSensor& heading, const SBasicSensor& angSpeed, const SPosition& position, const SBasicSensor& speed)
{
   SPosition resultPosition = position;
   if ( 0 < deltaTimestamp && position.IsValid() )
   {
      TValue posAccuracy = position.HorizontalAcc * (1 + deltaTimestamp);
      if ( speed.IsValid() )
      {
         posAccuracy = position.HorizontalAcc + speed.Value * deltaTimestamp + speed.Accuracy * deltaTimestamp;
         if ( heading.IsValid() && angSpeed.IsValid() )
         {
            TValue fi  = heading.Accuracy + (angSpeed.Accuracy * deltaTimestamp);
            if ( 90 > fi )
            {
               TValue horda_heading = TOOLS::ToHeading(heading.Value, angSpeed.Value / 2 * deltaTimestamp);
               TValue omega         = TOOLS::ToRadians(fabs(angSpeed.Value / 2 * deltaTimestamp));
               TValue arch          = speed.Value * deltaTimestamp;
               TValue horda         = arch * ( 0 < omega ? sin(omega) / omega : 1 );
               resultPosition       = TOOLS::ToPosition(position, horda, horda_heading);
               posAccuracy          = (position.HorizontalAcc + speed.Accuracy * deltaTimestamp) / cos( TOOLS::ToRadians(fi) );
            }
         }
      }
      resultPosition.HorizontalAcc = posAccuracy;
   }
   return resultPosition;
}


SBasicSensor PE::FUSION::PredictSpeed(const TTimestamp& deltaTimestamp, const SPosition& positionFirst, const SPosition& positionLast, const SBasicSensor& angSpeed)
{
   SBasicSensor resutlSpeed;
   if ( 0 < deltaTimestamp && positionFirst.IsValid() && positionLast.IsValid() )
   {
      TValue horda         = TOOLS::ToDistance(positionFirst.Latitude,positionFirst.Longitude,positionLast.Latitude,positionLast.Longitude);
      resutlSpeed.Value    = horda / deltaTimestamp ;
      resutlSpeed.Accuracy = (positionFirst.HorizontalAcc + positionLast.HorizontalAcc) / cos(TOOLS::ToRadians(45));
      if (  0.0 < horda && angSpeed.IsValid() )
      {
         TValue fi       = TOOLS::ToRadians(fabs(angSpeed.Accuracy * deltaTimestamp));
         if ( TOOLS::ToRadians(45) > fi )
         {
            TValue omega    = TOOLS::ToRadians(fabs(angSpeed.Value / 2 * deltaTimestamp));
            if ( TOOLS::ToRadians(90) > omega )
            {
               TValue arch          = horda * ( 0 < omega ? omega / sin(omega) : 1);
               resutlSpeed.Value    = arch / deltaTimestamp;
               resutlSpeed.Accuracy = (positionFirst.HorizontalAcc + positionLast.HorizontalAcc) / cos( fi );
            }
         }
      }
   }
   return resutlSpeed;
}


SBasicSensor PE::FUSION::PredictAngSpeed(const TTimestamp& deltaTimestamp, const SBasicSensor& headingFirst, const SBasicSensor& headingLast)
{
   SBasicSensor resultAngSpeed;
   if ( 0 < deltaTimestamp && headingFirst.IsValid() && headingLast.IsValid() )
   {
      resultAngSpeed.Value    = TOOLS::ToAngle(headingFirst.Value, headingLast.Value) / deltaTimestamp;
      resultAngSpeed.Accuracy = headingFirst.Accuracy + headingLast.Accuracy;
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

   TValue amplify = sen1.Accuracy + sen2.Accuracy;
   TAccuracy acc1 = sen1.Accuracy * amplify / sen2.Accuracy;
   TAccuracy acc2 = sen2.Accuracy * amplify / sen1.Accuracy;

   TValue val = KalmanFilter(sen1.Value, acc1, sen2.Value, acc2);
   TAccuracy acc = KalmanFilter(sen1.Accuracy, acc1, sen2.Accuracy, acc2);
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

   TValue amplify = pos1.HorizontalAcc + pos2.HorizontalAcc;
   TAccuracy acc1 = pos1.HorizontalAcc * amplify / pos2.HorizontalAcc;
   TAccuracy acc2 = pos2.HorizontalAcc * amplify / pos1.HorizontalAcc;
   TValue lat = KalmanFilter(pos1.Latitude, acc1, pos2.Latitude, acc2);
   TValue lon = KalmanFilter(lon1, acc1, lon2, acc2);
   TAccuracy horizontalAcc = KalmanFilter(pos1.HorizontalAcc, acc1, pos2.HorizontalAcc, acc2);

   if ( 180.0 <= lon )
   {
      lon -= 360.0;
   }

   return SPosition(lat,lon,horizontalAcc);
}


TValue PE::FUSION::KalmanFilter(const TValue& value1, const TAccuracy& accuracy1, const TValue& value2, const TAccuracy& accuracy2)
{
   TAccuracy   K = accuracy1 + accuracy2;
   return (value1 * (K - accuracy1) + value2 * (K - accuracy2)) / K;
}
