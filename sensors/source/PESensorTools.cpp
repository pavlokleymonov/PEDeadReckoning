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

#include "PESensorTools.h"

using namespace PE;

TValue PE::Sensor::PredictValue( const TTimestamp& requestedTs, const TTimestamp& leftTs, const TTimestamp& rightTs, const TValue& leftValue, const TValue& rightValue )
{
//    TTimestamp deltaTs = rightTs - leftTs;
//    TTimestamp deltaRequestedTs = requestedTs - leftTs;
//    TValue deltaValue = rightValue - leftValue;
//    TValue predictValue = deltaValue * deltaRequestedTs / deltaTs  + leftValue;
//    return predictValue;
   return (rightValue - leftValue) * (requestedTs - leftTs) / (rightTs - leftTs) + leftValue;
}


bool PE::Sensor::IsIntervalOk( const TTimestamp& deltaTs, const TTimestamp& interval, const TTimestamp& hysteresis )
{
   if ( PE::EPSILON < deltaTs )
   {
      if ( deltaTs < (interval + hysteresis) )
      {
         if ( deltaTs > (interval - hysteresis) )
         {
            return true;
         }
      }
   }
   return false;
}


bool PE::Sensor::IsAccuracyOk( const TValue& value, const TAccuracy& accuracy, const TValue& ratio )
{
   if ( value > (accuracy * ratio) )
   {
      return true;
   }
   return false;
}


bool PE::Sensor::IsInRange( const TTimestamp& testedTS, const TTimestamp& beginTS, const TTimestamp& endTS )
{
   if ( beginTS <= testedTS )
   {
      if ( endTS >= testedTS )
      {
         return true;
      }
   }
   return false;
}
