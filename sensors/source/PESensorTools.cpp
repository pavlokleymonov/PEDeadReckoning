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

double PE::Sensor::PredictValue( const double& requestedTs, const double& leftTs, const double& rightTs, const double& leftValue, const double& rightValue )
{
//    double deltaTs = rightTs - leftTs;
//    double deltaRequestedTs = requestedTs - leftTs;
//    double deltaValue = rightValue - leftValue;
//    double predictValue = deltaValue * deltaRequestedTs / deltaTs  + leftValue;
//    return predictValue;
   return (rightValue - leftValue) * (requestedTs - leftTs) / (rightTs - leftTs) + leftValue;
}


bool PE::Sensor::IsIntervalOk( const double& deltaTs, const double& interval, const double& hysteresis )
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


bool PE::Sensor::IsAccuracyOk( const double& value, const double& accuracy, const double& ratio )
{
   if ( value > (accuracy * ratio) )
   {
      return true;
   }
   return false;
}


bool PE::Sensor::IsInRange( const double& testedTS, const double& beginTS, const double& endTS )
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
