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

#include "PECPositionFilterSpeed.h"
#include "PETools.h"
#include <math.h>


using namespace PE;

PE::CPositionFilterSpeed::CPositionFilterSpeed(const TValue& speedLimit)
: IPositionFilter()
, mSpeedLimit(speedLimit)
{
}

void PE::CPositionFilterSpeed::AddPosition(const TTimestamp& timestamp, const SPosition& position)
{
   if ( 0 < mTimestamp && mPosition.IsValid() ) //internal position is valid
   {
      if ( mTimestamp < timestamp && position.IsValid() ) //new position is valid
      {
         TValue speed = PE::TOOLS::ToDistance(mPosition, position) / (timestamp - mTimestamp); //speed in m/s
         //printf("ts=%f oldts=%f speed=%.16f limit=%.16f\n",timestamp, mTimestamp, speed, m_Speed_limit);
         if ( speed > mSpeedLimit ) //speed is above speed limit
         {
            mPosition = position;
         }
         mTimestamp = timestamp;
      }
   }
   //incorrect internal position has to be updated
   else if ( 0 < timestamp && position.IsValid() )
   {
      mTimestamp = timestamp;
      mPosition = position;
   }
}

const TValue& PE::CPositionFilterSpeed::GetSpeedLimit() const
{
   return mSpeedLimit;
}


