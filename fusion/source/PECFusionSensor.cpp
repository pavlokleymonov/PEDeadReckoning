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


using namespace PE;

PE::CFusionSensor::CFusionSensor(const SPosition& position, const SBasicSensor& heading)
: m_Timestamp(PE::MAX_TIMESTAMP)
, m_Position(position)
, m_Heading(heading)
, m_AngSpeed()
, m_Speed()
{
}

PE::CFusionSensor::AddPosition(const TTimestamp& timestamp, const SPosition& position, const SBasicSensor& heading)
{
   if ( PE::MAX_TIMESTAMP != timestamp )
   {
      if ( position.IsValid() )
      {
         if ( m_Position.IsValid() )
         {
            SPosition newPos = GetPosition()
         }
         m_Position = FusionPosition()
      }
   }
}

TTimestamp PE::CFusionSensor::GetDeltaTime(const TTimestamp& timestamp) const
{
   return timestamp > m_Timestamp ? timestamp-m_Timestamp : 0;
}

SBasicSensor PE::CFusionSensor::GetHeading(const TTimestamp& deltaTime) const
{
   return SBasicSensor( 
      TOOLS::ToHeading(m_Heading.Value, deltaTime, m_AngSpeed.Value),
      m_Heading.Accuracy + m_AngSpeed.Accuracy * deltaTime);
}

SPosition PE::CFusionSensor::GetPosition(const TTimestamp& deltaTime, const SBasicSensor& heading) const
{
   TValue     distance    = m_Speed.Value * deltaTime;
   TValue     head        = heading.Value;
   SPosition  position    = TOOLS::ToPosition(m_Position, distance, heading);
   position.HorizontalAcc = m_Position.HorizontalAcc  + m_Speed.Accuracy * deltaTime;
   return position;
}

