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
   SPosition     newPos = MergePosition(
                             PredictPosition(m_Timestamp, m_Position, m_Heading, timestamp, m_Speed,m_AngSpeed),
                             position
                          );
   SBasicSensor newHead = MergeHeading(
                             PredictHeading(m_Timestamp, m_Heading, timestamp, m_AngSpeed),
                             heading
                          );

   m_Speed     = PredictSpeed(m_Timestamp, m_Position, timestamp, newPos);
   m_AngSpeed  = PredictAngSpeed(m_Timestamp, m_Heading, timestamp, newHead);
   m_Heading   = newHead;
   m_Position  = newPos;
   m_Timestamp = timestamp;
}


void PE::CFusionSensor::AddVelocity(const TTimestamp& timestamp, const SBasicSensor& speed)
{
   AddPosition(
      timestamp,
      PredictPosition(m_Timestamp, m_Position, m_Heading, timestamp,speed,m_AngSpeed),
      PredictHeading(m_Timestamp, m_Heading, timestamp, m_AngSpeed)
   );
}


void PE::CFusionSensor::AddAngularVelocity(const TTimestamp& timestamp, const SBasicSensor& angSpeed)
{
   AddPosition(
      timestamp,
      PredictPosition(m_Timestamp, m_Position, m_Heading, timestamp,m_Speed,angSpeed),
      PredictHeading(m_Timestamp, m_Heading, timestamp, angSpeed)
   );
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
