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
#include "PEFusionTools.h"
#include "PETools.h"



using namespace PE;
using namespace PE::FUSION;


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
