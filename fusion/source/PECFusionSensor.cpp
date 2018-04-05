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


PE::CFusionSensor::CFusionSensor(const TTimestamp& timestamp, const SPosition& position, const SBasicSensor& heading, const SBasicSensor& angSpeed, const SBasicSensor& speed)
: m_Timestamp(timestamp)
, m_Position(position)
, m_Heading(heading)
, m_AngSpeed(angSpeed)
, m_AngAcceleration(0)
, m_Speed(speed)
, m_LineAcceleration(0)
{
}


PE::CFusionSensor::~CFusionSensor()
{
}

void PE::CFusionSensor::AddPosition(const TTimestamp& timestamp, const SPosition& position)
{
   if ( m_SensorsList.empty() )
   {
      m_SensorsList.push_back(SSensorItem(timestamp, position, SBasicSensor(), SBasicSensor(), SBasicSensor()));
   }
   else if ( timestamp > m_SensorsList.back().timestamp )
   {
      m_SensorsList.push_back(SSensorItem(timestamp, position, SBasicSensor(), SBasicSensor(), SBasicSensor()));
   }
   else if ( timestamp == m_SensorsList.back().timestamp )
   {
      SPosition& oldPosition = m_SensorsList.back().position;
      oldPosition = MergePosition(oldPosition, position);
   }
}


void PE::CFusionSensor::AddHeading(const TTimestamp& timestamp, const SBasicSensor& heading)
{
   if ( m_SensorsList.empty() )
   {
      m_SensorsList.push_back(SSensorItem(timestamp, SPosition(), heading, SBasicSensor(), SBasicSensor()));
   }
   else if ( timestamp > m_SensorsList.back().timestamp )
   {
      m_SensorsList.push_back(SSensorItem(timestamp, SPosition(), heading, SBasicSensor(), SBasicSensor()));
   }
   else if ( timestamp == m_SensorsList.back().timestamp )
   {
      SBasicSensor& oldHeading = m_SensorsList.back().heading;
      oldHeading = MergeHeading(oldHeading, heading);
   }
}


void PE::CFusionSensor::AddSpeed(const TTimestamp& timestamp, const SBasicSensor& speed)
{
   if ( m_SensorsList.empty() )
   {
      m_SensorsList.push_back(SSensorItem(timestamp, SPosition(), SBasicSensor(), speed, SBasicSensor()));
   }
   else if ( timestamp > m_SensorsList.back().timestamp )
   {
      m_SensorsList.push_back(SSensorItem(timestamp, SPosition(), SBasicSensor(), speed, SBasicSensor()));
   }
   else if ( timestamp == m_SensorsList.back().timestamp )
   {
      SBasicSensor& oldSpeed = m_SensorsList.back().speed;
      oldSpeed = MergeSensor(oldSpeed, speed);
   }
}


void PE::CFusionSensor::AddAngSpeed(const TTimestamp& timestamp, const SBasicSensor& angSpeed)
{
   if ( m_SensorsList.empty() )
   {
      m_SensorsList.push_back(SSensorItem(timestamp, SPosition(), SBasicSensor(), SBasicSensor(), angSpeed));
   }
   else if ( timestamp > m_SensorsList.back().timestamp )
   {
      m_SensorsList.push_back(SSensorItem(timestamp, SPosition(), SBasicSensor(), SBasicSensor(), angSpeed));
   }
   else if ( timestamp == m_SensorsList.back().timestamp )
   {
      SBasicSensor& oldAngSpeed = m_SensorsList.back().angSpeed;
      oldAngSpeed = MergeSensor(oldAngSpeed, angSpeed);
   }
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


const SBasicSensor& PE::CFusionSensor::GetSpeed() const
{
   return m_Speed;
}


const SBasicSensor& PE::CFusionSensor::GetAngSpeed() const
{
   return m_AngSpeed;
}


void PE::CFusionSensor::DoOneItemFusion(const TTimestamp& timestamp, const SPosition& position, const SBasicSensor& heading, const SBasicSensor& speed, const SBasicSensor& angSpeed)
{
   if ( m_Timestamp == timestamp )
   {
      m_Position = MergePosition(position, m_Position);
      m_Heading  = MergeHeading(heading, m_Heading);
      m_Speed    = MergeSensor(speed, m_Speed);
      m_AngSpeed = MergeSensor(angSpeed, m_AngSpeed);
   }
   else if( m_Timestamp < timestamp )
   {
      TTimestamp deltaTimestamp = timestamp - m_Timestamp;

      SBasicSensor posHeading;
      SBasicSensor posAngSpeed;
      SBasicSensor posSpeed;

      if ( position.IsValid() )
      {
         TValue distPos = TOOLS::ToDistancePrecise(m_Position,position);
         TAccuracy accPos = m_Position.HorizontalAcc + position.HorizontalAcc;
         if (distPos > accPos)
         {
            posHeading  = PredictHeading(deltaTimestamp, m_Position, position, m_Heading);
            posAngSpeed = PredictAngSpeed(deltaTimestamp, m_Heading, posHeading);
            posSpeed    = PredictSpeed(deltaTimestamp, m_Position, position, posAngSpeed);
         }
      }

      if ( m_AngSpeed.IsValid() )
      {
         m_AngSpeed.Value += (m_AngAcceleration * deltaTimestamp);
      }
      SBasicSensor newAngSpeed = MergeSensor(
                     PredictSensorAccuracy(deltaTimestamp, m_AngSpeed),
                     angSpeed
                  );

      if ( m_Speed.IsValid() )
      {
         m_Speed.Value += (m_LineAcceleration * deltaTimestamp);
      }
      SBasicSensor newSpeed = MergeSensor(
                     PredictSensorAccuracy(deltaTimestamp, m_Speed),
                     speed
                  );

      SBasicSensor newHeading = MergeHeading(
                     PredictHeading(deltaTimestamp, m_Heading, newAngSpeed),
                     heading
                  );

      SPosition newPosition = MergePosition(
                     PredictPosition(deltaTimestamp, m_Heading, newAngSpeed, m_Position, newSpeed),
                     position
                  );

      m_Timestamp       = timestamp;

      newAngSpeed        = MergeSensor( newAngSpeed, posAngSpeed);
      if ( newAngSpeed.IsValid() && m_AngSpeed.IsValid() )
      {
         m_AngAcceleration  = (newAngSpeed.Value - m_AngSpeed.Value) / deltaTimestamp;
      }
      m_AngSpeed         = newAngSpeed;

      newSpeed           = MergeSensor( newSpeed, posSpeed);
      if ( newSpeed.IsValid() && m_Speed.IsValid() )
      {
         m_LineAcceleration = (newSpeed.Value - m_Speed.Value) / deltaTimestamp;
      }
      m_Speed            = newSpeed;

      m_Heading          = MergeHeading( newHeading, posHeading);

      m_Position         = newPosition;
   }
}


void PE::CFusionSensor::DoFusion()
{
   TSensorsList::const_iterator item = m_SensorsList.begin();
   while ( m_SensorsList.end() != item )
   {
      DoOneItemFusion(item->timestamp, item->position, item->heading, item->speed, item->angSpeed);
      ++item;
   }
   m_SensorsList.clear();
}


