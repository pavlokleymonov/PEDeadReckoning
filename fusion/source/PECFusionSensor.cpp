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

void PE::CFusionSensor::AddPosition(const TTimestamp& timestamp, const SPosition& position)
{
   if      ( m_Timestamp == timestamp )
   {
      m_Position = MergePosition( m_Position, position );
   }
   else if( m_Timestamp < timestamp )
   {
      TTimestamp deltaTimestamp = timestamp - m_Timestamp;

      SPosition newPosition = MergePosition(
                     PredictPosition(deltaTimestamp, m_Heading, m_AngSpeed, m_Position, m_Speed),
                     position
                  );

      SBasicSensor newHeading = MergeHeading(
                     PredictHeading(deltaTimestamp, m_Heading, m_AngSpeed),
                     PredictHeading(m_Position, newPosition)
                  );

      SBasicSensor newAngSpeed = MergeSensor(
                     PredictAngSpeed(deltaTimestamp, m_Heading, newHeading),
                     PredictSensorAccuracy(deltaTimestamp, m_AngSpeed)
                  );

      SBasicSensor newSpeed = MergeSensor(
                     PredictSpeed(deltaTimestamp, m_Position, newPosition, newAngSpeed),
                     PredictSensorAccuracy(deltaTimestamp, m_Speed)
                  );

      m_Timestamp = timestamp;
      m_Position  = newPosition;
      m_Heading   = newHeading;
      m_AngSpeed  = newAngSpeed;
      m_Speed     = newSpeed;
   }
}


void PE::CFusionSensor::AddHeading(const TTimestamp& timestamp, const SBasicSensor& heading)
{
   if      ( m_Timestamp == timestamp )
   {
      m_Heading = MergeHeading( m_Heading, heading );
   }
   else if( m_Timestamp < timestamp )
   {
      TTimestamp deltaTimestamp = timestamp - m_Timestamp;

      SBasicSensor newHeading = MergeHeading(
                     PredictHeading(deltaTimestamp, m_Heading, m_AngSpeed),
                     heading
                  );
      SBasicSensor newAngSpeed = MergeSensor(
                     PredictAngSpeed(deltaTimestamp, m_Heading, newHeading),
                     PredictSensorAccuracy(deltaTimestamp, m_AngSpeed)
                  );

      SPosition newPosition = PredictPosition(deltaTimestamp, m_Heading, newAngSpeed, m_Position, m_Speed);

      SBasicSensor newSpeed = MergeSensor(
                     PredictSpeed(deltaTimestamp, m_Position, newPosition, newAngSpeed),
                     PredictSensorAccuracy(deltaTimestamp, m_Speed)
                  );

      m_Timestamp = timestamp;
      m_Position  = newPosition;
      m_Heading   = newHeading;
      m_AngSpeed  = newAngSpeed;
      m_Speed     = newSpeed;
   }
}


void PE::CFusionSensor::AddSpeed(const TTimestamp& timestamp, const SBasicSensor& speed)
{
   if      ( m_Timestamp == timestamp )
   {
      m_Speed = MergeSensor( m_Speed, speed );
   }
   else if( m_Timestamp < timestamp )
   {
      TTimestamp deltaTimestamp = timestamp - m_Timestamp;

      SBasicSensor newSpeed = MergeSensor(
                     PredictSensorAccuracy(deltaTimestamp, m_Speed),
                     speed
                  );

      SBasicSensor newAngSpeed = PredictSensorAccuracy(deltaTimestamp, m_AngSpeed);

      SPosition newPosition = PredictPosition(deltaTimestamp, m_Heading, newAngSpeed, m_Position, newSpeed);

      SBasicSensor newHeading = MergeSensor(
                     PredictHeading(deltaTimestamp, m_Heading, newAngSpeed),
                     PredictHeading(m_Position, newPosition)
                  );

      m_Timestamp = timestamp;
      m_Position  = newPosition;
      m_Heading   = newHeading;
      m_AngSpeed  = newAngSpeed;
      m_Speed     = newSpeed;
   }
}


void PE::CFusionSensor::AddAngSpeed(const TTimestamp& timestamp, const SBasicSensor& angSpeed)
{
   if      ( m_Timestamp == timestamp )
   {
      m_AngSpeed = MergeSensor( m_AngSpeed, angSpeed );
   }
   else if( m_Timestamp < timestamp )
   {
      TTimestamp deltaTimestamp = timestamp - m_Timestamp;

      SBasicSensor newAngSpeed = MergeSensor(
                     PredictSensorAccuracy(deltaTimestamp, m_AngSpeed),
                     angSpeed
                  );

      SPosition newPosition = PredictPosition(deltaTimestamp, m_Heading, newAngSpeed, m_Position, m_Speed);

      SBasicSensor newHeading = MergeSensor(
                     PredictHeading(deltaTimestamp, m_Heading, newAngSpeed),
                     PredictHeading(m_Position, newPosition)
                  );

      SBasicSensor newSpeed = MergeSensor(
                     PredictSensorAccuracy(deltaTimestamp, m_Speed),
                     PredictSpeed(deltaTimestamp, m_Position, newPosition, newAngSpeed)
                  );

      m_Timestamp = timestamp;
      m_Position  = newPosition;
      m_Heading   = newHeading;
      m_AngSpeed  = newAngSpeed;
      m_Speed     = newSpeed;
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
