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

#include "PEPositionFilter.h"
#include "PETools.h"
#include <math.h>


using namespace PE;

PE::I_position_filter::I_position_filter()
: m_Timestamp(0)
, m_Position()
{
}

PE::I_position_filter::~I_position_filter()
{
}

const TTimestamp& PE::I_position_filter::get_timestamp() const
{
   return m_Timestamp;
}

const TPosition& PE::I_position_filter::get_position() const
{
   return m_Position;
}


PE::position_filter_speed::position_filter_speed(const TValue& speed_limit)
: I_position_filter()
, m_Speed_limit(speed_limit)
{
}

void PE::position_filter_speed::add_position(const TTimestamp& timestamp, const TPosition& position)
{
   if ( 0 < m_Timestamp && m_Position.is_valid() ) //internal position is valid
   {
      if ( m_Timestamp < timestamp && position.is_valid() ) //new position is valid
      {
         TValue speed = PE::TOOLS::to_distance(m_Position, position) / (timestamp - m_Timestamp); //speed in m/s
         if ( speed > m_Speed_limit ) //speed is above speed limit
         {
            m_Timestamp = timestamp;
            m_Position = position;
         }
      }
   }
   else //incorrect internal position has to be updated
   {
      m_Timestamp = timestamp;
      m_Position = position;
   }
}

const TValue& PE::position_filter_speed::get_speed_limit()
{
   return m_Speed_limit;
}


