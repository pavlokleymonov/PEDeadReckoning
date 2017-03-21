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
#ifndef __PE_PositionFilter_H__
#define __PE_PositionFilter_H__

#include "PETypes.h"
namespace PE
{
/**
 * Filters out new position according to speed limit
 *
 */
class position_filter
{
public:
   /**
    * Constructor of position_filter
    *
    * @param  speed_limit     speed limit below which new position will be ignored in m/s.
    */
   position_filter(const TValue& speed_limit);
   /**
    * Adds new position to the filter
    *
    * @param  timestamp    timestamp of new position in seconds.
    * @param  position     new position
    */
   void add_position(const TTimestamp& timestamp, const TPosition& position);
   /**
    * Returns timestamp of position
    *
    * @return    position timestamp in seconds.
    */
   const TTimestamp& get_timestamp();
   /**
    * Returns filtered position
    *
    * @return    filtered position
    */
   const TPosition& get_position();
   /**
    * Returns used speed limits
    *
    * @return    speed limit in m/s.
    */
   const TValue& get_speed_limit();

private:
   TTimestamp   m_Timestamp;
   TPosition    m_Position;
   TValue       m_Speed_limit;
};


} //namespace PE
#endif //__PE_PositionFilter_H__
