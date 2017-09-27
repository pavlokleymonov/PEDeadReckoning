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
#ifndef __PE_CPositionFilterSpeed_H__
#define __PE_CPositionFilterSpeed_H__

#include "PETypes.h"
#include "PEIPositionFilter.h"
namespace PE
{

/**
 * Filters out new position according to speed limit
 *
 */
class CPositionFilterSpeed: public IPositionFilter
{
public:
   /**
    * Constructor of position_filter_speed
    *
    * @param  speedLimit     speed limit below which new position will be ignored in m/s.
    */
   CPositionFilterSpeed(const TValue& speedLimit);
   /**
    * Adds new position to the filter. 
    *
    * @param  timestamp    timestamp of new position in seconds.
    * @param  position     new position
    */
   virtual void AddPosition(const TTimestamp& timestamp, const SPosition& position);
   /**
    * Returns used speed limits
    *
    * @return    speed limit in m/s.
    */
   const TValue& GetSpeedLimit() const;

private:
   TValue       mSpeedLimit;
};

} //namespace PE
#endif //__PE_CPositionFilterSpeed_H__
