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
#ifndef __PE_IPositionFilter_H__
#define __PE_IPositionFilter_H__

#include "PETypes.h"
#include "PESPosition.h"
namespace PE
{
/**
 * Interface definition for position filters
 *
 */
class IPositionFilter
{
public:
   /**
    * Constructor of interface position filter
    *
    */
   IPositionFilter()
      : mTimestamp(0)
      , mPosition()
      {}
   /**
    * Empty virtual destructor
    *
    */
   virtual ~IPositionFilter()
      {}
   /**
    * Adds new position to the filter.
    * Has to be overloaded in child classes
    *
    * @param  timestamp    timestamp of new position in seconds.
    * @param  position     new position
    */
   virtual void AddPosition(const TTimestamp& timestamp, const SPosition& position) = 0;
   /**
    * Returns timestamp of position
    *
    * @return    position timestamp in seconds.
    */
   const TTimestamp& GetTimestamp() const
      {
         return mTimestamp;
      }
   /**
    * Returns filtered position
    *
    * @return    filtered position
    */
   const SPosition& GetPosition() const
      {
         return mPosition;
      }
protected:
   TTimestamp   mTimestamp;
   SPosition    mPosition;
};

} //namespace PE
#endif //__PE_IPositionFilter_H__
