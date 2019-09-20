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
#ifndef __PE_ISensor_H__
#define __PE_ISensor_H__

#include "PETypes.h"

namespace PE
{
/**
 * Interface class for sensors
 */
class ISensor
{
public:
   /**
    * Destructor of calibartion
    */
   virtual ~ISensor();
   /**
    * The sensor value. Unit is related to data meaning. Prefereable in system SI
    */
   TValue Value;
   /**
    * The sensor accuracy in same unit like a value
    */
   TAccuracy Accuracy;
   /**
    * Is sensor valid. The value and accuracy have to be valid
    * @return true if sensor is valid
    */
   virtual bool IsValid() const = 0;
};

} //namespace PE
#endif //__PE_ISensor_H__
