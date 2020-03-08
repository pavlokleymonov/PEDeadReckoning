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
#ifndef __PE_SBasicSensor_H__
#define __PE_SBasicSensor_H__

#include "PETypes.h"

namespace PE
{
   /**
    * Basic sensor
    *
    */
   struct SBasicSensor
   {
      /**
       * Constructor
       */
      SBasicSensor();
      /**
       * Constructor
       */
      SBasicSensor(const double& value, const double& accuracy);
      /**
       * The sensor value. Unit is related to data meaning. Prefereable in system SI
       */
      double Value;
      /**
       * The sensor accuracy in same unit like a value
       */
      double Accuracy;
      /**
       * Is sensor valid. The value and accuracy have to be valid
       */
      bool IsValid() const;
   };

   bool operator==(const PE::SBasicSensor& lhs, const PE::SBasicSensor& rhs);

} //namespace PE
#endif //__PE_SBasicSensor_H__
