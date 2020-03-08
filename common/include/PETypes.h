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


#ifndef __PE_Types_H__
#define __PE_Types_H__

#include <stdint.h>
#include <utility>
#include <limits>
#include <string>
#include <vector>

namespace PE {

   static const double EPSILON = 0.0000000001;

   static const double PI = 3.1415926535897931;

   static const double EARTH_RADIUS_M = 6371000.0;

   static const double ABS_MAX_LATITUDE = 90;

   static const double ABS_MAX_LONGITUDE = 180;

   static const double MAX_TIMESTAMP = std::numeric_limits<double>::max();

   static const double MIN_TIMESTAMP = EPSILON; // timestamp could be only positive

   static const double MAX_SPEED = 299792458.0; // speed of light m/s

   static const double MIN_SPEED = EPSILON; // speed could be only positive

   static const double MAX_VALUE = std::numeric_limits<double>::max();

   static const double MIN_VALUE = std::numeric_limits<double>::min();

   static const double MAX_ACCURACY = std::numeric_limits<double>::max();

   static const double MIN_ACCURACY = EPSILON;

   static const double DEFAULT_RELIABLE_LIMIT = 99.5;

   template <typename T>
   bool isnan(T realValue)
   {
      return ( realValue != realValue );
   }

   inline bool isepsilon(double v)
   {
      return ( v > 0 ? v < EPSILON : v > -EPSILON );
   }

} // namespace PE

#endif //__PE_Types_H__
