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
#include <limits>
#include <string>

namespace PE {
namespace TYPES {
   typedef double TTimestamp;

   typedef double TValue;

   typedef double TAccuracy;

   static const double PI = 3.1415926535897931;

   static const double EARTH_RADIUS_M = 6371000.0;

   static const double ABS_MAX_LATITUDE = 90;

   static const double ABS_MAX_LONGITUDE = 180;

   static const TTimestamp MAX_TIMESTAMP = std::numeric_limits<TTimestamp>::max();

   static const TValue MAX_VALUE = std::numeric_limits<TValue>::max();

   static const TAccuracy MAX_ACCURACY = std::numeric_limits<TAccuracy>::max();

} // namespace TYPES
} // namespace PE

#endif //__PE_Types_H__
