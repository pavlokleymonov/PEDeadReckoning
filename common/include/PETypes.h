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

   typedef enum {
      SENSOR_UNKNOWN          = 0,
      SENSOR_LATITUDE         = 1,
      SENSOR_LONGITUDE        = 2,
      SENSOR_HEADING          = 3,
      SENSOR_SPEED            = 4,
      SENSOR_ODOMETER_AXIS    = 5,
      SENSOR_GYRO_Z           = 6,
      SENSOR_ODOMETER_WHEEL   = 7, //Not supported
      SENSOR_GYRO_X           = 8, //Not supported
      SENSOR_GYRO_Y           = 9, //Not supported
      SENSOR_ACCELEROMETER_X  = 10,//Not supported
      SENSOR_ACCELEROMETER_Y  = 11,//Not supported
      SENSOR_ACCELEROMETER_Z  = 12,//Not supported
      SENSOR_STEERING_ANGLE   = 13 //Not supported
   } TSensorTypeID;

   typedef uint32_t TSensorID;

   typedef double TTimestamp;

   typedef double TValue;

   typedef double TAccuracy;

   static const TValue EPSILON = 0.0000000001;

   static const TValue PI = 3.1415926535897931;

   static const TValue EARTH_RADIUS_M = 6371000.0;

   static const TValue ABS_MAX_LATITUDE = 90;

   static const TValue ABS_MAX_LONGITUDE = 180;

   static const TTimestamp MAX_TIMESTAMP = std::numeric_limits<TTimestamp>::max();

   static const TValue MAX_VALUE = std::numeric_limits<TValue>::max();

   static const TAccuracy MAX_ACCURACY = std::numeric_limits<TAccuracy>::max();

   static const TValue DEFAULT_RELIABLE_LIMIT = 99.5;

   template <typename T>
   bool isnan(T realValue)
   {
      return ( realValue != realValue );
   }

   inline bool isepsilon(TValue v)
   {
      return ( v > 0 ? v < EPSILON : v > -EPSILON );
   }

} // namespace PE

#endif //__PE_Types_H__
