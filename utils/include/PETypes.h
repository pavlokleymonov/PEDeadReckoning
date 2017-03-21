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

namespace PE
{
   typedef double TTimestamp;

   typedef double TValue;

   typedef double TAccuracy;

   static const double PI = 3.1415926535897931;

   static const double EARTH_RADIUS_M = 6371000.0;

   static const TTimestamp MAX_TIMESTAMP = std::numeric_limits<TTimestamp>::max();

   static const TValue MAX_VALUE = std::numeric_limits<TValue>::max();

   static const TAccuracy MAX_ACCURACY = std::numeric_limits<TAccuracy>::max();

   /**
    * Position information
    *
    */
   struct TPosition
   {
      TPosition()
         : Latitude(MAX_VALUE)
         , Longitude(MAX_VALUE)
         {};
      TPosition(const TValue& lat, const TValue& lon)
         : Latitude(lat)
         , Longitude(lon)
         {};
      /**
       * The latitude in decimal degrees (-90..+90)
       */
      TValue Latitude;
      /**
       * The longitude in decimal degrees (-180..+180)
       */
      TValue Longitude;
      /**
       * Is position valid. All values have to be valid
       */
      bool is_valid() const
         {
            return ( MAX_VALUE != Latitude && MAX_VALUE != Longitude );
         };
   };

} //namespace PE
#endif //__PE_Types_H__
