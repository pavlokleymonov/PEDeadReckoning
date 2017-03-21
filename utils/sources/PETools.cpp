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


#include "PETools.h"
#include <math.h>

using namespace PE;


TValue PE::TOOLS::to_radians(const TValue& degrees)
{
   return (degrees * PI / 180.0);
}


TValue PE::TOOLS::to_degrees(const TValue& radians)
{
   return (radians * 180.0 / PI);
}


TValue PE::TOOLS::to_distance(const TPosition& first, const TPosition& second)
{
   TValue rLat1 = PE::TOOLS::to_radians(first.Latitude);
   TValue rLat2 = PE::TOOLS::to_radians(second.Latitude);
   TValue rLon1 = PE::TOOLS::to_radians(first.Longitude);
   TValue rLon2 = PE::TOOLS::to_radians(second.Longitude);
   TValue x = (rLon2-rLon1) * cos((rLat1+rLat2)/2);
   TValue y = rLat2-rLat1;
   return sqrt(x*x+y*y) * EARTH_RADIUS_M;
}

TValue PE::TOOLS::to_distance_precise(const TPosition& first, const TPosition& second)
{
   TValue rLat1 = PE::TOOLS::to_radians(first.Latitude);
   TValue rLat2 = PE::TOOLS::to_radians(second.Latitude);
   TValue rLon1 = PE::TOOLS::to_radians(first.Longitude);
   TValue rLon2 = PE::TOOLS::to_radians(second.Longitude);
   TValue dLat = rLat2-rLat1;
   TValue dLon = rLon2-rLon1;
   TValue a = pow(sin(dLat/2),2) + cos(rLat1)*cos(rLat2)*pow(sin(dLon/2),2);
   TValue c = 2 * atan2( sqrt(a), sqrt(1-a));
   return c * EARTH_RADIUS_M;
}

TValue PE::TOOLS::to_heading(const TPosition& first, const TPosition& second)
{
   TValue rLat1 = PE::TOOLS::to_radians(first.Latitude);
   TValue rLat2 = PE::TOOLS::to_radians(second.Latitude);
   TValue rLon1 = PE::TOOLS::to_radians(first.Longitude);
   TValue rLon2 = PE::TOOLS::to_radians(second.Longitude);
   TValue dLon  = rLon2-rLon1;
   TValue rBearing = atan2( sin(dLon) * cos(rLat2), cos(rLat1) * sin(rLat2) - sin(rLat1) * cos(rLat2) * cos(dLon));
   return fmod(PE::TOOLS::to_degrees(rBearing) + 360, 360.0);
}

TPosition PE::TOOLS::to_position(const TPosition& start, const TValue& distance, const TValue& heading)
{
   TValue rLat1 = PE::TOOLS::to_radians(start.Latitude);
   TValue rLon1 = PE::TOOLS::to_radians(start.Longitude);
   TValue Q = PE::TOOLS::to_radians(heading);
   TValue b = distance / EARTH_RADIUS_M;

   TValue rLat2 = asin(sin(rLat1)*cos(b) + cos(rLat1)*sin(b)*cos(Q));
   TValue rLon2 = rLon1 + atan2(sin(Q)*sin(b)*cos(rLat1), cos(b) - sin(rLat1)*sin(rLat2));
//    TPosition new_position()
//    lat2 = PE::TOOLS::to_degrees(rLat2);
//    lon2 = fmod(PE::TOOLS::to_degrees(rLon2)+540, 360.0) -180.0;
   return TPosition( PE::TOOLS::to_degrees(rLat2),
                     fmod(PE::TOOLS::to_degrees(rLon2)+540, 360.0) -180.0 );
}

TValue PE::TOOLS::to_convergence(const TValue& first, const TValue& second)
{
   if ( first == second )
   {
      return 100.0;
   }

   if ( 0 > first*second ) //different sign is treated alwas as 0%
   {
      return 0.0;
   }

   TValue a_first  = fabs(first);
   TValue a_second = fabs(second);
   if ( a_first > a_second )
   {
      return 100 - ((a_first - a_second) * 100.0 / a_first);
   }
   else
   {
      return 100 - ((second - first) * 100.0 / second);
   }
}


