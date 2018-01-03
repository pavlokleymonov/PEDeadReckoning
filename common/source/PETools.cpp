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


TValue PE::TOOLS::ToRadians(const TValue& degrees)
{
   return (degrees * PI / 180.0);
}


TValue PE::TOOLS::ToDegrees(const TValue& radians)
{
   return (radians * 180.0 / PI);
}


TValue PE::TOOLS::ToDistance(const SPosition& first, const SPosition& second)
{
   TValue rLat1 = PE::TOOLS::ToRadians(first.Latitude);
   TValue rLat2 = PE::TOOLS::ToRadians(second.Latitude);
   TValue rLon1 = PE::TOOLS::ToRadians(first.Longitude);
   TValue rLon2 = PE::TOOLS::ToRadians(second.Longitude);
   TValue x = (rLon2-rLon1) * cos((rLat1+rLat2)/2);
   TValue y = rLat2-rLat1;
   return sqrt(x*x+y*y) * EARTH_RADIUS_M;
}

TValue PE::TOOLS::ToDistancePrecise(const SPosition& first, const SPosition& second)
{
   TValue rLat1 = PE::TOOLS::ToRadians(first.Latitude);
   TValue rLat2 = PE::TOOLS::ToRadians(second.Latitude);
   TValue rLon1 = PE::TOOLS::ToRadians(first.Longitude);
   TValue rLon2 = PE::TOOLS::ToRadians(second.Longitude);
   TValue dLat = rLat2-rLat1;
   TValue dLon = rLon2-rLon1;
   TValue a = pow(sin(dLat/2),2) + cos(rLat1)*cos(rLat2)*pow(sin(dLon/2),2);
   TValue c = 2 * atan2( sqrt(a), sqrt(1-a));
   return c * EARTH_RADIUS_M;
}

TValue PE::TOOLS::ToHeading(const SPosition& first, const SPosition& second)
{
   TValue rLat1 = PE::TOOLS::ToRadians(first.Latitude);
   TValue rLat2 = PE::TOOLS::ToRadians(second.Latitude);
   TValue rLon1 = PE::TOOLS::ToRadians(first.Longitude);
   TValue rLon2 = PE::TOOLS::ToRadians(second.Longitude);
   TValue dLon  = rLon2-rLon1;
   TValue rBearing = atan2( sin(dLon) * cos(rLat2), cos(rLat1) * sin(rLat2) - sin(rLat1) * cos(rLat2) * cos(dLon));
   return fmod(PE::TOOLS::ToDegrees(rBearing) + 360, 360.0);
}

TValue PE::TOOLS::ToHeading(const TValue& startHeading, const TTimestamp& deltaTime, const TValue& angularSpeed)
{
   return fmod(startHeading + (-angularSpeed * deltaTime) +360, 360.0);
}

SPosition PE::TOOLS::ToPosition(const SPosition& start, const TValue& distance, const TValue& heading)
{
   if (0 == distance)
      return SPosition(start);

   TValue rLat1 = PE::TOOLS::ToRadians(start.Latitude);
   TValue rLon1 = PE::TOOLS::ToRadians(start.Longitude);
   TValue Q = PE::TOOLS::ToRadians(heading);
   TValue b = distance / EARTH_RADIUS_M;

   TValue rLat2 = asin(sin(rLat1)*cos(b) + cos(rLat1)*sin(b)*cos(Q));
   TValue rLon2 = rLon1 + atan2(sin(Q)*sin(b)*cos(rLat1), cos(b) - sin(rLat1)*sin(rLat2));
   return SPosition( PE::TOOLS::ToDegrees(rLat2),
                     fmod(PE::TOOLS::ToDegrees(rLon2)+540, 360.0) -180.0,
                     start.HorizontalAcc );
}

void PE::TOOLS::Transform3D(TValue& xValue, TValue& yValue, TValue& zValue, const TValue& xRot, const TValue& yRot, const TValue& zRot )
{
   Transform2D(xValue,yValue,zRot);
   Transform2D(yValue,zValue,xRot);
   Transform2D(xValue,zValue,yRot);
}

void PE::TOOLS::Transform2D(TValue& xValue, TValue& yValue, const TValue& zRot )
{
   TValue x  = xValue;
   TValue y  = yValue;
   TValue fi = PE::TOOLS::ToRadians(zRot);
   xValue = x*cos(fi) - y*sin(fi);
   yValue = y*cos(fi) + x*sin(fi);
}

