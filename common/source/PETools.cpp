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
#include <sstream>

using namespace PE;


TValue PE::TOOLS::ToRadians(const TValue& degrees)
{
   return (degrees * PI / 180.0);
}


TValue PE::TOOLS::ToDegrees(const TValue& radians)
{
   return (radians * 180.0 / PI);
}


TValue PE::TOOLS::ToDistance(const TValue& firstLatitude, const TValue& firstLongitude, const TValue& lastLatitude, const TValue& lastLongitude)
{
   TValue rLat1 = ToRadians(firstLatitude);
   TValue rLat2 = ToRadians(lastLatitude);
   TValue rLon1 = ToRadians(firstLongitude);
   TValue rLon2 = ToRadians(lastLongitude);
   TValue dLat = rLat2-rLat1;
   TValue dLon = rLon2-rLon1;
   TValue a = pow(sin(dLat/2),2) + cos(rLat1)*cos(rLat2)*pow(sin(dLon/2),2);
   TValue c = 2 * atan2( sqrt(a), sqrt(1-a));
   return c * EARTH_RADIUS_M;
}


TValue PE::TOOLS::ToDistance(const TValue& firstHeading, const TValue& firstLatitude, const TValue& firstLongitude, const TValue& lastLatitude, const TValue& lastLongitude)
{
   TValue secondHeading = ToHeading(firstLatitude, firstLongitude, lastLatitude, lastLongitude);
   TValue rBeta         = fabs(ToRadians(ToAngle(firstHeading, secondHeading)));
   TValue horda         = ToDistance(firstLatitude, firstLongitude, lastLatitude, lastLongitude);
   if ( EPSILON > rBeta )
   {
      return horda;
   }
   else if ( PI/2 > rBeta )
   {
      return horda / sin(rBeta) * rBeta;
   }
   else
   {
      return horda;
   }
}


TValue PE::TOOLS::ToAngle(const TValue& firstHeading, const TValue& secondHeading)
{
   if   ( 180 < (firstHeading - secondHeading))
   {
      return firstHeading - (secondHeading+360.0);
   }
   else if ( -180 > (firstHeading - secondHeading))
   {
      return firstHeading+360.0 - secondHeading;
   }
   else
   {
      return firstHeading - secondHeading;
   }
}


TValue PE::TOOLS::ToHeading(const TValue& firstLatitude, const TValue& firstLongitude, const TValue& lastLatitude, const TValue& lastLongitude)
{
   TValue rLat1 = ToRadians(firstLatitude);
   TValue rLat2 = ToRadians(lastLatitude);
   TValue rLon1 = ToRadians(firstLongitude);
   TValue rLon2 = ToRadians(lastLongitude);
   TValue dLon  = rLon2-rLon1;
   TValue rBearing = atan2( sin(dLon) * cos(rLat2), cos(rLat1) * sin(rLat2) - sin(rLat1) * cos(rLat2) * cos(dLon));
   return fmod(ToDegrees(rBearing) + 360, 360.0);
}


TValue PE::TOOLS::ToHeading(const TValue& startHeading, const TValue& angle)
{
   return fmod(startHeading + (-angle) +360, 360.0);
}


void PE::TOOLS::Transform3D(TValue& xValue, TValue& yValue, TValue& zValue, const TValue& xRot, const TValue& yRot, const TValue& zRot )
{
   Transform2D(yValue,zValue,xRot);
   Transform2D(zValue,xValue,yRot);
   Transform2D(xValue,yValue,zRot);
}


void PE::TOOLS::Transform2D(TValue& xValue, TValue& yValue, const TValue& zRot )
{
   TValue x  = xValue;
   TValue y  = yValue;
   TValue fi = PE::TOOLS::ToRadians(zRot);
   xValue = x*cos(fi) - y*sin(fi);
   yValue = y*cos(fi) + x*sin(fi);
}


std::vector<std::string> PE::TOOLS::Split(const std::string& str, char delimiter)
{
   std::vector<std::string> result;
   std::stringstream ss(str);
   std::string item;
   while (std::getline(ss, item, delimiter))
   {
      result.push_back(item);
   }
   if ( (false == str.empty()) && (delimiter == str[str.size()-1]) )
   {
      result.push_back("");
   }
   return result;
}


SPosition PE::TOOLS::ToPosition(const SPosition& start, const TValue& distance, const TValue& heading)
{
   if (0 == distance)
      return SPosition(start);

//    TValue rLat1 = PE::TOOLS::ToRadians(start.Latitude);
//    TValue rLon1 = PE::TOOLS::ToRadians(start.Longitude);
//    TValue Q = PE::TOOLS::ToRadians(heading);
//    TValue b = distance / EARTH_RADIUS_M;
// 
//    TValue rLat2 = asin(sin(rLat1)*cos(b) + cos(rLat1)*sin(b)*cos(Q));
//    TValue rLon2 = rLon1 + atan2(sin(Q)*sin(b)*cos(rLat1), cos(b) - sin(rLat1)*sin(rLat2));
//    return SPosition( PE::TOOLS::ToDegrees(rLat2),
//                      fmod(PE::TOOLS::ToDegrees(rLon2)+540, 360.0) -180.0,
//                      start.HorizontalAcc );
   std::pair<TValue, TValue> pos = ToPosition(start.Latitude, start.Longitude, distance, heading);
   return SPosition(pos.first, pos.second,start.HorizontalAcc);
}


std::pair<TValue, TValue> PE::TOOLS::ToPosition(const TValue& latitude, const TValue& longitude, const TValue& distance, const TValue& heading)
{
   TValue rLat1 = PE::TOOLS::ToRadians(latitude);
   TValue rLon1 = PE::TOOLS::ToRadians(longitude);
   TValue Q = PE::TOOLS::ToRadians(heading);
   TValue b = distance / EARTH_RADIUS_M;

   TValue rLat2 = asin(sin(rLat1)*cos(b) + cos(rLat1)*sin(b)*cos(Q));
   TValue rLon2 = rLon1 + atan2(sin(Q)*sin(b)*cos(rLat1), cos(b) - sin(rLat1)*sin(rLat2));

   return std::make_pair(PE::TOOLS::ToDegrees(rLat2), fmod(PE::TOOLS::ToDegrees(rLon2)+540, 360.0) -180.0);
}


// std::pair<TValue, TValue> PE::TOOLS::ToPosition(const TValue& latitude, const TValue& longitude, const TValue& distance, const TValue& heading, const TValue& angle)
// {
//    TValue rBeta = ToRadians(angle) / 2;
//    if ( EPSILON < angle && PI/2 > rBeta )
//    {
//       TValue hordaHeading = ToHeading(heading, angle / 2);
//       TValue horda = distance * sin(rBeta) / rBeta;
//       return ToPosition(latitude, longitude, horda, hordaHeading);
//    }
//    return ToPosition(latitude, longitude, distance, heading);
// }
