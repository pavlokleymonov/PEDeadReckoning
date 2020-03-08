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


double PE::TOOLS::ToRadians(const double& degrees)
{
   return (degrees * PI / 180.0);
}


double PE::TOOLS::ToDegrees(const double& radians)
{
   return (radians * 180.0 / PI);
}


double PE::TOOLS::ToDistance(const double& firstLatitude, const double& firstLongitude, const double& lastLatitude, const double& lastLongitude)
{
   double rLat1 = ToRadians(firstLatitude);
   double rLat2 = ToRadians(lastLatitude);
   double rLon1 = ToRadians(firstLongitude);
   double rLon2 = ToRadians(lastLongitude);
   double dLat = rLat2-rLat1;
   double dLon = rLon2-rLon1;
   double a = pow(sin(dLat/2),2) + cos(rLat1)*cos(rLat2)*pow(sin(dLon/2),2);
   double c = 2 * atan2( sqrt(a), sqrt(1-a));
   return c * EARTH_RADIUS_M;
}


double PE::TOOLS::ToDistance(const double& firstHeading, const double& firstLatitude, const double& firstLongitude, const double& lastLatitude, const double& lastLongitude)
{
   double secondHeading = ToHeading(firstLatitude, firstLongitude, lastLatitude, lastLongitude);
   double rBeta         = fabs(ToRadians(ToAngle(firstHeading, secondHeading)));
   double horda         = ToDistance(firstLatitude, firstLongitude, lastLatitude, lastLongitude);
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


double PE::TOOLS::ToAngle(const double& firstHeading, const double& secondHeading)
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


double PE::TOOLS::ToHeading(const double& firstLatitude, const double& firstLongitude, const double& lastLatitude, const double& lastLongitude)
{
   double rLat1 = ToRadians(firstLatitude);
   double rLat2 = ToRadians(lastLatitude);
   double rLon1 = ToRadians(firstLongitude);
   double rLon2 = ToRadians(lastLongitude);
   double dLon  = rLon2-rLon1;
   double rBearing = atan2( sin(dLon) * cos(rLat2), cos(rLat1) * sin(rLat2) - sin(rLat1) * cos(rLat2) * cos(dLon));
   return fmod(ToDegrees(rBearing) + 360, 360.0);
}


double PE::TOOLS::ToHeading(const double& startHeading, const double& angle)
{
   return fmod(startHeading + (-angle) +360, 360.0);
}


void PE::TOOLS::Transform3D(double& xValue, double& yValue, double& zValue, const double& xRot, const double& yRot, const double& zRot )
{
   Transform2D(yValue,zValue,xRot);
   Transform2D(zValue,xValue,yRot);
   Transform2D(xValue,yValue,zRot);
}


void PE::TOOLS::Transform2D(double& xValue, double& yValue, const double& zRot )
{
   double x  = xValue;
   double y  = yValue;
   double fi = PE::TOOLS::ToRadians(zRot);
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


SPosition PE::TOOLS::ToPosition(const SPosition& start, const double& distance, const double& heading)
{
   if (0 == distance)
      return SPosition(start);

//    double rLat1 = PE::TOOLS::ToRadians(start.Latitude);
//    double rLon1 = PE::TOOLS::ToRadians(start.Longitude);
//    double Q = PE::TOOLS::ToRadians(heading);
//    double b = distance / EARTH_RADIUS_M;
// 
//    double rLat2 = asin(sin(rLat1)*cos(b) + cos(rLat1)*sin(b)*cos(Q));
//    double rLon2 = rLon1 + atan2(sin(Q)*sin(b)*cos(rLat1), cos(b) - sin(rLat1)*sin(rLat2));
//    return SPosition( PE::TOOLS::ToDegrees(rLat2),
//                      fmod(PE::TOOLS::ToDegrees(rLon2)+540, 360.0) -180.0,
//                      start.HorizontalAcc );
   std::pair<double, double> pos = ToPosition(start.Latitude, start.Longitude, distance, heading);
   return SPosition(pos.first, pos.second,start.HorizontalAcc);
}


std::pair<double, double> PE::TOOLS::ToPosition(const double& latitude, const double& longitude, const double& distance, const double& heading)
{
   double rLat1 = PE::TOOLS::ToRadians(latitude);
   double rLon1 = PE::TOOLS::ToRadians(longitude);
   double Q = PE::TOOLS::ToRadians(heading);
   double b = distance / EARTH_RADIUS_M;

   double rLat2 = asin(sin(rLat1)*cos(b) + cos(rLat1)*sin(b)*cos(Q));
   double rLon2 = rLon1 + atan2(sin(Q)*sin(b)*cos(rLat1), cos(b) - sin(rLat1)*sin(rLat2));

   return std::make_pair(PE::TOOLS::ToDegrees(rLat2), fmod(PE::TOOLS::ToDegrees(rLon2)+540, 360.0) -180.0);
}


// std::pair<double, double> PE::TOOLS::ToPosition(const double& latitude, const double& longitude, const double& distance, const double& heading, const double& angle)
// {
//    double rBeta = ToRadians(angle) / 2;
//    if ( EPSILON < angle && PI/2 > rBeta )
//    {
//       double hordaHeading = ToHeading(heading, angle / 2);
//       double horda = distance * sin(rBeta) / rBeta;
//       return ToPosition(latitude, longitude, horda, hordaHeading);
//    }
//    return ToPosition(latitude, longitude, distance, heading);
// }
