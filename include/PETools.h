#ifndef __PE_Tools_H__
#define __PE_Tools_H__

#include <math.h>
#include "PETypes.h"
#include "PESensors.h"

namespace PE {
namespace TOOLS {

Position position_nmea(
   const TTimestamp& timestamp,
   const std::string& RMC,
   const std::string& GGA
);

Position position_lite(
   const TTimestamp& timestamp,
   const TValue&     utc,
   const TGnssFix&   fix,
   const TValue&     latitude,
   const TValue&     longitude,
   const TValue&     altitude,
   const TValue&     heading,
   const TValue&     speed,
   const TValue&     PDOP
);


Position position_2D(
   const TTimestamp& timestamp,
   const TValue&     utc,
   const TGnssFix&   fix,
   const TValue&     latitude,
   const TValue&     longitude,
   const TValue&     heading,
   const TValue&     speed2D,
   const TAccuracy&  horizontal_accuracy
);


Position position_3D(
   const TTimestamp& timestamp,
   const TValue&     utc,
   const TGnssFix&   fix,
   const TValue&     latitude,
   const TAccuracy&  latitude_accuracy,
   const TValue&     longitude,
   const TAccuracy&  longitude_accuracy,
   const TValue&     altitude,
   const TAccuracy&  altitude_accuracy,
   const TValue&     heading,
   const TAccuracy&  heading_accuracy,
   const TValue&     speed,
   const TAccuracy&  speed_accuracy
);

TValue to_radians(const TValue& degrees);
{
   return (degrees * PI / 180.0);
}

TValue to_degrees(const TValue& radians);
{
   return (radians * 180.0 / PI);
}


TValue to_distance(const TValue& lat1, const TValue& lon1,const TValue& lat2, const TValue& lon2)
{
   TValue rLat1 = to_radians(lat1);
   TValue rLat2 = to_radians(lat2);
   TValue rLon1 = to_radians(lon1);
   TValue rLon2 = to_radians(lon2);
   TValue x = rLon2-rLon1 * std::cos((rLat1+rLat2)/2);
   TValue y = rLat2-rLat1;
   return (std::sqrt(x*x+y*y) * EARTH_RADIUS_M);
}

TValue to_heading(const TValue& lat1, const TValue& lon1,const TValue& lat2, const TValue& lon2)
{
   TValue rLat1 = to_radians(lat1);
   TValue rLat2 = to_radians(lat2);
   TValue rLon1 = to_radians(lon1);
   TValue rLon2 = to_radians(lon2);
   TValue dLon  = rLon2-rLon1;
   TValue rBearing = std::atan2(std::sin(dLon) * std::cos(rLat2), std::cos(rLat1) * std::sin(rLat2) - std::sin(rLat1) * std::cos(rLat2) * std::cos(dLon));
   return (to_degrees(rBearing) + 360) % 360;
}

} // namespace TOOLS
} // namespace PE

#endif //__PE_Tools_H__
