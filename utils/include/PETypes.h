#ifndef __PE_Types_H__
#define __PE_Types_H__

#include <stdint.h>
#include <limits>
#include <string>

namespace PE
{
   typedef int8_t TSensorID;

   typedef double TTimestamp;

   typedef double TValue;

   typedef double TAccuracy;

   enum TSensor
   {
      ESensorLatitude  = 0x00,
      ESensorLongitude = 0x01,
      ESensorAltitude  = 0x02,
      ESensorHeading   = 0x03,
      ESensorSpeed     = 0x04,
      ESensorInvalid   = 0xFF
   };

   enum TGnssFix
   {
      ENoFix   = 0,
      ETimeFix = 1,
      E2DFix   = 2,
      E3DFix   = 3
   };

   //NMEA Status V = Navigation receiver warning, A = Data valid
   enum TNmeaStatus
   {
      E_V_INVALID = 0x0, //V = Navigation receiver warning
      E_A_VALID   = 0x1  //A = Data valid
   };

   //NMEA Mode Indicator N=No Fix, E=Estimated/Dead Reckoning Fix, A=Autonomous GNSS Fix, D=Differential GNSS Fix
   enum TNmeaMode
   {
      E_N_NO_FIX        = 0x0, //N=No Fix
      E_E_DR_FIX        = 0x1, //E=Estimated/Dead Reckoning Fix
      E_A_GNSS_FIX      = 0x2, //A=Autonomous GNSS Fix
      E_D_DIFF_GNSS_FIX = 0x3, //D=Differential GNSS Fix (e.g.:SBAS)
   };

   static const double UNCALIBRATED_SENSOR = 0.0;

   static const double ACCURACY_LIMIT = 999.99;

   static const double PI = 3.1415926535897931;

   static const double EARTH_RADIUS_M = 6371000.0;

   static const double INVALID = std::numeric_limits<double>::quiet_NaN();

} //namespace PE
#endif //__PE_Types_H__
