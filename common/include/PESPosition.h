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


#ifndef __PE_SPosition_H__
#define __PE_SPosition_H__

#include "PETypes.h"

namespace PE {

   /**
    * Position information
    *
    */
   struct SPosition
   {
      /**
       * Default constructor
       */
      SPosition();
      /**
       * Short constructor
       */
      SPosition(const double& lat, const double& lon);
      /**
       * Full size constructor
       */
      SPosition(const double& lat, const double& lon, const double& hAcc);
      /**
       * The latitude in decimal degrees (-90..+90)
       */
      double Latitude;
      /**
       * The longitude in decimal degrees (-180..+180)
       */
      double Longitude;
      /**
       * The horizontal accuracy in meter
       */
      double HorizontalAcc;
      /**
       * Is position valid. All values have to be valid
       */
      bool IsValid() const;
   };

   bool operator==(const PE::SPosition& lhs, const PE::SPosition& rhs);

} // namespace PE

#endif //__PE_SPosition_H__
