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


#include "PESPosition.h"

using namespace PE;


PE::SPosition::SPosition()
: Latitude(ABS_MAX_LATITUDE+1)
, Longitude(ABS_MAX_LONGITUDE+1)
, LatitudeAcc(MAX_ACCURACY)
, LongitudeAcc(MAX_ACCURACY)
{};

PE::SPosition::SPosition(const TValue& lat, const TValue& lon)
: Latitude(lat)
, Longitude(lon)
, LatitudeAcc(MAX_ACCURACY)
, LongitudeAcc(MAX_ACCURACY)
{};

PE::SPosition::SPosition(const TValue& lat, const TValue& lon, const TAccuracy& latAcc, const TAccuracy& lonAcc)
: Latitude(lat)
, Longitude(lon)
, LatitudeAcc(latAcc)
, LongitudeAcc(lonAcc)
{};

bool PE::SPosition::IsValid() const
{
   return ( ABS_MAX_LATITUDE >= Latitude && 
           -ABS_MAX_LATITUDE <= Latitude && 
            ABS_MAX_LONGITUDE >= Longitude &&
           -ABS_MAX_LONGITUDE <= Longitude );
};

