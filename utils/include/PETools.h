/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2017 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file Copyright.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */


#ifndef __PE_Tools_H__
#define __PE_Tools_H__

#include "PETypes.h"

namespace PE {
namespace TOOLS {

/**
 * Converts degrees to radians
 *
 * @param degrees    degrees which are going to be converted
 * @return           conversion result in radians
 */
TValue to_radians(const TValue& degrees);

/**
 * Converts radians to degrees
 *
 * @param radians    radians which are going to be converted
 * @return           conversion result in degrees
 */
TValue to_degrees(const TValue& radians);

/**
 * Calculates distance between two coordinates.
 * Uses fast algorithm.
 * Accuracy better then 1 meter for distance less then 10km.
 *
 * @param lat1    latitude of first coordinate in degrees
 * @param lon1    longitude of first coordinate in degrees
 * @param lat2    latitude of second coordinate in degrees
 * @param lon2    longitude of second coordinate in degrees
 * @return        distance in meters
 */
TValue to_distance(const TValue& lat1, const TValue& lon1,const TValue& lat2, const TValue& lon2);

/**
 * Calculates distance between two coordinates.
 * Uses slow but precise algorithm.
 * Accuracy better then 1 meter for distance less then 1000km.
 *
 * @param lat1    latitude of first coordinate in degrees
 * @param lon1    longitude of first coordinate in degrees
 * @param lat2    latitude of second coordinate in degrees
 * @param lon2    longitude of second coordinate in degrees
 * @return        distance in meters
 */
TValue to_distance_precise(const TValue& lat1, const TValue& lon1,const TValue& lat2, const TValue& lon2);

/**
 * Calculates heading between two coordinates.
 *
 * @param lat1    latitude of first coordinate in degrees
 * @param lon1    longitude of first coordinate in degrees
 * @param lat2    latitude of second coordinate in degrees
 * @param lon2    longitude of second coordinate in degrees
 * @return        heading in degrees with reference to true north, 0.0 -> north, 90.0 -> east, 180.0 south, 270.0 -> west
 */
TValue to_heading(const TValue& lat1, const TValue& lon1,const TValue& lat2, const TValue& lon2);

/**
 * Calculates new coordinates based on distance, heading and first coordinates.
 *
 * @param [in]lat1     latitude of first coordinate in degrees
 * @param [in]lon1     longitude of first coordinate in degrees
 * @param [in]distnce  distance in meters
 * @param [in]heading  heading in degrees
 * @param [out]lat2    latitude of second coordinate in degrees
 * @param [out]lon2    longitude of second coordinate in degrees
 */
void get_next_coordinates(const TValue& lat1, const TValue& lon1, const TValue& distance, const TValue& heading, TValue& lat2, TValue& lon2);


} // namespace TOOLS
} // namespace PE

#endif //__PE_Tools_H__
