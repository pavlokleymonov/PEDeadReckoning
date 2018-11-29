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


#ifndef __PE_Tools_H__
#define __PE_Tools_H__

#include "PETypes.h"
#include "PESPosition.h"

namespace PE {
namespace TOOLS {

/**
 * Converts degrees to radians
 *
 * @param degrees    degrees which are going to be converted
 * @return           conversion result in radians
 */
TValue ToRadians(const TValue& degrees);
/**
 * Converts radians to degrees
 *
 * @param radians    radians which are going to be converted
 * @return           conversion result in degrees
 */
TValue ToDegrees(const TValue& radians);
/**
 * Calculates distance between two coordinates.
 * Uses slow but precise algorithm.
 * Accuracy better then 1 meter for distance less then 1000km.
 *
 * @param firstLatitude    Latitude of first position in degrees
 * @param firstLongitude   Longitude of first position in degrees
 * @param lastLatitude     Latitude of last position in degrees
 * @param lastLongitude    Longitude of last position in degrees
 * @return                 distance in meters
 */
TValue ToDistance(const TValue& firstLatitude, const TValue& firstLongitude, const TValue& lastLatitude, const TValue& lastLongitude);
/**
 * Calculates distance between two coordinates.
 * Uses slow but precise algorithm.
 * Accuracy better then 1 meter for distance less then 1000km.
 * Considers curvature of turning driving
 *
 * @param firstHeading     first heading in degrees
 * @param firstLatitude    Latitude of first position in degrees
 * @param firstLongitude   Longitude of first position in degrees
 * @param lastLatitude     Latitude of last position in degrees
 * @param lastLongitude    Longitude of last position in degrees
 * @return                 distance in meters
 */
TValue ToDistance(const TValue& firstHeading, const TValue& firstLatitude, const TValue& firstLongitude, const TValue& lastLatitude, const TValue& lastLongitude);
/**
 * Calculates angle between two headings.
 * It takes always the shortest way
 *
 * @param firstHeading    first heading in degrees
 * @param secondHeading   second heading in degrees
 * @return                angle between two headings in degrees turning left("+") - positive, turning right("-") - negative
 */
TValue ToAngle(const TValue& firstHeading, const TValue& lastHeading);
/**
 * Calculates heading between two coordinates.
 *
 * @param firstLatitude    Latitude of first position in degrees
 * @param firstLongitude   Longitude of first position in degrees
 * @param lastLatitude     Latitude of last position in degrees
 * @param lastLongitude    Longitude of last position in degrees
 * @return                 heading in degrees with reference to true north, 0.0 -> north, 90.0 -> east, 180.0 south, 270.0 -> west
 */
TValue ToHeading(const TValue& firstLatitude, const TValue& firstLongitude, const TValue& lastLatitude, const TValue& lastLongitude);
/**
 * Calculates heading based on original heading and provided angle.
 *
 * @param heading   original heading in degrees with reference to true north, 0.0 -> north, 90.0 -> east, 180.0 south, 270.0 -> west
 * @param angle     angle to the new heading in degree turning left("+") - positive, turning right("-") - negative
 * @return          new heading in degrees with reference to true north, 0.0 -> north, 90.0 -> east, 180.0 south, 270.0 -> west
 */
TValue ToHeading(const TValue& heading, const TValue& angle);
/**
 * 2D - Transforms X/Y values to new coordinate system and update them accordingly
 *
 * @param xValue    X-component of value
 * @param yValue    Y-component of value
 * @param zRot      rotation around z-axis [deg]
 */
void Transform2D(TValue& xValue, TValue& yValue, const TValue& zRot );
/**
 * 3D - Transforms X/Y/Z values to new coordinate system and update them accordingly
 *
 * @param xValue    X-component of value
 * @param yValue    Y-component of value
 * @param zValue    Z-component of value
 * @param xRot      rotation around x-axis [deg]
 * @param yRot      rotation around y-axis [deg]
 * @param zRot      rotation around z-axis [deg]
 */
void Transform3D(TValue& xValue, TValue& yValue, TValue& zValue, const TValue& xRot, const TValue& yRot, const TValue& zRot );
/**
 * Splits string by delimiter
 * 
 * @param str        string which will be splitted by delimiter
 * @param delimiter  symbol which separated parts of the string
 * @return           vector of strings
 */
std::vector<std::string> Split(const std::string& str, char delimiter);


// /**
//  * Calculates new coordinates based on distance, heading and first coordinates.
//  *
//  * @param start      start position
//  * @param distnce    distance in meters
//  * @param heading    heading in degrees
//  * @return           new calculated position
//  */
//TO BE REMOVED!!!
SPosition ToPosition(const SPosition& start, const TValue& distance, const TValue& heading);

std::pair<TValue, TValue> ToPosition(const TValue& latitude, const TValue& longitude, const TValue& distance, const TValue& heading);


} // namespace TOOLS
} // namespace PE

#endif //__PE_Tools_H__
