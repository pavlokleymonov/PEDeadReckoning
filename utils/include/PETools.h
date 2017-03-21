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
 * @param first    first position
 * @param second   second position
 * @return         distance in meters
 */
TValue to_distance(const TPosition& first, const TPosition& second);

/**
 * Calculates distance between two coordinates.
 * Uses slow but precise algorithm.
 * Accuracy better then 1 meter for distance less then 1000km.
 *
 * @param first    first position
 * @param second   second position
 * @return         distance in meters
 */
TValue to_distance_precise(const TPosition& first, const TPosition& second);

/**
 * Calculates heading between two coordinates.
 *
 * @param first    first position
 * @param second   second position
 * @return         heading in degrees with reference to true north, 0.0 -> north, 90.0 -> east, 180.0 south, 270.0 -> west
 */
TValue to_heading(const TPosition& first, const TPosition& second);

/**
 * Calculates new coordinates based on distance, heading and first coordinates.
 *
 * @param position   start position
 * @param distnce    distance in meters
 * @param heading    heading in degrees
 * @return           new calculated position
 */
TPosition to_position(const TPosition& start, const TValue& distance, const TValue& heading);

/**
 * Calcs convergence persentage between two values. range [0..100] percent.
 * Bigger is better convergence.
 *
 * @param  first   first value to estimate
 * @param  second  second value to estimate
 * @return         convergence status. range [0..100] percent.
 */
TValue to_convergence(const TValue& first, const TValue& second);


} // namespace TOOLS
} // namespace PE

#endif //__PE_Tools_H__
