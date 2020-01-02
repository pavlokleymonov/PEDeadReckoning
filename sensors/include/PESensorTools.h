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
#ifndef __PE_SensorTools_H__
#define __PE_SensorTools_H__

#include "PETypes.h"

namespace PE {
namespace Sensor {

/**
 * Predict value based on linear relationship which is fit to the requested timestamp
 * @return   calculated value in the same units of boundary values
 *
 * @param requestedTs   requested timestamp in [s]
 * @param leftTs        timestamp before requested timestamp [s]
 * @param rightTs       timestamp after requested timestamp [s]
 * @param leftValue     left border value in units
 * @param rightValue    right border value in units
 */
TValue PredictValue( const TTimestamp& requestedTs, const TTimestamp& leftTs, const TTimestamp& rightTs, const TValue& leftValue, const TValue& rightValue );

/**
 * Checks if interval between two timestamp corresponds to defined limits
 * @return   true if delta time between two timestamps passed all checkings
 *
 * @param  deltaTs      delta time for checking in [s]
 * @param  interval     interval limit in [s]
 * @param  hysteresis   hysteresis between interval and difference of two timestamps in [s]
 */
bool IsIntervalOk( const TTimestamp& deltaTs, const TTimestamp& interval, const TTimestamp& hysteresis );

/**
 * Checks if value is bigger than accuracy with specified ratio coefficient
 * @return   true if value passed all checkings
 *
 * @param  value      to be checked
 * @param  accuracy   accuracy of the value
 * @param  ratio      ratio coefficient (how much times the value has to be bigger than accuracy )
 */
bool IsAccuracyOk( const TValue& value, const TAccuracy& accuracy, const TValue& ratio );

/**
 * Checks if given tested timestamp is in specified range
 * @return   true if testedTS belongs to the range [beginTS .. endTS]
 *
 * @param testedTS   tested timestamp
 * @param beginTS    beginning of the specified range
 * @param endTS      end of the specified range
 */
bool IsInRange( const TTimestamp& testedTS, const TTimestamp& beginTS, const TTimestamp& endTS );


} // namespace Sensor
} // namespace PE

#endif //__PE_SensorTools_H__
