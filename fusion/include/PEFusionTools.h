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
#ifndef __PE_FusionTools_H__
#define __PE_FusionTools_H__

#include "PETypes.h"
#include "PESPosition.h"
#include "PESBasicSensor.h"

namespace PE {
namespace FUSION {


//SBasicSensor GetHeading(const SBasicSensor& firstHeading, const SPosition& firstPos, const SPosition& lastPos);






/**
 * Predicts(reduce) sensor accuracy based on delta time and original accuracy
 *
 * @param deltaTimestamp   delta timestamp between original and predicted accuracy
 * @param sensor           original sensors data
 * @return                 sensors data wich is adopted according to deltatimestamp
 */
SBasicSensor PredictSensorAccuracy(const double& deltaTimestamp, const SBasicSensor& sensor);

/**
 * Predicts new heading based on knowed angular velocity, delta time and original heading.
 *
 * @param deltaTimestamp   delta timestamp between original values and predicted heading
 * @param heading          original heading
 * @param angSpeed         angular velocity
 * @return                 predicted heading
 */
SBasicSensor PredictHeading(const double& deltaTimestamp, const SBasicSensor& heading, const SBasicSensor& angSpeed);

/**
 * Predicts new heading based on knowed old and new positions.
 *
 * @param deltaTimestamp      delta timestamp between oold and new positions
 * @param positionFirst       old position
 * @param positionLast        new position
 * @param previouse heading   heading
 * @return                    predicted heading
 */
SBasicSensor PredictHeading(const double& deltaTimestamp, const SPosition& positionFirst, const SPosition& positionLast, const SBasicSensor& heading);

/**
 * Predicts new position based on knowed start heading, angular velocity, linear speed, delta time and original position.
 *
 * @param deltaTimestamp   delta timestamp between original values and predicted position
 * @param heading          start heading
 * @param angSpeed         angular velocity
 * @param position         original position
 * @param speed            linear speed
 * @return                 predicted position
 */
SPosition PredictPosition(const double& deltaTimestamp, const SBasicSensor& heading, const SBasicSensor& angSpeed, const SPosition& position, const SBasicSensor& speed);

/**
 * Predicts new linear speed based on knowed delta time, angular velocity, old and new positions.
 *
 * @param deltaTimestamp   delta timestamp between old and new positions
 * @param positionFirst    old position
 * @param positionLast     new position
 * @param angSpeed         angular velocity
 * @return                 predicted linear speed
 */
SBasicSensor PredictSpeed(const double& deltaTimestamp, const SPosition& positionFirst, const SPosition& positionLast, const SBasicSensor& angSpeed);

/**
 * Predicts new angular velocity based on knowed delta time, old and new headings.
 *
 * @param deltaTimestamp   delta timestamp between old and new headings
 * @param headingFirst     old heading
 * @param headingLast      new heading
 * @return                 predicted angular velocity
 */
SBasicSensor PredictAngSpeed(const double& deltaTimestamp, const SBasicSensor& headingFirst, const SBasicSensor& headingLast);

/**
 * Merges two sensors with concerning of its accuracies.
 *
 * @param sen1             first sensor
 * @param sen2             second sensor
 * @return                 merged sensor
 */
SBasicSensor MergeSensor(const SBasicSensor& sen1, const SBasicSensor& sen2);

/**
 * Merges two headings with concerning of its accuracies.
 *
 * @param head1            first heading
 * @param head2            second heading
 * @return                 merged heading
 */
SBasicSensor MergeHeading(const SBasicSensor& head1, const SBasicSensor& head2);

/**
 * Merges two positions with concerning of its accuracies.
 *
 * @param pos1             first position
 * @param pos2             second position
 * @return                 merged position
 */
SPosition MergePosition(const SPosition& pos1, const SPosition& pos2);

/**
 * Merges two values based on accuracy
 * by using Kalman filter
 *
 * @param value1             first value
 * @param accuracy1          first accuracy
 * @param value2             second value
 * @param accuracy2          second accuracy
 * @return                   merged values
 */
double KalmanFilter(const double& value1, const double& accuracy1, const double& value2, const double& accuracy2);


} // namespace FUSION
} // namespace PE

#endif //__PE_FusionTools_H__
