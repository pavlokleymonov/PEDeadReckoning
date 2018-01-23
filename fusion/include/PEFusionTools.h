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

/**
 * Predicts(reduce) sensor accuracy based on delta time and original accuracy
 *
 * @param timestampFirst   timestamp of original accuracy
 * @param sensor           original sensors data
 * @param timestampLast    timestamp for predicted status of sensor
 * @return                 sensors data wich is adopted for new timestamp
 */
SBasicSensor PredictSensorAccuracy(const TTimestamp& timestampFirst, const SBasicSensor& sensor, const TTimestamp& timestampLast);

/**
 * Predicts new heading based on knowed angular velocity, delta time and original heading.
 *
 * @param timestampFirst   timestamp of original heading
 * @param heading          original heading
 * @param timestampLast    timestamp for predicted heading
 * @param angSpeed         angular velocity
 * @return                 predicted heading
 */
SBasicSensor PredictHeading(const TTimestamp& timestampFirst, const SBasicSensor& heading, const TTimestamp& timestampLast, const SBasicSensor& angSpeed);

/**
 * Predicts new heading based on knowed old and new positions.
 *
 * @param positionFirst    old position
 * @param positionLast     new position
 * @return                 predicted heading
 */
SBasicSensor PredictHeading(const SPosition& positionFirst, const SPosition& positionLast);

/**
 * Predicts new position based on knowed old, new heading, linear speed, delta time and original position.
 *
 * @param timestampFirst   timestamp of original heading and position
 * @param headingFirst     old heading
 * @param timestampLast    timestamp for predicted position
 * @param headingLast      new heading
 * @param position         original position
 * @param speed            linear speed
 * @return                 predicted position
 */
SPosition   PredictPosition(const TTimestamp& timestampFirst, const SBasicSensor& headingFirst, const TTimestamp& timestampLast, const SBasicSensor& headingLast, const SPosition& position, const SBasicSensor& speed);

/**
 * Predicts new linear speed based on knowed delta time, old and new positions.
 *
 * @param timestampFirst   timestamp of old position
 * @param positionFirst    old position
 * @param timestampLast    timestamp for new position
 * @param positionLast     new position
 * @return                 predicted linear speed
 */
SBasicSensor PredictSpeed(const TTimestamp& timestampFirst, const SPosition& positionFirst, const TTimestamp& timestampLast, const SPosition& positionLast);

/**
 * Predicts new angular velocity based on knowed delta time, old and new headings.
 *
 * @param timestampFirst   timestamp of old heading
 * @param headingFirst     old heading
 * @param timestampLast    timestamp of new heading
 * @param headingLast      new heading
 * @return                 predicted angular velocity
 */
SBasicSensor PredictAngSpeed(const TTimestamp& timestampFirst, const SBasicSensor& headingFirst, const TTimestamp& timestampLast, const SBasicSensor& headingLast);

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
SPosition    MergePosition(const SPosition& pos1, const SPosition& pos2);

} // namespace FUSION
} // namespace PE

#endif //__PE_FusionTools_H__
