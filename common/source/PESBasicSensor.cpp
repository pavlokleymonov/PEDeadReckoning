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


#include "PESBasicSensor.h"

using namespace PE;


PE::SBasicSensor::SBasicSensor()
: Value(PE::MAX_VALUE)
, Accuracy(PE::MAX_ACCURACY)
{};


PE::SBasicSensor::SBasicSensor(const TValue& value, const TAccuracy& accuracy)
: Value(value)
, Accuracy(accuracy)
{};


bool PE::SBasicSensor::IsValid() const
{
   return ( MAX_VALUE > Value && 
            MAX_ACCURACY > Accuracy );
};


bool PE::operator==(const PE::SBasicSensor& lhs, const PE::SBasicSensor& rhs)
{
   return ( lhs.Value == rhs.Value &&
            lhs.Accuracy == rhs.Accuracy );
};
