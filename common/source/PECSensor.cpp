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


#include "PECSensor.h"

using namespace PE;


PE::CSensor::CSensor()
: Value(PE::MAX_VALUE)
, Accuracy(PE::MAX_ACCURACY)
{};


PE::CSensor::CSensor(const TValue& value, const TAccuracy& accuracy)
: Value(value)
, Accuracy(accuracy)
{};


bool PE::CSensor::IsValid() const
{
   return (    MAX_VALUE > Value &&
            MAX_ACCURACY > Accuracy );
};


bool PE::CSensor::operator==(const PE::CSensor& sensor)
{
   return ( Value == sensor.Value && Accuracy == sensor.Accuracy );
};


bool PE::CHeading::IsValid() const
{
   return ( EPSILON  < Accuracy &&
               45.0  > Accuracy &&
              360.0  > Value &&
                0.0 <= Value);
};


bool PE::CSpeed::IsValid() const
{
   return (      EPSILON < Accuracy &&
            MAX_ACCURACY > Accuracy &&
               MAX_VALUE > Value &&
                   Value > Accuracy);
};


bool PE::CDistance::IsValid() const
{
   return (      EPSILON < Value &&
                 EPSILON < Accuracy &&
            MAX_ACCURACY > Accuracy &&
               MAX_VALUE > Value &&
                   Value > Accuracy);
};


bool PE::CAngle::IsValid() const
{
   return ( MAX_VALUE > Value && MAX_ACCURACY > Accuracy );
};



