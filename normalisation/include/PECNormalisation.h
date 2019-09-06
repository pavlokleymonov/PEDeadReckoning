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
#ifndef __PE_CNormalisation_H__
#define __PE_CNormalisation_H__

#include "PETypes.h"
namespace PE
{
/**
 * Finding real zero value of the sensors and its accuracy
 *
 */
class CNormalisation
{
public:
   /**
    * Constructor of normalisation
    * initial values are unset and unknowned
    */
   CNormalisation();
   /**
    * Constructor of normalisation
    *
    * @param  accumulatedValue    accumulated all values before
    * @param  accumulatedMld      accumulated all mlds before
    * @param  accumulatedReliable accumulated all reliables before
    * @param  sampleCount         sampels count before
    */
   CNormalisation(const TValue& accumulatedValue, const TValue& accumulatedMld, const TValue& accumulatedReliable, const TValue& sampleCount);
   /**
    * Adds new raw value of the sensor to the normalization
    *
    * @param  value     Raw sensor measurement
    */
   void AddSensor(const TValue& value);
   /**
    * Returns expected value of the sensor
    *
    * @return    mean value of the sensor
    */
   const TValue& GetMean() const;
   /**
    * Returns mean linear deviation of the sensor signal
    *
    * @return    deviation in unit of sensor value.
    */
   const TValue& GetMld() const;
   /**
    * Returns sensor reliable status
    *
    * @return    reliable status. range [0..100] percent.
    */
   const TValue& GetReliable() const;
   /**
    * Returns accumulated value of the sensor
    */
   const TValue& GetAccumulatedValue() const;
   /**
    * Returns accumulated mlds
    */
   const TValue& GetAccumulatedMld() const;
   /**
    * Returns accumulated reliable status
    */
   const TValue& GetAccumulatedReliable() const;
   /**
    * Returns sensors sample count
    */
   const TValue& GetSampleCount() const;

protected:
   TValue      mMean;
   TValue      mMld;
   TValue      mReliable;

   TValue      mAccumulatedValue;
   TValue      mAccumulatedMld;
   TValue      mAccumulatedReliable;
   TValue      mSampleCount;
};


} //namespace PE
#endif //__PE_CNormalisation_H__
