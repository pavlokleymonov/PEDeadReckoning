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
   CNormalisation(const double& accumulatedValue, const double& accumulatedMld, const double& accumulatedReliable, const double& sampleCount);
   /**
    * Adds new raw value of the sensor to the normalization
    *
    * @param  value     Raw sensor measurement
    */
   void AddSensor(const double& value);
   /**
    * Returns expected value of the sensor
    *
    * @return    mean value of the sensor
    */
   const double& GetMean() const;
   /**
    * Returns mean linear deviation of the sensor signal
    *
    * @return    deviation in unit of sensor value.
    */
   const double& GetMld() const;
   /**
    * Returns sensor reliable status
    *
    * @return    reliable status. range [0..100] percent.
    */
   const double& GetReliable() const;
   /**
    * Returns accumulated value of the sensor
    */
   const double& GetAccumulatedValue() const;
   /**
    * Returns accumulated mlds
    */
   const double& GetAccumulatedMld() const;
   /**
    * Returns accumulated reliable status
    */
   const double& GetAccumulatedReliable() const;
   /**
    * Returns sensors sample count
    */
   const double& GetSampleCount() const;

protected:
   double      mMean;
   double      mMld;
   double      mReliable;

   double      mAccumulatedValue;
   double      mAccumulatedMld;
   double      mAccumulatedReliable;
   double      mSampleCount;
};


} //namespace PE
#endif //__PE_CNormalisation_H__
