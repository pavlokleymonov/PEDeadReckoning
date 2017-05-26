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
#ifndef __PE_Normalisation_H__
#define __PE_Normalisation_H__

#include "PETypes.h"
namespace PE
{
/**
 * Finding real zero value of the sensors and its accuracy
 *
 */
class normalisation
{
public:
   /**
    * Constructor of normalisation
    * initial values are unset and unknowned
    */
   normalisation();
   /**
    * Constructor of normalisation
    *
    * @param  accumulated_value    accumulated all values before
    * @param  accumulated_sigma    accumulated all sigmas before
    * @param  accumulated_reliable accumulated all reliables before
    * @param  sample_count         sampels count before
    */
   normalisation(const TValue& accumulated_value, const TValue& accumulated_sigma, const TValue& accumulated_reliable, const TValue& sample_count);
   /**
    * Destructor of normalisation
    */
   virtual ~normalisation();
   /**
    * Adds new raw value of the sensor to the normalization
    *
    * @param  value     Raw sensor measurement
    */
   virtual void add_sensor(const TValue& value);
   /**
    * Returns expected value of the sensor
    *
    * @return    mean value of the sensor
    */
   const TValue& get_mean() const;
   /**
    * Returns standart deviation of the sensor signal
    *
    * @return    sigma in unit of sensor value.
    */
   const TValue& get_sigma() const;
   /**
    * Returns sensor reliable status
    *
    * @return    reliable status. range [0..100] percent.
    */
   const TValue& get_reliable() const;
   /**
    * Returns accumulated value of the sensor
    */
   const TValue& get_accumulated_value() const;
   /**
    * Returns accumulated sigmas
    */
   const TValue& get_accumulated_sigma() const;
   /**
    * Returns accumulated reliable status
    */
   const TValue& get_accumulated_reliable() const;
   /**
    * Returns sensors sample count
    */
   const TValue& get_sample_count() const;

protected:
   TValue    m_mean;
   TValue    m_sigma;
   TValue    m_reliable;

   TValue    m_accumulated_value;
   TValue    m_accumulated_sigma;
   TValue    m_accumulated_reliable;
   TValue    m_sample_count;
};


} //namespace PE
#endif //__PE_Normalisation_H__
