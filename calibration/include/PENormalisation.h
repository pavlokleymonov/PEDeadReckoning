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
 * Calculates null value of the sensors
 *
 */
class normalisation
{
public:
   /**
    * Constructor of normalisation
    * initial values are unset and unknown
    *
    * @param  count      minimum smaples count to get stable normalisation, has to be set based on sensor behavior
    *                    has to be defined based on statistics of the sensor
    */
   explicit normalisation(const size_t count);

   /**
    * Constructor of normalisation
    *
    * @param  init_mean  initial mean value
    * @param  init_sigma initial sigma value
    * @param  count      minimum smaples count to get stable normalisation, has to be set based on sensor behavior
    *                    has to be defined based on statistics of the sensor
    */
   normalisation(const TValue& init_mean, const TValue& init_sigma, const size_t init_reliable ,const size_t count);
   /**
    * Adds new raw value of the sensor to the normalisation
    *
    * @param  value     Raw sensor measurement
    */
   void add_sensor(const TValue& value);

   /**
    * Returns expected value of the sensor
    *
    * @return    mean value of the sensor
    */
   const TValue& get_mean() const
      {
         return m_mean;
      }
   /**
    * Returns standart deviation of the sensor signal
    *
    * @return    sigma in unit of sensor value.
    */
   const TValue& get_sigma() const
      {
         return m_sigma;
      }
   /**
    * Returns sensor reliable status
    *
    * @return    reliable status. range [0..100] percent.
    */
    size_t get_reliable() const
       {
          return m_reliable;
       }

private:
   TValue    m_mean;
   TValue    m_sigma;
   size_t    m_reliable;
   const size_t m_min_sample_count;

   TValue    m_accumulated_value;
   TValue    m_accumulated_delta_sigma;
   size_t    m_sample_count;
};


} //namespace PE
#endif //__PE_Normalisation_H__
