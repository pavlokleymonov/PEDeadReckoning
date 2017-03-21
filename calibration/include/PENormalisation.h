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
    *
    * @param  init_null     initial quiet zero value of the sensor
    */
   normalisation(const TValue& init_null);
   /**
    * Destructor of normalisation
    *
    */
   virtual ~normalisation();
   /**
    * Adds new sensor steps to the normalisation
    *
    * @param  sensor_steps     sensor steps
    */
   virtual void add_sensor_steps(const TValue& sensor_steps);
   /**
    * Returns null value of the sensor
    *         This means that null values coresponds to sensor value in quiet state
    *
    * @return    quiet zero value of the sensor
    */
   virtual const TValue& get_null();
   /**
    * Returns sensor reliable status
    *
    * @return    reliable status. range [0..100] percent.
    */
   const TValue& get_null_reliable();

private:
   TValue    m_null;
   TValue    m_reliable;
   TValue    m_sensor_accumulated_steps;
   TValue    m_sensor_accumulated_count;
};


} //namespace PE
#endif //__PE_Normalisation_H__
