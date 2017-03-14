/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2017 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file Copyright.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */
#ifndef __PE_State_H__
#define __PE_State_H__

#include "PETypes.h"
namespace PE
{

/**
 * Makes calibartion of incoming sensor value to the reference value
 *
 */
class calibration
{
public:
   /**
    * Calcs sensor calibration status. range [0..100] percent.
    *
    * @param  old_values_per_step   old values per one step
    * @param  new_values_per_step   new values per one step
    */
   static TValue calc_callibration(const TValue& old_values_per_step, const TValue& new_values_per_step);
   /**
    * Constructor of calibration
    *
    * @param  init_values_per_step     initial values per one step of the sensor
    * @param  accuracy_limit           accuracy limit till wich all incoming values will not be used
    */
   calibration(const TValue& init_values_per_step, const TAccuracy& accuracy_limit);
   /**
    * Adds new reference value to the calibration
    *
    * @param  reference_values     values of reference signal
    * @param  reference_accuracy   accuracy of the reference signal
    */
   void add_reference_value(const TValue& reference_values, const TAccuracy& reference_accuracy);
   /**
    * Adds new sensor steps to the calibration
    *
    * @param  sensor_steps     sensor steps
    * @param  sensor_accuracy  accuracy of the sensor signal
    */
   void add_sensor_steps(const TValue& sensor_steps, const TAccuracy& sensor_accuracy);
   /**
    * Returns reference values per one sensor step
    *
    * @return    how many values per one sensor step
    */
   const TValue& get_values_per_step() const
      {
         return m_values_per_step;
      }
   /**
    * Returns sensor accuracy
    *
    * @return    accuracy of the sensors according to values
    */
   const TAccuracy& get_sensor_accuracy() const
      {
         return m_sensor_accuracy;
      }
   /**
    * Returns sensor calibartion status
    *
    * @return    calibration status. range [0..100] percent.
    */
   const TValue& get_sensor_calibration() const
      {
         return m_sensor_calibration;
      }
private:
   TValue    m_values_per_step;
   TAccuracy m_sensor_accuracy;
   TValue    m_sensor_calibration;
   TAccuracy m_accuracy_limit;

   TValue    m_reference_accumulated_values;
   TAccuracy m_reference_accumulated_accuracy;
   TValue    m_reference_accumulated_count;

   TValue    m_sensor_accumulated_steps;
   TAccuracy m_sensor_accumulated_accuracy;
   TValue    m_sensor_accumulated_count;

   void _clear_reference();
   TAccuracy _get_reference_accuracy();

   void _clear_sensor();
   TAccuracy _get_sensor_accuracy();


   void _process();
};


} //namespace PE
#endif //__PE_State_H__
