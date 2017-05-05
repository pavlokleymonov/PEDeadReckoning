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
#ifndef __PE_CallibrationEx_H__
#define __PE_CallibrationEx_H__

#include "PETypes.h"
namespace PE
{
/**
 * Makes calibartion of incoming sensor steps to the reference value
 * extended algorithm
 *
 */
class calibration_ex
{
public:
   /**
    * Constructor of calibration
    *
    */
   calibration_ex(const TValue& ref_limit, const size_t min_ref_count);
   /**
    * Adds new reference value to the calibration
    *
    * @param  ref_value      values of reference value
    * @param  ref_accuracy   accuracy of the reference value
    */
   void add_reference(const TValue& ref_value, const TValue& ref_accuracy);
   /**
    * Adds new sensor raw value to the calibration
    *
    * @param  sen_value     raw sensor value
    */
   void add_sensor(const TValue& sen_value);
   /**
    * Returns scale factor between sensors and reference value
    *
    * @return    scale factor
    */
   const TValue& get_scale() const
      {
         return m_scale;
      }
   /**
    * Returns sensor accuracy
    *
    * @return    accuracy of the sensors according to reference value
    */
   const TValue& get_accuracy() const
      {
         return m_accuracy;
      }
   /**
    * Returns sensor calibartion status
    *
    * @return    calibration status. range [0..100] percent.
    */
   const TValue& get_calibration() const
      {
         return m_calibration;
      }

private:
   TValue     m_scale;
   TValue     m_calibration;
   TValue     m_accuracy;


   const TValue  m_reference_limit;
   const size_t  m_minimum_reference_count;


   TValue     m_reference_accumulated;
   TValue     m_sensor_accumulated;

   TAccuracy  m_accuracy_accumulated;
   size_t     m_reference_count;


   size_t m_reference_count;

};


} //namespace PE
#endif //__PE_CallibrationEx_H__
