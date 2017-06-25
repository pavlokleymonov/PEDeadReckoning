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
#ifndef __PE_CallibrationScale_H__
#define __PE_CallibrationScale_H__

#include "PETypes.h"
#include "PENormalisation.h"
namespace PE
{
/**
 * Makes calibartion of incoming sensor steps to the reference value
 * extended algorithm based on normalization.
 * Preconditions:
 *  - all gaps in incoming values are filter outed
 *  - reference value is filtered
 *  - sensor value is adjusted to offset of zero
 *
 */
class calibration_scale
{
public:
   /**
    * Constructor of Scale calibration
    *
    * @param  norm            instance of normalisation class
    * @param  accuracy_limit  accuracy for reference value, all values above this limit will be excluded from the calculation
    *
    */
   calibration_scale(PE::normalisation& norm, const TValue& accuracy_limit);
   /**
    * Adds new reference value to the calibration
    *
    * @param  value      values of reference value
    * @param  accuracy   accuracy of the reference value
    */
   void add_reference(const TValue& value, const TValue& accuracy);
   /**
    * Adds new sensor raw value to the calibration
    *
    * @param  value     raw sensor value, zero offset has to be adjusted before
    */
   void add_sensor(const TValue& value);
   /**
    * Returns scale factor between sensors and reference value
    *
    * @return    scale factor
    */
   const TValue get_scale() const;
   /**
    * Returns sensor accuracy
    *
    * @return    accuracy of the sensors which is equal to three sigmas
    */
   const TValue get_accuracy() const;
   /**
    * Returns sensor calibartion status
    *
    * @return    calibration status. range [0..100] percent based on reliable status 
    */
   const TValue get_calibration() const;

private:
   PE::normalisation& m_norm;

   const TValue       m_accuracy_limit;

   bool               m_last_ref_valid;
   bool               m_last_sen_valid;

   TValue             m_ref_accumulated;
   TValue             m_sen_accumulated;
   TValue             m_sen_chunk;
};


} //namespace PE
#endif //__PE_CallibrationScale_H__
