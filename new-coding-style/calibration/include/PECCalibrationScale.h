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
#ifndef __PE_CCallibrationScale_H__
#define __PE_CCallibrationScale_H__

#include "PETypes.h"
#include "PECNormalisation.h"
namespace PE
{
/**
 * Makes calibration of incoming sensor steps to the reference value
 * extended algorithm based on normalization.
 * Preconditions:
 *  - all gaps in incoming values are filter outed
 *  - reference value is filtered
 *  - sensor value is adjusted to the base of zero
 *
 */
class CCalibrationScale
{
public:
   /**
    * Constructor of Scale calibration
    *
    * @param  norm   instance of normalisation class
    * @param  limit  accuracy for reference value, all values above this limit will be excluded from the calculation
    *
    */
   CCalibrationScale(PE::CNormalisation& norm, const TValue& limit);
   /**
    * Adds new reference value to the calibration
    *
    * @param  value      values of reference value
    * @param  accuracy   accuracy of the reference value
    */
   void AddReference(const TValue& value, const TValue& accuracy);
   /**
    * Adds new sensor raw value to the calibration
    *
    * @param  value     raw sensor value, zero offset has to be adjusted before
    */
   void AddSensor(const TValue& value);
   /**
    * Returns scale factor between sensors and reference value
    *
    * @return    scale factor
    */
   const TValue GetScale() const;
   /**
    * Returns sensor accuracy
    *
    * @return    accuracy of the sensors
    */
   const TValue GetAccuracy() const;
   /**
    * Returns sensor calibartion status
    *
    * @return    calibration status. range [0..100] percent based on reliable status 
    */
   const TValue GetCalibration() const;

private:
   PE::CNormalisation& mNorm;

   const TValue        mAccuracyLimit;

   bool                mLastRefValid;
   bool                mLastSenValid;

   TValue              mRefAccumulated;
   TValue              mSenAccumulated;
   TValue              mSenChunk;

   /**
    * Calculates scale factor
    */
   void CalcScale(const TValue& value);
};


} //namespace PE
#endif //__PE_CCallibrationScale_H__
