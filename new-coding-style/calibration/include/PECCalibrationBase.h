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
#ifndef __PE_CCallibrationBase_H__
#define __PE_CCallibrationBase_H__

#include "PETypes.h"
#include "PECNormalisation.h"
namespace PE
{
/**
 * Calculates base of the sensor. 
 * Makes sensor base calibration according to zero reference value
 * 
 * Preconditions:
 *  - all gaps in incoming values are filter outed
 *  - reference value is filtered
 *
 */
class CCalibrationBase
{
public:
   /**
    * Constructor of base calibration
    *
    * @param  norm   instance of normalisation class
    * @param  limit  accuracy for reference value, all values above this limit will be excluded from the calculation
    *
    */
   CCalibrationBase(PE::CNormalisation& norm, const TValue& limit);
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
    * @param  value     raw sensor value
    */
   void AddSensor(const TValue& value);
   /**
    * Returns base of the sensor
    *
    * @return    base factor
    */
   const TValue GetBase() const;
   /**
    * Returns sensor accuracy
    *
    * @return    accuracy of the sensors
    */
   const TValue GetAccuracy() const;
   /**
    * Returns sensor calibration status
    *
    * @return    calibration status. Range [0..100] percent based on reliable status 
    */
   const TValue GetCalibration() const;

private:
   PE::CNormalisation& mNorm;
   const TValue        mAccuracyLimit;

   bool                mLastRefZero;
   bool                mLastSenValid;
   TValue              mSenChunk;
   TValue              mSenChunkCnt;
};


} //namespace PE
#endif //__PE_CCallibrationBase_H__
