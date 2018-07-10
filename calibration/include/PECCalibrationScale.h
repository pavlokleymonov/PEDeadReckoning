/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2018 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */
#ifndef __PE_CCalibrationScale_H__
#define __PE_CCalibrationScale_H__

#include "PETypes.h"
#include "PECNormalisation.h"
#include "PESBasicSensor.h"

namespace PE
{
/**
 *
 */
class CCalibrationScale
{
public:
   /**
    * Constructor of calibration
    *
    * @param  norm       instance of normalisation class for the scale
    * @param  ratio      ration between reference and raw values
    *                        e.g.: 10 means 10 raw values for one reference
    * @param  threshold  threshold for ratio
    *                        e.g.: valid ratio is ratio +/- threshold
    */
   CCalibrationScale( CNormalisation& norm, TValue ratio, TValue threshold );
   /**
    * Adds new reference value to the calibration
    *
    * @param  ref      reference sensor data
    */
   void AddReference( const SBasicSensor& ref );
   /**
    * Adds new sensor raw value to the calibration
    *
    * @param  raw     raw sensor data
    */
   void AddSensor( const SBasicSensor& raw );
   /**
    * Converts raw sansor according to calibartion status
    * returns sensor based on converted raw value and valid accuracy
    *
    * @param  raw     raw sensor data
    * @return         Scaled sensor data or invalid sensor if convertion is not possible
    */
   virtual SBasicSensor GetSensor( const SBasicSensor& raw ) const;
   /**
    * Returns current persantage of calibartion
    *
    * @return         calibration status in %
    */
   const TValue& CalibratedTo() const;

protected:

   /**
    * Normalisation instance for scale calibration
    */
   CNormalisation& mNorm;
   /**
    * Ration between reference and raw values
    *    e.g.: 10 means 10 raw values for one reference
    */
   TValue mRatio;
   /**
    * threshold for ratio
    *    e.g.: valid ratio is ratio +/- threshold
    */
   TValue mThreshold;

   TValue mRefMin;
   TValue mRefMax;
   TValue mRefLast;
   TValue mRefInstAcc;
   TValue mRefInstCnt;
   TValue mRefDeltaAcc;
   TValue mRefDeltaCnt;

   TValue mRawMin;
   TValue mRawMax;
   TValue mRawLast;
   TValue mRawInstAcc;
   TValue mRawInstCnt;
   TValue mRawDeltaAcc;
   TValue mRawDeltaCnt;

   /**
    * Calculate new max, min values, returns delta between max and min values
    */
   TValue processValue(TValue& last, TValue& max, TValue& min, const TValue& value);
   /**
    * Clear all intermediate instant values of raw and references
    */
   void clearInst();
   /**
    * Returns true if ration between raw and reference samples number more then ration
    */
   bool IsOverRatio();
   /**
    * Does calibration calculation based on internal accumulated reference and raw values
    */
   virtual void DoCalibration();
};


} //namespace PE
#endif //__PE_CCalibrationScale_H__
