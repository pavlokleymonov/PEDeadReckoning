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
    * @param  norm    instance of normalisation class for the scale
    *
    */
   CCalibrationScale(PE::CNormalisation& norm);
   /**
    * Adds new reference value to the calibration
    *
    * @param  value      value of reference source
    */
   void AddReference(const TValue& value);
   /**
    * Adds new sensor raw value to the calibration
    *
    * @param  value     raw sensor value
    */
   void AddSensor(const TValue& value);
   /**
    * Returns scale factor between sensor and reference values based on simple average
    *         of delta between them
    *
    * @return    scale factor
    */
   const TValue& GetScale() const;
   /**
    * Does calibration calculation based on internal accumulated reference and sensor values
    */
   void DoCalibration();
   /**
    * Resets internal state to be able to process new data without dependencies for previouse one.
    * It would be usefull after gap detection in any income data.
    */
   void Reset();

private:

   /**
    * Normalisation instance for Scale calibration
    */
   PE::CNormalisation& mNorm;
   /**
    * Latest simple average of delta between sensor and reference values
    */
   PE::TValue mScale;

   PE::TValue mRefMin;
   PE::TValue mRefMax;
   PE::TValue mRefLast;
   PE::TValue mInstRefAcc;
   uint32_t mInstRefCnt;
   PE::TValue mRefDelatAcc;
   uint32_t mRefDeltaCnt;

   PE::TValue mSenMin;
   PE::TValue mSenMax;
   PE::TValue mSenLast;
   PE::TValue mInstSenAcc;
   uint32_t mInstSenCnt;
   PE::TValue mSenDeltaAcc;
   uint32_t mSenDeltaCnt;

   /**
    * Calculate new max, min values, returns delta between max and min values
    */
   TValue processValue(TValue& last, TValue& max, TValue& min, const TValue& value);
   /**
    * Clear all intermediate instant values of sensors and references
    */
   void clearInst();
};


} //namespace PE
#endif //__PE_CCalibrationScale_H__
