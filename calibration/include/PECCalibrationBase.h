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
#ifndef __PE_CCalibrationBase_H__
#define __PE_CCalibrationBase_H__

#include "PETypes.h"
#include "PECNormalisation.h"
namespace PE
{
/**
 *
 */
class CCalibrationBase
{
public:
   /**
    * Constructor of calibration
    *
    * @param  norm    instance of normalisation class for the scale
    *
    */
   CCalibrationBase(PE::CNormalisation& norm);
   /**
    * Adds new reference value to the calibration
    *
    * @param  value      reference value
    */
   void AddReference(const TValue& value);
   /**
    * Adds new sensor raw value to the calibration
    *
    * @param  value     raw sensor value
    */
   void AddSensor(const TValue& value);
   /**
    * Adds new scale to the sensor
    *
    * @param  value     latest scale value of the sensor
    */
   void AddScale(const TValue& value);
   /**
    * Returns sensor base which has to be substructed from scaled sensor value 
    *            (reference = raw_sens * scale - base)
    *
    * @return    sensor base
    */
   const TValue GetBase() const;
   /**
    * Returns sensor base which has to be substructed from scaled sensor value
    *         (reference = raw_sens * scale - base)
    *         More accurate then simple GetBase.
    *
    * @return    sensor base
    */
   const TValue GetMean() const;
   /**
    * Returns mean linear deviation of the sensor base
    *
    * @return    mld of sensor base(ecvivalent 1 sigma)
    */
   const TValue GetMld() const;
   /**
    * Returns reliable status of the sensor base
    *
    * @return    reliable status. range [0..100] percent.
    */
   const TValue GetReliable() const;
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
   /**
    * Letest sensor base which has to be substructed from scaled sensor value 
    *            (reference = raw_sens * scale - base)
    *
    */
   PE::TValue mBase;

   PE::TValue mInstRefAcc;
   uint32_t mInstRefCnt;
   PE::TValue mRefAcc;
   uint32_t mRefCnt;

   PE::TValue mInstSenAcc;
   uint32_t mInstSenCnt;
   PE::TValue mSenAcc;
   uint32_t mSenCnt;

   /**
    * Clear all intermediate instant values of sensors and references
    */
   void clearInst();
};


} //namespace PE
#endif //__PE_CCalibrationBase_H__