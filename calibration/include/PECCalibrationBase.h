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
#ifndef __PE_CCalibrationBase_H__
#define __PE_CCalibrationBase_H__

#include "PETypes.h"
#include "PECNormalisation.h"
namespace PE
{
/**
 *  All sensors data have to be adjusted to reference value frequancy
 *  Mean value has to be calculated from all sensors data in a time range between two referenc values
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
    * @param  value      values of reference value
    */
   void AddReference(const TValue& value);
   /**
    * Adds new sensor raw value to the calibration
    *
    * @param  value     raw sensor value, zero offset has to be adjusted before
    */
   void AddSensor(const TValue& value);
   /**
    * Adds new scale to the sensor
    *
    * @param  value     scale value of the sensor
    */
   void AddScale(const TValue& value);
   /**
    * Returns sensor base which has to be substructed from scaled sensor value
    *
    * @return    sensor base
    */
   const TValue GetBase() const;
   /**
    * Returns sensor base which has to be substructed from scaled sensor value
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

private:

   /**
    * Normalisation instance for Scale calibration
    */
   PE::CNormalisation& mNorm;
   PE::TValue mBase;
   PE::TValue mScale;
   PE::TValue mRefAcc;
   uint32_t mRefCnt;
   PE::TValue mSenAcc;
   uint32_t mSenCnt;
};


} //namespace PE
#endif //__PE_CCalibrationBase_H__
