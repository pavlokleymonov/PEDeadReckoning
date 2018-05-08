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
#ifndef __PE_CCalibration_H__
#define __PE_CCalibration_H__

#include "PETypes.h"
#include "PECNormalisation.h"
namespace PE
{
/**
 *  All sensors data have to be adjusted to reference value frequancy
 *  Mean value has to be calculated from all sensors data in a time range between two referenc values
 *
 */
class CCalibration
{
public:
   /**
    * Constructor of calibration
    *
    * @param  normScale    instance of normalisation class for the scale
    * @param  normBase     instance of normalisation class for the base
    *
    */
   CCalibration(PE::CNormalisation& normScale, PE::CNormalisation& normBase);
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
    * Returns scale factor between sensors and reference value,
    *         it has to be multiply to sensor value after substruction of the base
    *
    * @return    scale factor
    */
   const TValue GetScale() const;
   /**
    * Returns mean linear deviation of the scale factor
    *
    * @return    mld of scale factor(ecvivalent 1 sigma)
    */
   const TValue GetScaleMld() const;
   /**
    * Returns reliable status of the scale
    *
    * @return    reliable status. range [0..100] percent.
    */
   const TValue GetScaleReliable() const;
   /**
    * Returns sensor base which has to be substructed from original sensor value
    *
    * @return    sensor base
    */
   const TValue GetBase() const;
   /**
    * Returns mean linear deviation of the sensor base
    *
    * @return    mld of sensor base(ecvivalent 1 sigma)
    */
   const TValue GetBaseMld() const;
   /**
    * Returns reliable status of the sensor base
    *
    * @return    reliable status. range [0..100] percent.
    */
   const TValue GetBaseReliable() const;
   /**
    *
    */
   void DoCalibration();

private:

   /**
    * Normalisation instance for Scale calibration
    */
   PE::CNormalisation& mNormScale;
   /**
    * Normalisation instance for Base calibration positive values
    */
   PE::CNormalisation& mNormBase;

   PE::TValue mRefMin;
   PE::TValue mRefMax;
   PE::TValue mRefLast;
   PE::TValue mInstRefAcc;
   uint32_t mInstRefCnt;
   PE::TValue mRefAcc;
   uint32_t mRefCnt;
   PE::TValue mRefDelatAcc;
   uint32_t mRefDeltaCnt;
   
   PE::TValue mSenMin;
   PE::TValue mSenMax;
   PE::TValue mSenLast;
   PE::TValue mInstSenAcc;
   uint32_t mInstSenCnt;
   PE::TValue mSenAcc;
   uint32_t mSenCnt;
   PE::TValue mSenDeltaAcc;
   uint32_t mSenDeltaCnt;

   void processValue(TValue& last, TValue& max, TValue& min, const TValue& value);
   void processScale();
   void processBase();
   void cleanInst();
};


} //namespace PE
#endif //__PE_CCalibration_H__
