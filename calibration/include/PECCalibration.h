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
 *  - 
 *
 */
class CCalibration
{
public:
   /**
    * Constructor of calibration
    *
    * @param  normScale         instance of normalisation class for the scale
    * @param  normBasePositive  instance of normalisation class for the base on posetive values
    * @param  normBaseNegative  instance of normalisation class for the base on negative values
    * @param  limit             minimum level of reference value
    *
    */
   CCalibration(PE::CNormalisation& normScale, PE::CNormalisation& normBasePositive, PE::CNormalisation& normBaseNegative, const PE::TValue& limit);
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
    * Returns scale factor between sensors and reference value
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
    * Returns sensor base by positive values
    *
    * @return    positive sensor base
    */
   const TValue GetBasePositive() const;
   /**
    * Returns mean linear deviation of the positive sensor base
    *
    * @return    mld of positive sensor base(ecvivalent 1 sigma)
    */
   const TValue GetBasePositiveMld() const;
   /**
    * Returns reliable status of the positive sensor base
    *
    * @return    reliable status. range [0..100] percent.
    */
   const TValue GetBasePositiveReliable() const;
   /**
    * Returns sensor base by negative values
    *
    * @return    negative sensor base
    */
   const TValue GetBaseNegative() const;
   /**
    * Returns mean linear deviation of the negative sensor base
    *
    * @return    mld of negative sensor base(ecvivalent 1 sigma)
    */
   const TValue GetBaseNegativeMld() const;
   /**
    * Returns reliable status of the negative sensor base
    *
    * @return    reliable status. range [0..100] percent.
    */
   const TValue GetBaseNegativeReliable() const;

private:

   /**
    * Flag shows if reference value crossed the absolute limit
    */
   bool               mStarted;
   /**
    */
   PE::TValue         mStartLimit;
   std::size_t        mRefCountPositive;
   PE::TValue         mRefAccumPositive;
   std::size_t        mRefCountNegative;
   PE::TValue         mRefAccumNegative;
   std::size_t        mSenCountPositive;
   PE::TValue         mSenAccumPositive;
   std::size_t        mSenCountNegative;
   PE::TValue         mSenAccumNegative;

   /**
    * Normalisation instance for Scale calibration
    */
   PE::CNormalisation& mNormScale;
   PE::CNormalisation* mpNormScale;
   /**
    * Normalisation instance for Base calibration positive values
    */
   PE::CNormalisation& mNormBasePositive;
   PE::CNormalisation* mpNormBasePositive;
   /**
    * Normalisation instance for Base calibration negative values
    */
   PE::CNormalisation& mNormBaseNegative;
   PE::CNormalisation* mpNormBaseNegative;
};


} //namespace PE
#endif //__PE_CCalibration_H__
