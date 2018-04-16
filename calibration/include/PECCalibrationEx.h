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
#ifndef __PE_CCalibrationEx_H__
#define __PE_CCalibrationEx_H__

#include <vector>
#include "PECNormalisation.h"
namespace PE
{
/**
 *  - 
 *
 */
class CCalibrationEx
{
public:
   /**
    * Constructor of calibration
    *
    * @param  normScale   instance of normalisation class for the scale
    * @param  normBase    instance of normalisation class for the base
    * @param  limit       minimum level of reference value
    *
    */
   CCalibrationEx(PE::CNormalisation& normScale, PE::CNormalisation& normBase, const PE::TValue& limit);
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
    * Returns base of the sensor
    *
    * @return    base factor
    */
   const TValue GetBase() const;
   /**
    * Returns sensor accuracy
    *
    * @return    accuracy of the sensors 1 sigma
    */
   const TValue GetAccuracy() const;
   /**
    * Returns sensor calibartion status
    *
    * @return    calibration status. range [0..100] percent based on reliable status 
    */
   const std::size GetStatus() const;

private:

   /**
    * Flag shows if reference value crossed the positive limit
    */
   bool               mStarted;
   /**
    */
   PE::TValue         mStartLimit;
   std::size          mRefCount;
   PE::TValue         mRefAccumPositive;
   PE::TValue         mRefAccumNegative;
   std::size          mSenCount;
   PE::TValue         mSenAccumPositive;
   PE::TValue         mSenAccumNegative;

   /**
    * Normalisation instance for Scale calibration
    */
   PE::CNormalisation& mNormScale;
   /**
    * Normalisation instance for Base calibration
    */
   PE::CNormalisation& mNormBase;





//    TValue mBase;
//    TValue mScale;
//    TValue mAccuracy;
//    std::size mStatus;

   
};


} //namespace PE
#endif //__PE_CCalibrationEx_H__
