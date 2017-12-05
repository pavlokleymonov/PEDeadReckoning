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
#ifndef __PE_CCallibration_H__
#define __PE_CCallibration_H__

#include <vector>
#include "PECNormalisation.h"
#include "PECCalibrationScale.h"
#include "PECCalibrationBase.h"
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
    * @param  calibQueueLength   calibration queue length
    *                            0 - unlimited
    */
   CCalibration( std::size_t calibQueueLength );
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
    * Returns sensor calibartion status
    *
    * @return    calibration status. range [0..100] percent based on reliable status 
    */
   const std::size GetStatus() const;

private:
   CNormalisation& mNormScale;
   CNormalisation& mNormBase;

   std::vector<CCalibrationBase*>  mBaseQueue;
   std::vector<CCalibrationScale*> mScaleQueue;

   TValue mBase;
   TValue mScale;
   TValue mAccuracy;
   std::size mStatus;

   /**
    * Checks base queue and calculates new base
    *
    */
   void AdjustBase();
   /**
    * Checks scale queue and calculates new base
    *
    */
   void AdjustScale();
};


} //namespace PE
#endif //__PE_CCallibration_H__
