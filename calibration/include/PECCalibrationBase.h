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
#include "PESBasicSensor.h"
//todo move common functionality into parent class
#include "PECCalibrationScale.h"

namespace PE
{
/**
 *
 */
class CCalibrationBase: public CCalibrationScale
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
   CCalibrationBase( CNormalisation& norm, TValue ratio, TValue threshold );
   /**
    * Converts raw sansor according to calibartion status
    * returns sensor based on converted raw value and valid accuracy
    *
    * @param  raw     raw sensor data
    * @return         Scaled sensor data or invalid sensor if convertion is not possible
    */
   virtual SBasicSensor GetSensor( const SBasicSensor& raw ) const;

protected:
   TValue mRefAcc;
   TValue mRefCnt;
   TValue mRawAcc;
   TValue mRawCnt;

   /**
    * Does calibration calculation based on internal accumulated reference and raw values
    */
   virtual void DoCalibration();
};


} //namespace PE
#endif //__PE_CCalibrationBase_H__
