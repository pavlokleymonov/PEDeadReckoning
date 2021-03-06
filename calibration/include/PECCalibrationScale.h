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

#include "PECCalibration.h"

namespace PE
{
/**
 *
 */
class CCalibrationScale: public CCalibration
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
   CCalibrationScale( CNormalisation* norm, TValue ratio, TValue threshold );
   /**
    * Destructor of calibartion
    */
   virtual ~CCalibrationScale();
   /**
    * Converts raw sansor according to calibartion status
    * returns sensor based on converted raw value and valid accuracy
    *
    * @param  raw     raw sensor data
    * @return         Scaled sensor data or invalid sensor if convertion is not possible
    */
   virtual SBasicSensor GetSensor( const SBasicSensor& raw ) const;

protected:


   TValue mRefMin;
   TValue mRefMax;
   TValue mRefLast;
   TValue mRefDeltaAcc;
   TValue mRefDeltaCnt;

   TValue mRawMin;
   TValue mRawMax;
   TValue mRawLast;
   TValue mRawDeltaAcc;
   TValue mRawDeltaCnt;

   /**
    * Does calibration calculation based on internal accumulated reference and raw values
    */
   virtual void DoCalibration();
};


} //namespace PE
#endif //__PE_CCalibrationScale_H__
