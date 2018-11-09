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
#ifndef __PE_CCalibrationUni_H__
#define __PE_CCalibrationUni_H__

#include "PECCalibration.h"

namespace PE
{
/**
 *
 */
class CCalibrationUni: public CCalibration
{
public:
   /**
    * Constructor of calibration
    *
    * @param  normScale    instance of normalisation class for the scale
    * @param  normBase     instance of normalisation class for the base
    * @param  ratio        ration between reference and raw values
    *                         e.g.: 10 means 10 raw values for one reference
    * @param  threshold    threshold for ratio
    *                        e.g.: valid ratio is ratio +/- threshold
    */
   CCalibrationUni( CNormalisation* normScale, CNormalisation* normBase, TValue ratio, TValue threshold );
   /**
    * Destructor of calibartion
    */
   virtual ~CCalibrationUni();
   /**
    * Converts raw sansor according to calibartion status
    * returns sensor based on converted raw value and valid accuracy
    *
    * @param  raw     raw sensor data
    * @return         Converted sensor data or invalid sensor if convertion is not possible
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
