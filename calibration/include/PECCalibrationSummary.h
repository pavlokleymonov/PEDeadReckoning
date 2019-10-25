/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2019 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */
#ifndef __PE_CCalibrationSummary_H__
#define __PE_CCalibrationSummary_H__

#include "PEICalibration.h"

namespace PE
{

/**
 * class for calibartion functionality
 *
 * IMPORTANT:  reference and raw value have to be on a same timebase without any delay or latency
 *             If there is any latency or delay it has to be compensated before calibration.
 *             Same value at startup has to be avoid.
 *
 */
class CCalibrationSummary : public ICalibration
{
public:
   /**
    * Constructor of calibartion
    */
   CCalibrationSummary();
   /**
    * Destructor of calibartion
    */
   virtual ~CCalibrationSummary();
   /**
    * Adds new reference value to the calibration
    *
    * @param  ref      reference value
    */
   virtual void AddRef( const TValue& ref );
   /**
    * Adds new raw value to the calibration
    *
    * @param  raw     raw value
    */
   virtual void AddRaw( const TValue& raw );
   /**
    * Returns bias/base of the calibration
    * @return         bias/base or NaN if there is no valid data
    */
   virtual const TValue& GetBias() const;
   /**
    * Returns scale of the calibration
    * @return         scale or NaN if there is no valid data
    */
   virtual const TValue& GetScale() const;
   /**
    * Calculates bias/base and scale of raw value.
    */
   virtual void Recalculate();

private:

   TValue m_Sum_Ref_before;
   TValue m_Sum_Raw_before;
   uint32_t m_Index_before;

   TValue m_Sum_Ref_now;
   TValue m_Sum_Raw_now;
   uint32_t m_Index_now;

   TValue m_Divisor;

   TValue m_Bias;
   TValue m_Scale;

   /**
    * Calculates bias/base and scale of raw value.
    * Did not check first iteration
    */
   void CalculateBaseScale();
};

} //namespace PE

#endif //__PE_CCalibrationSummary_H__
