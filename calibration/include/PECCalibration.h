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
#ifndef __PE_CCalibration_H__
#define __PE_CCalibration_H__

#include "PETypes.h"

namespace PE
{

/**
 * class for calibartion functionality
 *
 * The scaled value could be calculated by this formula:
 *
 *   value = ( raw - bias ) * scale
 *
 * IMPORTANT:  reference and raw value have to be on a same timebase without any delay or latency
 *             If there is any latency or delay it has to be compensated before calibration.
 *             Same value at startup has to be avoid.
 *
 */
class CCalibration
{
public:
   /**
    * Constructor of calibration
    */
   CCalibration();
   /**
    * Adds new reference value to the calibration
    *
    * @param  ref      reference value
    */
   void AddRef( const double& ref );
   /**
    * Adds new raw value to the calibration
    *
    * @param  raw     raw value
    */
   void AddRaw( const double& raw );
   /**
    * Returns bias/base of the calibration
    * @return         bias/base or NaN if there is no valid data
    */
   const double& GetBias() const;
   /**
    * Returns scale of the calibration
    * @return         scale or NaN if there is no valid data
    */
   const double& GetScale() const;
   /**
    * Calculates bias/base and scale of raw value.
    */
   void Recalculate();
   /**
    * Clean all data since last Recalculate call
    */
   void CleanLastStep();

private:
   /**
    * Summ of all reference data before calculation - SUM(N)
    */
   double m_Sum_Ref_before;
   /**
    * Summ of all raw data before calculation  - SUM(K)
    */
   double m_Sum_Raw_before;
   /**
    * Index of summ of reference data before calculation - N
    */
   uint32_t m_Index_before;
   /**
    * Summ of all reference data during calculation - SUM(N+1)
    */
   double m_Sum_Ref_now;
   /**
    * Summ of all raw data during calculation  - SUM(K+1)
    */
   double m_Sum_Raw_now;
   /**
    * Index of summ of reference data during calculation - N+1
    */
   uint32_t m_Index_now;
   /**
    * Bias of the raw data
    */
   double m_Bias;
   /**
    * Scale of the raw data
    */
   double m_Scale;
   /**
    * Calculates bias/base and scale of raw value.
    * Did not check first iteration
    */
   void CalculateBaseScale();
};

} //namespace PE

#endif //__PE_CCalibration_H__
