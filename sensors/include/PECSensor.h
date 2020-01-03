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
#ifndef __PE_CSensor_H__
#define __PE_CSensor_H__

#include "PETypes.h"
#include "PECNormalisation.h"
#include "PECCalibration.h"


namespace PE
{

/**
 * class for processing sensors data
 *
 */
class CSensor
{
public:
   /**
    * Constructor
    */
   CSensor();
   /**
    * Adds new reference data
    * @return true if reference data was accepted
    *
    * @param  refTimestamp   Timestamp of reference value [s]
    * @param  refValue       Reference value in [units]
    * @param  refAccuracy    Reference accuracy in +/-[units]
    */
   bool AddRef(const TTimestamp& refTimestamp, const TValue& refValue, const TAccuracy& refAccuracy);
   /**
    * Adds new sensor value
    * @return true if sensor data was accepted
    *
    * @param  senTimestamp   Timestamp of sensor value [s]
    * @param  senValue       Sensor value - measurement units does not matter
    * @param  senValid       True if sensors data is valid
    */
   bool AddSen(const TTimestamp& senTimestamp, const TValue& senValue, bool senValid );

private:
   /**
     * Last reference data timestamp
     */
   TTimestamp m_refTimestamp;
   /**
     * Last reference data in [units]
     */
   TValue m_refValue;
   /**
    * Last valid sensor timestamp
    */
   TTimestamp m_senTimestamp;
   /**
    * Last sensor value
    */
   TValue m_senValue;
   /**
    * Sensor calibration service
    */
   CCalibration m_SenCalib;
   /**
    * Sensor normalisation service for bias
    */
   CNormalisation m_SenBias;
   /**
    * Sensor normalisation service for scale
    */
   CNormalisation m_SenScale;
   /**
    * Resets uncomplited calibration in case some inconsistency during current sensors processing
    */
   void ResetUncomplitedProcessing();
   /**
    * Inject new bias into normalisation stuff
    *
    * @param bias   new bias value
    */
   void UpdateBias(const TValue& bias);
   /**
    * Inject new scale into normalisation stuff
    *
    * @param scale   new bias value
    */
   void UpdateScale(const TValue& scale);
};

} //namespace PE

#endif //__PE_CSensor_H__
