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
protected: //It is only possible to have a child class. Should not be explicitly instantiated
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
   /**
    * Checks if given reference value is fit to expected conditions and accuracy
    *
    * IMPORTANT: It is pure virtual function. Has to be overridden by successor class.
    *
    * @return true if reference value passed all checkings
    *
    * @param refTimestamp   Timestamp of reference value [s]
    * @param refValue       Reference value in [units]
    * @param refAccuracy    Reference accuracy in +/-[units]
    */
   virtual bool IsRefOk( const TTimestamp& refTimestamp, const TValue& refValue, const TAccuracy& refAccuracy) const = 0;
   /**
    * Checks if sensor value is fit to expected conditions and accuracy
    *
    * IMPORTANT: It is pure virtual function. Has to be overridden by successor class.
    *
    * @return true if sensor passed all checkings
    *
    * @param  senTimestamp   Timestamp of sensor value [s]
    * @param  senValue       Sensor value
    * @param  senValid       True if sensor is valid
    */
   virtual bool IsSenOk(const TTimestamp& senTimestamp, const TValue& senValue, bool senValid ) const = 0;
   /**
    * Checks if reference timestamp inside range between two sensors
    *
    * IMPORTANT: It is pure virtual function. Has to be overridden by successor class.
    *
    * @return   true if calibration is possible
    *
    * @param senTimestamp   Timestamp of sensors value [s]
    */
   virtual bool IsCalibrationPossible( const TTimestamp& senTimestamp ) const = 0;

protected:
   /**
     * Last reference data timestamp
     */
    TTimestamp m_refTimestamp;
   /**
     * Last reference data in [units]
     */
    TTimestamp m_refValue;
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

private:
   /**
    * Resets uncomplited calibration in case some inconsistency during current sensors processing
    */
   void ResetUncomplitedProcessing();
   /**
    * Predict sensor value which is fit to the timestamp of the reference value
    * @return sensor value
    *
    * @param  senTimestamp   Timestamp of sensor value [s]
    * @param  senValue       Sensor value - measurement units does not matter
    */
   TValue PredictSensorValue( const TTimestamp& senTimestamp, const TValue& senValue ) const;
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
