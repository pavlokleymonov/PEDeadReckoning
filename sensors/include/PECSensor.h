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
 * class for adjusting and checking input reference/sensor data
 *
 */
class ISensorAdjuster
{
public:
   /**
    * Sets new reference value and checks if value, interval and accuracy are fit to expected conditions
    * @return   true if it passed all checkings
    *
    * @param  oldRefTimestamp   timestamp of the previouse reference value in seconds [s]
    * @param  newRefTimestamp   timestamp of the new reference value in seconds [s]
    * @param  refValue          reference value in [units]
    * @param  refAccuracy       reference value accuracy in +/-[units]
    */
   virtual bool SetRefValue(const TTimestamp& oldRefTimestamp, const TTimestamp& newRefTimestamp, const TValue& refValue, const TAccuracy& refAccuracy) = 0;
   /**
    * Returns adjusted into the velocity reference value which was prepared by previouse call SetRefValue()
    *    For instance: heading into angular velocity[deg/s] or distance to linear velocity[m/s]
    *
    * @return adjusted value in corespondednt units or NaN in case of any errors
    */
   virtual const TValue& GetRefValue() const = 0;
   /**
    * Sets new sensor value and checks if value, interval and validity are fit to expected conditions
    * @return   true if it passed all checkings
    *
    * @param  oldRefTimestamp   timestamp of the previouse reference value in seconds [s]
    * @param  oldSenTimestamp   timestamp of the previouse sensor value in seconds [s]
    * @param  newSenTimestamp   timestamp of the new sensor value in seconds [s]
    * @param  senValue          sensor value in [units]
    * @param  IsValid           true if sensor is valid
    */
   virtual bool SetSenValue(const TTimestamp& oldRefTimestamp, const TTimestamp& oldSenTimestamp, const TTimestamp& newSenTimestamp, const TValue& senValue, bool IsValid) = 0;
   /**
    * Returns adjusted into the velocity sensor value which was provided by previouse call SetSenValue()
    *    For instance: odometer ticks into linear velocity[ticks/s]
    *
    * @return adjusted value in corespondednt units or NaN in case of any errors
    */
   virtual const TValue& GetSenValue() const = 0;
};


/**
 * class for processing sensors data
 *
 */
class CSensor
{
public:
   /**
    * Constructor
    *
    * @param  adjuster   Reference to the adjuster instance
    */
   explicit CSensor(ISensorAdjuster& adjuster);
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
    * @return   last reference data timestamp
    */
   const TTimestamp& GetRefTimeStamp() const;
   /**
    * @return   last sensor data timestamp
    */
   const TTimestamp& GetSenTimeStamp() const;
   /**
    * @return   Sensor normalisation service for bias
    */
   const CNormalisation& GetBias() const;
   /**
    * @return   Sensor normalisation service for scale
    */
   const CNormalisation& GetScale() const;

private:
   /**
    * Rference to sensor adjuster instance
    */
   ISensorAdjuster& m_adjuster;
   /**
     * Last reference data timestamp
     */
   TTimestamp m_refTimestamp;
   /**
    * Last valid sensor timestamp
    */
   TTimestamp m_senTimestamp;
   /**
    * Sensor calibration service
    */
   CCalibration m_Calibration;
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
    * Adds reference value into calibration service
    *
    * @param  refValue       Reference value in [units]
    */
   void UpdateCalibrationRef(const TValue& refValue);
   /**
    * Adds sensor value into calibration service
    *
    * @param  senValue       Sensor value - measurement units does not matter
    */
   void UpdateCalibrationSen(const TValue& senValue);
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
