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
#ifndef __PE_CGyroscope_H__
#define __PE_CGyroscope_H__

#include "PETypes.h"
#include "PECNormalisation.h"
#include "PECCalibration.h"


namespace PE
{

/**
 * class for processing gyroscope sensors data
 *
 */
class CGyroscope
{
public:
   /**
    * Constructor
    */
   CGyroscope();
   /**
    * Adds new reference heading
    * @return true if reference data was accepted
    *
    * @param  ts     Timestamp of heading in [s]
    * @param  head   Heading in [deg] with reference to true north, 0.0 -> north, 90.0 -> east, 180.0 south, 270.0 -> west
    * @param  acc    Heading accuracy in +/-[deg]
    */
   bool AddHeading(const TTimestamp& ts, const TValue& head, const TAccuracy& acc);
   /**
    * Adds new gyroscope sensor value
    * @return true if sensor data was accepted
    *
    * @param  ts        Timestamp of gyro sensor value [s]
    * @param  gyro      Gyroscope angular velicity in [units/s]
    * @param  isValid   True if sensors data is valid
    */
   bool AddGyro(const TTimestamp& ts, const TValue& gyro, bool isValid );
   /**
    * Returns timestamp of last successfully added gyroscope sensor value.
    *         It is undefined if last AddGyro() call was unsuccessful
    * @return Sensor timestamp in seconds
    */
   const TTimestamp& GetTimeStamp() const;
   /**
    * Returns converted gyroscope angular velocity according to reference information.
    *         It is undefined if last AddGyro() call was unsuccessful
    * @return calculated angular velocity based on gyroscope sensor data in [deg/s]
    */
   const TValue GetValue() const;
   /**
    * Returns accuracy of converted gyroscope sensor data.
    *         It is undefined if last AddGyro() call was unsuccessful
    * @return calculated accuracy of angular velocity based on gyroscope sensor data in +/-[deg/s]
    */
   const TAccuracy GetAccuracy() const;
   /**
    * Returns gyroscope bias value
    * @return   bias of the gyroscope
    */
   const TValue& GetBias() const;
   /**
    * Returns calibration completion status of bias in %
    * @return   bias calibration status in %
    */
   const TValue& BiasCalibartedTo() const;
   /**
    * Returns gyroscope  scale value
    * @return   scale of the gyroscope sensor
    */
   const TValue& GetScale() const;
   /**
    * Returns calibration completion status of scale in %
    * @return   scale calibration status in %
    */
   const TValue& ScaleCalibartedTo() const;

private:
   /**
     * Last reference heading value in [deg]
     */
   TValue m_headValue;
   /**
     * Last reference heading accuracy in +/-[deg]
     */
   TAccuracy m_headAccuracy;
   /**
     * Last reference data timestamp in [s]
     */
   TTimestamp m_refTimestamp;
   /**
     * Last reference data in [units]
     */
   TValue m_refValue;
   /**
    * Last valid sensor timestamp in [s]
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
    * Resets uncomplited calibration in case some inconsistency during current sensors processing
    */
   void ResetUncomplitedProcessing();
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

#endif //__PE_CGyroscope_H__
