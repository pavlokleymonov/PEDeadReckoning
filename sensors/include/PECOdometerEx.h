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
#ifndef __PE_COdometerEx_H__
#define __PE_COdometerEx_H__

#include "PECSensor.h"


namespace PE
{

/**
 * class for processing odometers data
 *
 */
class COdometerEx : protected CSensor
{
public:
   /**
    * Constructor
    */
   COdometerEx();
   /**
    * Initialized odomer object
    * @return true if init process was done successfully
    *
    * @param  odoInterval               Odometer sensors time interval in seconds
    * @param  odoIntervalHysteresis     Hysteresis of odometer time interval in +/-[s]
    * @param  speedInterval             Reference speed time interval in seconds
    * @param  speedIntervalHysteresis   Hysteresis of speed time interval in +/-[s]
    * @param  speedAccRatio             Consider the reference speed only when value is more then accuracy by this ratio.
    *                                   For instance ratio 5 means that value is 5 times bigger then its accuracy.
    */
   bool Init(const TTimestamp& odoInterval, const TTimestamp& odoIntervalHysteresis, const TTimestamp& speedInterval, const TTimestamp& speedIntervalHysteresis, const TValue& speedAccRatio);
   /**
    * Adds new reference speed data
    * @return true if speed was accepted
    *
    * @param  timestamp   Timestamp of the speed [s]
    * @param  speed       Reference speed in [m/s]
    * @param  accuracy    Reference speed accuracy in +/-[m/s]
    */
   bool AddSpeed(const TTimestamp& timestamp, const TValue& speed, const TAccuracy& accuracy);
   /**
    * Adds new odometer ticks value
    * @return true if odometer data was accepted
    *
    * @param  timestamp   Timestamp of the odometer value [s]
    * @param  ticks       Odometer ticks number
    * @param  valid       True if sensors data is valid
    */
   bool AddTicks(const TTimestamp& timestamp, const TValue& ticks, bool valid );
   /**
    * Returns timestamp of last added sensor value.
    *         It is undefined if last AddTicks() call was unsuccessful
    * @return Sensor timestamp in seconds
    */
   const TTimestamp& GetTimeStamp() const;
   /**
    * Returns converted odometer ticks value according to reference information.
    *         Uses same units as reference value
    *         It is undefined if last AddTicks() call was unsuccessful
    * @return calculated speed based on odometer ticks value in [m/s]
    */
   const TValue GetValue() const;
   /**
    * Returns accuracy of converted odometer ticks value.
    *         Uses same units as accuracy of reference value
    *         It is undefined if last AddTicks() call was unsuccessful
    * @return calculated speed accuracy based on odometer ticks value in +/-[m/s]
    */
   const TAccuracy GetAccuracy() const;
   /**
    * Returns calibration completion status of the odometer in %
    * @return calibration status in %
    */
   const TValue& CalibratedTo() const;

protected:
   /**
    * Checks if given reference value is fit to expected conditions and accuracy
    * @return true if reference value passed all checkings
    *
    * @param refTimestamp   Timestamp of reference value [s]
    * @param refValue       Reference value in [units]
    * @param refAccuracy    Reference accuracy in +/-[units]
    */
   virtual bool IsRefOk( const TTimestamp& refTimestamp, const TValue& refValue, const TAccuracy& refAccuracy) const;
   /**
    * Checks if sensor value is fit to expected conditions and accuracy
    * @return true if sensor passed all checkings
    *
    * @param  senTimestamp   Timestamp of sensor value [s]
    * @param  senValue       Sensor value
    * @param  senValid       True if sensor is valid
    */
   virtual bool IsSenOk(const TTimestamp& senTimestamp, const TValue& senValue, bool senValid ) const;
   /**
    * Checks if reference timestamp inside range between two sensors
    * @return   true if calibration is possible
    *
    * @param senTimestamp   Timestamp of sensors value [s]
    */
   virtual bool IsCalibrationPossible( const TTimestamp& senTimestamp ) const;

private:
   /**
    * Shows if init process was done successfully
    */
   bool m_isInitOk;
   /**
    * Odometer sensors interval in seconds
    */
   TTimestamp m_odoInterval;
   /**
    * Odometer sensors time interval hysteresis in +/-[s]
    */
   TTimestamp m_odoIntervalHysteresis;
   /**
    * Reference speed interval in seconds
    */
   TTimestamp m_speedInterval;
   /**
    * Speed reference time interval hysteresis in +/-[s]
    */
   TTimestamp m_speedIntervalHysteresis;
   /**
    * Accuracy ration consider the reference speed only when value is more then accuracy by this ratio. 
    * For instance ration 5 means that value is 5 times bigger then accuracy.
    */
   TValue m_speedAccRatio;
   /**
    * Checks if delta time in a range of interval +/- hysteresis
    * IMPORTANT:   negative values have to be excluded!!!
    * @return   true if delta time passed all checkings
    *
    * @param  deltaTs      delta time for checking in second
    * @param  interval     interval of delat time in second
    * @param  hysteresis   hysteresis of delta time for given interval in second
    */
   bool IsIntervalOk(const TTimestamp& deltaTs, const TValue& interval, const TValue& hysteresis) const;
   /**
    * Checks if value is bigger than accuracy with specified ratio coefficient
    * IMPORTANT:   negative values have to be excluded!!!
    * @return   true if value passed all checkings
    *
    * @param  value      to be checked
    * @param  accuracy   accuracy of the value
    * @param  ratio      ratio coefficient
    */
   bool IsAccuracyOk(const TValue& value, const TAccuracy& accuracy, const TValue& ratio) const;

};

} //namespace PE

#endif //__PE_COdometerEx_H__
