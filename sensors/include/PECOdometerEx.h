/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2020 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
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

#include "PETypes.h"
#include "PECSensor.h"

class PECOdometerExTest; //to get possibility for test class

namespace PE
{

/**
 * class for processing odometers data
 *
 */
class COdometerEx : public ISensorAdjuster
{

friend class ::PECOdometerExTest;

public:
   /**
    * Constructor
    */
   COdometerEx( const TValue& speedInterval,
                const TValue& speedHysteresis,
                const TValue& speedMin,
                const TValue& speedMax,
                const TValue& speedAccuracyRatio,
                const TValue& odoInterval,
                const TValue& odoHysteresis,
                const TValue& odoMin,
                const TValue& odoMax);
   /**
    * Constructor
    */
   COdometerEx();
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
    * Returns timestamp of last successfully added sensor value.
    *         It is undefined if last AddOdo() call was unsuccessful
    * @return Sensor timestamp in seconds
    */
   const TTimestamp& TimeStamp() const;
   /**
    * Returns converted odometer ticks value according to reference information.
    *         It is undefined if last AddTicks() call was unsuccessful
    * @return calculated speed based on odometer ticks value in [m/s]
    */
   const TValue Value() const;
   /**
    * Returns accuracy of converted odometer ticks value.
    *         It is undefined if last AddTicks() call was unsuccessful
    * @return calculated speed accuracy based on odometer ticks value in +/-[m/s]
    */
   const TAccuracy Accuracy() const;
   /**
    * Returns odometer bias value
    * @return   bias of the odometer
    */
   const TValue& Base() const;
   /**
    * Returns odometer scale value
    * @return   scale of the odometer sensor
    */
   const TValue& Scale() const;
   /**
    * Returns calibration completion status of the base in %
    * @return base calibration status in %
    */
   const TValue& CalibratedTo() const;

public:
   /**************************************************************************************
    * ISensorAdjuster service methods
    **************************************************************************************/

   /**
    * Sets new speed value and checks if value, interval and accuracy are fit to expected conditions
    * @return   true if it passed all checkings
    *
    * @param  oldSpeedTS   timestamp of the previouse speed in seconds [s]
    * @param  newSpeedTS   timestamp of the new speed in seconds [s]
    * @param  speed        new speed value in [m/s]
    * @param  accuracy     accuracy of new speed value in +/-[m/s]
    */
   virtual bool SetRefValue(const TTimestamp& oldSpeedTS, const TTimestamp& newSpeedTS, const TValue& speed, const TAccuracy& accuracy);
   /**
    * Just simple return of last speed value.
    *
    * @return speed value in [m/s] or NaN in case of any errors
    */
   virtual const TValue& GetRefValue() const;
   /**
    * Sets new odometer ticks and checks if the value, interval and validity are fit to expected conditions
    * @return   true if it passed all checkings
    *
    * @param  oldSpeedTS   timestamp of the previouse speed in seconds [s]
    * @param  oldTicksTS   timestamp of the previouse ticks value in seconds [s]
    * @param  newTicksTS   timestamp of the new ticks value in seconds [s]
    * @param  ticks        odometer ticks number
    * @param  valid        true if ticks number is valid
    */
   virtual bool SetSenValue(const TTimestamp& oldSpeedTS, const TTimestamp& oldTicksTS, const TTimestamp& newTicksTS, const TValue& ticks, bool valid);
   /**
    * Adjust odometer ticks number to lineral velocity which was provided by previouse call SetSenValue()
    * Second call without upfront call of SetSenValue() has to return NaN
    *
    * @return adjusted linear velocity in [m/s] or NaN in case of any errors
    */
   virtual const TValue& GetSenValue() const;

private:
   /**
    * Service for processing sensors data
    */
   CSensor m_sensor;
   /**
    * Last reference speed [m/s]
    */
   TValue m_speed;
   /**
    * Last odometer sensor ticks value
    */
   TValue m_ticks;
   /**
    * Last odometer sensor validity flag
    */
   bool m_ticksValid;
   /**
    * Last odometer ticks per second speed [ticks/s]
    */
   TValue m_ticksPerSecond;
   /**
    * Last odometer linear velocity adjusted to reference timestamp [ticks/s]
    */
   TValue m_odoLinearVelocity;

   
private:
   /**************************************************************************************
    * Constant operation limits
    **************************************************************************************/

   const TValue m_speedInterval;
   const TValue m_speedHysteresis;
   const TValue m_speedMin;
   const TValue m_speedMax;
   const TValue m_speedAccuracyRatio;
   const TValue m_odoInterval;
   const TValue m_odoHysteresis;
   const TValue m_odoMin;
   const TValue m_odoMax;
};

} //namespace PE

#endif //__PE_COdometerEx_H__
