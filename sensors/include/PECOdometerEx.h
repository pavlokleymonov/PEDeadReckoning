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
   COdometerEx( const double& speedInterval,
                const double& speedHysteresis,
                const double& speedMin,
                const double& speedMax,
                const double& speedAccuracyRatio,
                const double& odoInterval,
                const double& odoHysteresis,
                const double& odoMin,
                const double& odoMax);
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
   bool AddSpeed(const double& timestamp, const double& speed, const double& accuracy);
   /**
    * Adds new odometer ticks value
    * @return true if odometer data was accepted
    *
    * @param  timestamp   Timestamp of the odometer value [s]
    * @param  ticks       Odometer ticks number
    * @param  valid       True if sensors data is valid
    */
   bool AddTicks(const double& timestamp, const double& ticks, bool valid );
   /**
    * Returns timestamp of last successfully added sensor value.
    *         It is undefined if last AddOdo() call was unsuccessful
    * @return Sensor timestamp in seconds
    */
   const double& TimeStamp() const;
   /**
    * Returns converted odometer ticks value according to reference information.
    *         It is undefined if last AddTicks() call was unsuccessful
    * @return calculated speed based on odometer ticks value in [m/s]
    */
   const double Value() const;
   /**
    * Returns accuracy of converted odometer ticks value.
    *         It is undefined if last AddTicks() call was unsuccessful
    * @return calculated speed accuracy based on odometer ticks value in +/-[m/s]
    */
   const double Accuracy() const;
   /**
    * Returns odometer bias value
    * @return   bias of the odometer
    */
   const double& Base() const;
   /**
    * Returns odometer scale value
    * @return   scale of the odometer sensor
    */
   const double& Scale() const;
   /**
    * Returns calibration completion status of the base in %
    * @return base calibration status in %
    */
   const double& CalibratedTo() const;

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
   virtual bool SetRefValue(const double& oldSpeedTS, const double& newSpeedTS, const double& speed, const double& accuracy);
   /**
    * Just simple return of last speed value.
    *
    * @return speed value in [m/s] or NaN in case of any errors
    */
   virtual const double& GetRefValue() const;
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
   virtual bool SetSenValue(const double& oldSpeedTS, const double& oldTicksTS, const double& newTicksTS, const double& ticks, bool valid);
   /**
    * Adjust odometer ticks number to lineral velocity which was provided by previouse call SetSenValue()
    * Second call without upfront call of SetSenValue() has to return NaN
    *
    * @return adjusted linear velocity in [m/s] or NaN in case of any errors
    */
   virtual const double& GetSenValue() const;

private:
   /**
    * Service for processing sensors data
    */
   CSensor m_sensor;
   /**
    * Last reference speed [m/s]
    */
   double m_speed;
   /**
    * Last odometer sensor ticks value
    */
   double m_ticks;
   /**
    * Last odometer sensor validity flag
    */
   bool m_ticksValid;
   /**
    * Last odometer ticks per second speed [ticks/s]
    */
   double m_ticksPerSecond;
   /**
    * Last odometer linear velocity adjusted to reference timestamp [ticks/s]
    */
   double m_odoLinearVelocity;

   
private:
   /**************************************************************************************
    * Constant operation limits
    **************************************************************************************/

   const double m_speedInterval;
   const double m_speedHysteresis;
   const double m_speedMin;
   const double m_speedMax;
   const double m_speedAccuracyRatio;
   const double m_odoInterval;
   const double m_odoHysteresis;
   const double m_odoMin;
   const double m_odoMax;
};

} //namespace PE

#endif //__PE_COdometerEx_H__
