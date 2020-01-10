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
#ifndef __PE_CGyroscope_H__
#define __PE_CGyroscope_H__

#include "PETypes.h"
#include "PECSensor.h"

class PECGyroscopeTest; //to get possibility for test class

namespace PE
{

/**
 * class for processing gyroscope sensors data
 *
 */
class CGyroscope : public ISensorAdjuster
{

   friend class ::PECGyroscopeTest;

public:
   /**
    * Constructor
    */
   CGyroscope( const TValue& headInterval,
               const TValue& headHysteresis,
               const TValue& headMin,
               const TValue& headMax,
               const TValue& headAccuracyRatio,
               const TValue& gyroInterval,
               const TValue& gyroHysteresis,
               const TValue& gyroMin,
               const TValue& gyroMax);
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
   const TTimestamp& TimeStamp() const;
   /**
    * Returns converted gyroscope angular velocity according to reference information.
    *         It is undefined if last AddGyro() call was unsuccessful
    * @return calculated angular velocity based on gyroscope sensor data in [deg/s]
    */
   const TValue Value() const;
   /**
    * Returns accuracy of converted gyroscope sensor data.
    *         It is undefined if last AddGyro() call was unsuccessful
    * @return calculated accuracy of angular velocity based on gyroscope sensor data in +/-[deg/s]
    */
   const TAccuracy Accuracy() const;
   /**
    * Returns gyroscope bias value
    * @return   bias of the gyroscope
    */
   const TValue& Base() const;
   /**
    * Returns gyroscope  scale value
    * @return   scale of the gyroscope sensor
    */
   const TValue& Scale() const;
   /**
    * Returns calibration completion status of scale in %
    * @return   scale calibration status in %
    */
   const TValue& CalibartedTo() const;

public:
   /**************************************************************************************
    * ISensorAdjuster service methods
    **************************************************************************************/

   /**
    * Sets new heading value and checks if value, interval and accuracy are fit to expected conditions
    * @return   true if it passed all checkings
    *
    * @param  oldHeadTS   timestamp of the previouse heading in seconds [s]
    * @param  newHeadTS   timestamp of the new heading in seconds [s]
    * @param  head        new heading value in [deg]
    * @param  acc         accuracy of new heading value in +/-[deg]
    */
   virtual bool SetRefValue(const TTimestamp& oldHeadTS, const TTimestamp& newHeadTS, const TValue& head, const TAccuracy& acc);
   /**
    * Adjust heading to angular velocity which was provided by previouse call SetRefValue()
    * Second call without upfront call of SetRefValue() has to return NaN
    *
    * @return adjusted angular velocity in [deg/s] or NaN in case of any errors
    */
   virtual const TValue& GetRefValue() const;
   /**
    * Checks if given gyroscope angular velocity value, interval and validity are fit to expected conditions
    * @return   true if it passed all checkings
    *
    * @param  oldHeadTS   timestamp of the previouse heading in seconds [s]
    * @param  oldGyroTS   timestamp of the previouse gyroscope value in seconds [s]
    * @param  newGyroTS   timestamp of the new gyroscope value in seconds [s]
    * @param  gyro        gyroscope value in [unit/s]
    * @param  IsValid     true if gyro is valid
    */
   virtual bool SetSenValue(const TTimestamp& oldHeadTS, const TTimestamp& oldGyroTS, const TTimestamp& newGyroTS, const TValue& gyro, bool IsValid);
   /**
    * Just simple return of last gyroscope value.
    * Gyroscope value is already angular velocity in [unit/s]
    * Second call without upfront call of SetSenValue() has to return NaN
    *
    * @return gyroscope value in [unit/s] or NaN in case of any errors
    */
   virtual const TValue& GetSenValue() const;

private:
   /**
    * Service for processing sensors data
    */
   CSensor m_sensor;
   /**
    * Last reference heading value in [deg]
    */
   TValue m_headValue;
   /**
    * Last reference heading accuracy in +/-[deg]
    */
   TAccuracy m_headAccuracy;
   /**
    * Last reference heading angular velocity [deg/s]
    */
   TValue m_headAngularVelocity;
   /**
    * Last gyroscope sensor value
    */
   TValue m_gyroValue;
   /**
    * Last gyroscope sensor validity flag
    */
   bool m_gyroValid;
   /**
    * Last gyroscope angular velocity adgusted to reference timestamp [unit/s]
    */
   TValue m_gyroAngularVelocity;

   bool IsAllValueInRange(const TValue& v1, const TValue& v2, const TValue& min, const TValue& max) const;
private:
   /**************************************************************************************
    * Constant operation limits
    **************************************************************************************/

   const TValue m_headInterval;
   const TValue m_headHysteresis;
   const TValue m_headMin;
   const TValue m_headMax;
   const TValue m_headAccuracyRatio;
   const TValue m_gyroInterval;
   const TValue m_gyroHysteresis;
   const TValue m_gyroMin;
   const TValue m_gyroMax;
};

} //namespace PE

#endif //__PE_CGyroscope_H__
