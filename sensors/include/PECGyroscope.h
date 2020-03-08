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
   CGyroscope( const double& headInterval,
               const double& headHysteresis,
               const double& headMin,
               const double& headMax,
               const double& headAccuracyRatio,
               const double& gyroInterval,
               const double& gyroHysteresis,
               const double& gyroMin,
               const double& gyroMax);
   /**
    * Adds new reference heading
    * @return true if reference data was accepted
    *
    * @param  ts     Timestamp of heading in [s]
    * @param  head   Heading in [deg] with reference to true north, 0.0 -> north, 90.0 -> east, 180.0 south, 270.0 -> west
    * @param  acc    Heading accuracy in +/-[deg]
    */
   bool AddHeading(const double& ts, const double& head, const double& acc);
   /**
    * Adds new gyroscope sensor value
    * @return true if sensor data was accepted
    *
    * @param  ts        Timestamp of gyro sensor value [s]
    * @param  gyro      Gyroscope angular velicity in [units/s]
    * @param  isValid   True if sensors data is valid
    */
   bool AddGyro(const double& ts, const double& gyro, bool isValid );
   /**
    * Returns timestamp of last successfully added gyroscope sensor value.
    *         It is undefined if last AddGyro() call was unsuccessful
    * @return Sensor timestamp in seconds
    */
   const double& TimeStamp() const;
   /**
    * Returns converted gyroscope angular velocity according to reference information.
    *         It is undefined if last AddGyro() call was unsuccessful
    * @return calculated angular velocity based on gyroscope sensor data in [deg/s]
    */
   const double Value() const;
   /**
    * Returns accuracy of converted gyroscope sensor data.
    *         It is undefined if last AddGyro() call was unsuccessful
    * @return calculated accuracy of angular velocity based on gyroscope sensor data in +/-[deg/s]
    */
   const double Accuracy() const;
   /**
    * Returns gyroscope bias value
    * @return   bias of the gyroscope
    */
   const double& Base() const;
   /**
    * Returns gyroscope  scale value
    * @return   scale of the gyroscope sensor
    */
   const double& Scale() const;
   /**
    * Returns calibration completion status of base in %
    * @return  base calibration status in %
    */
   const double& CalibratedTo() const;

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
   virtual bool SetRefValue(const double& oldHeadTS, const double& newHeadTS, const double& head, const double& acc);
   /**
    * Adjust heading to angular velocity which was provided by previouse call SetRefValue()
    * Second call without upfront call of SetRefValue() has to return NaN
    *
    * @return adjusted angular velocity in [deg/s] or NaN in case of any errors
    */
   virtual const double& GetRefValue() const;
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
   virtual bool SetSenValue(const double& oldHeadTS, const double& oldGyroTS, const double& newGyroTS, const double& gyro, bool IsValid);
   /**
    * Just simple return of last gyroscope value.
    * Gyroscope value is already angular velocity in [unit/s]
    * Second call without upfront call of SetSenValue() has to return NaN
    *
    * @return gyroscope value in [unit/s] or NaN in case of any errors
    */
   virtual const double& GetSenValue() const;

private:
   /**
    * Service for processing sensors data
    */
   CSensor m_sensor;
   /**
    * Last reference heading value in [deg]
    */
   double m_headValue;
   /**
    * Last reference heading accuracy in +/-[deg]
    */
   double m_headAccuracy;
   /**
    * Last reference heading angular velocity [deg/s]
    */
   double m_headAngularVelocity;
   /**
    * Last gyroscope sensor value
    */
   double m_gyroValue;
   /**
    * Last gyroscope sensor validity flag
    */
   bool m_gyroValid;
   /**
    * Last gyroscope angular velocity adjusted to reference timestamp [unit/s]
    */
   double m_gyroAngularVelocity;
private:
   /**************************************************************************************
    * Constant operation limits
    **************************************************************************************/

   const double m_headInterval;
   const double m_headHysteresis;
   const double m_headMin;
   const double m_headMax;
   const double m_headAccuracyRatio;
   const double m_gyroInterval;
   const double m_gyroHysteresis;
   const double m_gyroMin;
   const double m_gyroMax;
};

} //namespace PE

#endif //__PE_CGyroscope_H__
