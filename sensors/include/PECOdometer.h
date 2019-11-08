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
#ifndef __PE_COdometer_H__
#define __PE_COdometer_H__

#include "PETypes.h"
#include "PESBasicSensor.h"
#include "PECNormalisation.h"
#include "PECCalibrationSummary.h"

namespace PE
{

/**
 * class for handling all odometer data
 *
 */
class COdometer
{
public:
   /**
    * Constructor of odemeter
    */
   COdometer();
   /**
    * Adds new reference speed
    *
    * @param  ts      timestamp of reference speed [s]
    * @param  speed   reference speed value in [m/s]
    * @param  acc     reference speed accuracy in +/-[m/s]
    */
   void AddSpeed(const TTimestamp& ts, const TValue& speed, const TAccuracy& acc);
   /**
    * Adds new raw value of odometer
    *
    * @param  ts    timestamp of odometer value [s]
    * @param  odo   raw odometer value - measurement units does not matter
    */
   void AddOdo(const TTimestamp& ts, const TValue& odo );
   /**
    * Returns calibration completion status of bias in %
    * @return   bias calibration status in %
    */
   const TValue& BiasCalibartedTo() const;
   /**
    * Returns calibration completion status of scale in %
    * @return   scale calibration status in %
    */
   const TValue& ScaleCalibartedTo() const;
   /**
    * Returns speed based on last added odometer value
    * @return   odometer speed in [m/s] with accuracy in +/-[m/s]
    */
   const SBasicSensor& GetOdoSpeed() const;

private:
   /**
    * Last speed timestamp
    */
   TTimestamp m_Speed_Ts;
   /**
    * Last valid odometer timestamp
    */
   TTimestamp m_Odo_Ts;
   /**
    * Last valid odometer speed in [m/s] with accuracy +/-[m/s]
    */
   SBasicSensor m_Last_Odo_Speed;
   /**
    * Odometer calibration service
    */
   CCalibrationSummary m_Odo_Calib;
   /**
    * Odometer normalisation service for bias
    */
   CNormalisation m_Odo_Bias;
   /**
    * Odometer normalisation service for scale
    */
   CNormalisation m_Odo_Scale;
   /**
    * Resets uncomplited calibration in case some inconsistency during current sensors processing
    */
   void ResetUncomplitedProcessing();
   /**
    * Calculates real speed based on odometer tick speed
    * @return   calculated odomete speed in [m/s] with accuracy +/-[m/s]
    *
    * @param odoTickSpeed   odemeter tick speed in ticks per second
    */
   SBasicSensor CalculateOdoSpeed( const TValue& odoTickSpeed );
   /**
    * Checks if given speed is fit to expected conditions and accuracy
    * @return   true if speed passed all checkings
    *
    * @param  deltaTs   delta time since last valid speed in second
    * @param  speed     speed value in [m/s] 
    * @param  acc       speed accuracy in +/-[m/s]
    */
   bool IsSpeedOk( const TTimestamp& deltaTs, const TValue& speed, const TAccuracy& acc);
   /**
    * Checks if odemeter is fit to expected conditions and accuracy
    * @return   true if odometer passed all checkings
    *
    * @param  deltaTs   delta time since last valid odometer in second
    * @param  odo       odometer raw value
    * @param  IsValid   true if odometer value is valid
    */
   bool IsOdoOk(const TTimestamp& deltaTs, const TValue& odo, bool IsValid );
   /**
    * Checks if delta time in a range of interval +/- hysteresis
    * @return   true if delta time passed all checkings
    *
    * @param  deltaTs      delta time for checking in second
    * @param  interval     interval of delat time in second
    * @param  hysteresis   hysteresis of delta time for given interval in second
    */
   bool IsIntervalOk(const TTimestamp& deltaTs, const TValue& interval, const TValue& hysteresis);
   /**
    * Checks if value is bigger than accuracy with specified ratio coefficient
    * @return   true if value passed all checkings
    *
    * @param  value      to be checked
    * @param  accuracy   accuracy of the value
    * @param  ratio      ratio coefficient
    */
   bool IsAccuracyOk(const TValue& value, const TAccuracy& accuracy, const TValue& ratio);
   /**
    * Checks if bias and scale of odometer riched specified calibration limit
    * @return   true if odometer reached calibration level
    */
   bool IsOdoCalibrated();
};

} //namespace PE

#endif //__PE_COdometer_H__
