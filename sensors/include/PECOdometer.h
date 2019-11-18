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

class PECOdometerTest; //to get possibility for test class

namespace PE
{

/**
 * class for handling all odometer data
 *
 */
class COdometer
{
   friend class ::PECOdometerTest;

public:
   /**
    * Constructor of odemeter
    */
   COdometer();
   /**
    * Initialized odomer object
    * @return   true if init process was done successfully
    *
    * @param  odoInterval     odometer sensors interval in seconds
    * @param  speedInterval   reference speed interval in seconds
    * @param  biasLimit       base calibartion limit in %
    * @param  scaleLimit      scale calibartion limit in %
    * @param  speedAccuracyRatio   consider the reference speed only when value is more then accuracy by this ratio. For instance ration 5 means that value is 5 times bigger then accuracy.
    * @param  valueCalibrationCount   how much consecutive valid reference speed signals have to be received before calculation new bias and scale.
    */
   bool Init(const TValue& odoInterval, const TValue& speedInterval, const TValue& biasLimit, const TValue& scaleLimit, const uint32_t speedAccuracyRatio, const uint32_t valueCalibrationCount);
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
    * @param  ts        timestamp of odometer value [s]
    * @param  ticks     raw odometer value - measurement units does not matter
    * @param  IsValid   true if odometer sensors is valid
    */
   void AddOdo(const TTimestamp& ts, const TValue& ticks, bool IsValid );
   /**
    * Returns timestamp of last added odometer value
    * @return   odometer timestamp in seconds
    */
   const TTimestamp& GetOdoTimeStamp() const;
   /**
    * Returns odometer speed based on last added odometer value
    * @return   odometer speed in [m/s] with accuracy in +/-[m/s]
    */
   SBasicSensor GetOdoSpeed();
   /**
    * Returns odometer bias value
    * @return   bias of odometer
    */
   const TValue& GetOdoBias() const;
   /**
    * Returns calibration completion status of bias in %
    * @return   bias calibration status in %
    */
   const TValue& BiasCalibartedTo() const;
   /**
    * Returns odometer scale value
    * @return   scale of odometer
    */
   const TValue& GetOdoScale() const;
   /**
    * Returns calibration completion status of scale in %
    * @return   scale calibration status in %
    */
   const TValue& ScaleCalibartedTo() const;

private:
   /**
    * Shows if init process was done successfully
    */
   bool m_isInitOk;
   /**
    * Odometer sensors interval in seconds
    */
   TValue m_odoInterval;
   /**
    * Reference speed interval in seconds
    */
   TValue m_speedInterval;
   /**
    * Base calibartion limit in %
    */
   TValue m_biasLimit;
   /**
    * Scale calibartion limit in %
    */
   TValue m_scaleLimit;
   /**
    * Accuracy ration consider the reference speed only when value is more then accuracy by this ratio. For instance ration 5 means that value is 5 times bigger then accuracy.
    */
   uint32_t m_speedAccuracyRatio;
   /**
    * It shows how much consecutive valid reference speed signals have to be received before calculation new bias and scale.
    */
   uint32_t m_valueCalibrationCount;
   /**
     * Last speed timestamp
     */
    TTimestamp m_SpeedTs;
   /**
     * Last speed in [m/s]
     */
    TTimestamp m_Speed;
   /**
    * Last valid odometer timestamp
    */
   TTimestamp m_OdoTs;
   /**
    * Last odometer tick speed in ticks per second
    */
   TValue m_OdoTickSpeed;
   /**
    * Odometer calibration service
    */
   CCalibrationSummary m_OdoCalib;
   /**
    * Odometer normalisation service for bias
    */
   CNormalisation m_OdoBias;
   /**
    * Odometer normalisation service for scale
    */
   CNormalisation m_OdoScale;
   /**
    * Resets uncomplited calibration in case some inconsistency during current sensors processing
    */
   void ResetUncomplitedProcessing();
   /**
    * Calculates real odometer speed based on odometer ticks per second
    * @return   calculated odomete speed in [m/s] with accuracy +/-[m/s]
    *
    * @param odoTickSpeed   odemeter ticks speed ticks per second
    */
   SBasicSensor CalculateOdoSpeed( const TValue& odoTickSpeed );
   /**
    * Predict odometer ticks speed which is fit to the reference speed timestamp
    * @return   calculated odomete speed in ticks per second
    *
    * @param referenceSpeedTs     reference speed timestamp
    * @param odoTsBefore          odometer timestamp before reference speed timestamp
    * @param odoTsAfter           odometer timestamp after reference speed timestamp
    * @param odoTickSpeedBefore   odometer tick speed before reference speed timestamp
    * @param odoTickSpeedAfter    odometer tick speed after reference speed timestamp
    */
   TValue PredictOdoTickSpeed( const TTimestamp& referenceSpeedTs, const TTimestamp& odoTsBefore, const TTimestamp& odoTsAfter, const TValue& odoTickSpeedBefore, const TValue& odoTickSpeedAfter );
   /**
    * Inject new bias into normalization stuff
    *
    * @param bias   new bias value
    */
   void UpdateBias(const TValue& bias);
   /**
    * Inject new scale into normalization stuff
    *
    * @param scale   new bias value
    */
   void UpdateScale(const TValue& scale);
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
    * @param  ticks     odometer raw value
    * @param  IsValid   true if odometer value is valid
    */
   bool IsOdoOk(const TTimestamp& deltaTs, const TValue& ticks, bool IsValid );
   /**
    * Checks if delta time in a range of interval +/- hysteresis
    * IMPORTANT:   negative values have to be excluded!!!
    * @return   true if delta time passed all checkings
    *
    * @param  deltaTs      delta time for checking in second
    * @param  interval     interval of delat time in second
    * @param  hysteresis   hysteresis of delta time for given interval in second
    */
   bool IsIntervalOk(const TTimestamp& deltaTs, const TValue& interval, const TValue& hysteresis);
   /**
    * Checks if value is bigger than accuracy with specified ratio coefficient
    * IMPORTANT:   negative values have to be excluded!!!
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
    *
    * @param biasCalibartedTo    bias calibration status with range [0..100] in %
    * @param scaleCalibartedTo   scale calibration status with range [0..100] in %
    */
   bool IsOdoCalibrated(const TValue& biasCalibartedTo, const TValue& scaleCalibartedTo);
   /**
    * Checks if given tested timestamp is in specified range
    * @return   true if testedTS belongs to the range [beginTS .. endTS]
    *
    * @param testedTS   tested timestamp
    * @param beginTS    beginning of the specified range
    * @param endTS      end of the specified range
    */
   bool IsInRange(const TTimestamp& testedTS, const TTimestamp& beginTS, const TTimestamp& endTS);
   /**
    * Checks if calibration is possible based on given timestamps and speeds
    * @return   true if calibration is possible
    *
    * @param speedTs              reference speed timeestamp
    * @param speed                reference speed
    * @param OdoTsBefore          odometer timestamp before reference timestamp
    * @param OdoTickSpeedBefore   odometer ticks speed before reference timestamp
    * @param OdoTsAfter           odometer timestamp after reference timestamp
    * @param OdoTickSpeedAfter    odometer ticks speed after reference timestamp
    */
   bool IsCalibrationPossible( const TTimestamp& speedTs, const TValue& speed, const TTimestamp& OdoTsBefore, const TValue& OdoTickSpeedBefore, const TTimestamp& OdoTsAfter, const TValue& OdoTickSpeedAfter );
};

} //namespace PE

#endif //__PE_COdometer_H__
