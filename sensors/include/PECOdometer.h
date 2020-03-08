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
#include "PECNormalisation.h"
#include "PECCalibration.h"

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
    */
   bool Init(const double& odoInterval, const double& speedInterval, const double& biasLimit, const double& scaleLimit, const uint32_t speedAccuracyRatio);
   /**
    * Adds new reference speed
    *
    * @param  ts      timestamp of reference speed [s]
    * @param  speed   reference speed value in [m/s]
    * @param  acc     reference speed accuracy in +/-[m/s]
    */
   void AddSpeed(const double& ts, const double& speed, const double& acc);
   /**
    * Adds new raw value of odometer
    *
    * @param  ts        timestamp of odometer value [s]
    * @param  ticks     raw odometer value - measurement units does not matter
    * @param  IsValid   true if odometer sensors is valid
    */
   void AddOdo(const double& ts, const double& ticks, bool IsValid );
   /**
    * Returns timestamp of last added odometer value
    * @return   odometer timestamp in seconds
    */
   const double& GetOdoTimeStamp() const;
   /**
    * Checks if claibration of base and scale is enough for odometer speed calculation
    * @return   true if odometer speed could be calculated
    */
   bool IsOdoSpeedCalibrated() const;
   /**
    * Returns odometer speed value based on last added odometer value
    * @return   odometer speed in [m/s]
    */
   const double GetOdoSpeedValue() const;
   /**
    * Returns odometer speed accuracy based on last added odometer value
    * @return   odometer speed accuracy in +/-[m/s]
    */
   const double GetOdoSpeedAccuracy() const;
   /**
    * Returns odometer bias value
    * @return   bias of odometer
    */
   const double& GetOdoBias() const;
   /**
    * Returns calibration completion status of bias in %
    * @return   bias calibration status in %
    */
   const double& BiasCalibartedTo() const;
   /**
    * Returns odometer scale value
    * @return   scale of odometer
    */
   const double& GetOdoScale() const;
   /**
    * Returns calibration completion status of scale in %
    * @return   scale calibration status in %
    */
   const double& ScaleCalibartedTo() const;

private:
   /**
    * Shows if init process was done successfully
    */
   bool m_isInitOk;
   /**
    * Odometer sensors interval in seconds
    */
   double m_odoInterval;
   /**
    * Reference speed interval in seconds
    */
   double m_speedInterval;
   /**
    * Base calibartion limit in %
    */
   double m_biasLimit;
   /**
    * Scale calibartion limit in %
    */
   double m_scaleLimit;
   /**
    * Accuracy ration consider the reference speed only when value is more then accuracy by this ratio. For instance ration 5 means that value is 5 times bigger then accuracy.
    */
   uint32_t m_speedAccuracyRatio;
   /**
     * Last speed timestamp
     */
    double m_SpeedTs;
   /**
     * Last speed in [m/s]
     */
    double m_Speed;
   /**
    * Last valid odometer timestamp
    */
   double m_OdoTs;
   /**
    * Last odometer tick speed in ticks per second
    */
   double m_OdoTickSpeed;
   /**
    * Odometer calibration service
    */
   CCalibration m_OdoCalib;
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
    * Inject new bias into normalization stuff
    *
    * @param bias   new bias value
    */
   void UpdateBias(const double& bias);
   /**
    * Inject new scale into normalization stuff
    *
    * @param scale   new bias value
    */
   void UpdateScale(const double& scale);
   /**
    * Checks if given speed is fit to expected conditions and accuracy
    * @return   true if speed passed all checkings
    *
    * @param  deltaTs   delta time since last valid speed in second
    * @param  speed     speed value in [m/s] 
    * @param  acc       speed accuracy in +/-[m/s]
    */
   bool IsSpeedOk( const double& deltaTs, const double& speed, const double& acc) const;
   /**
    * Checks if odemeter is fit to expected conditions and accuracy
    * @return   true if odometer passed all checkings
    *
    * @param  deltaTs   delta time since last valid odometer in second
    * @param  ticks     odometer raw value
    * @param  IsValid   true if odometer value is valid
    */
   bool IsOdoOk(const double& deltaTs, const double& ticks, bool IsValid ) const;
   /**
    * Checks if bias and scale of odometer riched specified calibration limit
    * @return   true if odometer reached calibration level
    *
    * @param biasCalibartedTo    bias calibration status with range [0..100] in %
    * @param scaleCalibartedTo   scale calibration status with range [0..100] in %
    */
   bool IsOdoCalibrated(const double& biasCalibartedTo, const double& scaleCalibartedTo) const;
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
   bool IsCalibrationPossible( const double& speedTs, const double& speed, const double& OdoTsBefore, const double& OdoTickSpeedBefore, const double& OdoTsAfter, const double& OdoTickSpeedAfter ) const;
};

} //namespace PE

#endif //__PE_COdometer_H__
