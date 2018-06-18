/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2017 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */
#ifndef __PE_CCore_H__
#define __PE_CCore_H__

#include "PETypes.h"
#include "PESPosition.h"
#include "PECNormalisation.h"
#include "PECCalibrationScale.h"
#include "PECCalibrationBase.h"
#include "PECFusionSensor.h"



class PECCoreTest; //to get possibility for test class

namespace PE
{

class CNormCfg
{
public:
   CNormCfg()
      : mAccValue(0.0)
      , mAccMld(0.0)
      , mAccRel(0.0)
      , mCount(0)
      {}

   CNormCfg( const TValue& accumulatedValue, const TValue& accumulatedMld, const TValue& accumulatedReliable, const std::size_t& sampleCount )
      : mAccValue(accumulatedValue)
      , mAccMld(accumulatedMld)
      , mAccRel(accumulatedReliable)
      , mCount(sampleCount)
      {}

   TValue mAccValue;
   TValue mAccMld;
   TValue mAccRel;
   std::size_t mCount;
};


class CSensorCfg
{
public:

   CSensorCfg( TSensorTypeID type, const CNormCfg& scale, const CNormCfg& base, TValue reliableLimit = DEFAULT_RELIABLE_LIMIT )
      : mType(type)
      , mScale(scale)
      , mBase(base)
      , mReliableLimit(reliableLimit)
      {}

   const TSensorTypeID& GetType() const
      {
         return mType;
      }

   bool IsValid() const
      {
         return ( SENSOR_UNKNOWN != mType );
      }

   const CNormCfg& GetScale() const
      {
         return mScale;
      }

   const CNormCfg& GetBase() const
      {
         return mBase;
      }

   const TValue& GetLimit() const
      {
         return mReliableLimit;
      }

private:
   TSensorTypeID mType;
   CNormCfg mScale;
   CNormCfg mBase;
   TValue mReliableLimit;
};


class CSensorEntity
{
public:
   CSensorEntity( const CSensorCfg& cfg )
      : referenceIsReady(false)
      , sensorIsReady(false)
      , mType(cfg.GetType())
      , mLimit(cfg.GetLimit())
      , mNormScale(CNormalisation(cfg.GetScale().mAccValue, cfg.GetScale().mAccMld, cfg.GetScale().mAccRel, cfg.GetScale().mCount))
      , mNormBase (CNormalisation(cfg.GetBase().mAccValue,  cfg.GetBase().mAccMld,  cfg.GetBase().mAccRel,  cfg.GetBase().mCount ))
      , mCalScale (mNormScale)
      , mCalBase  (mNormBase)
      {}

   //maybe accumulated data has to be reduced
   CSensorCfg GetCfg() const
      {
         return CSensorCfg(
            mType,
            CNormCfg(mNormScale.GetAccumulatedValue(), mNormScale.GetAccumulatedMld(), mNormScale.GetAccumulatedReliable(), mNormScale.GetSampleCount()),
            CNormCfg(mNormBase.GetAccumulatedValue(), mNormBase.GetAccumulatedMld(), mNormBase.GetAccumulatedReliable(), mNormBase.GetSampleCount()),
            mLimit
         );
      }

   const TSensorTypeID& GetType() const
      {
         return mType;
      }

   bool IsReliable() const
      {
         return ( IsScaleReliable() && IsBaseReliable() );
      }

   TValue CalibartedTo() const
      {
         return (mNormScale.GetReliable() + mNormBase.GetReliable()) / 2;
      }

   const TValue& GetScale() const
      {
         return mCalScale.GetScale();
      }

   const TValue& GetBase() const
      {
         return mCalBase.GetBase();
      }

   TValue GetValue(const TValue& raw) const
      {
         return (raw - mCalBase.GetBase()) * mCalScale.GetScale();
      }

   void AddReference(const SBasicSensor& reference)
      {
         if ( reference.IsValid() )
         {
            referenceIsReady = true;
            mCalScale.AddReference(value);
            mCalBase.AddReference(value);
            DoCalibration();
         }
      }

   void AddSensor(const TValue& raw)
      {
         sensorIsReady = true;
         mCalScale.AddSensor(raw);
         mCalBase.AddSensor(raw);
         DoCalibration();
      }

   void AddPredictedValue()

   /**
    * Resets internal state to be able to process new data without dependencies for previouse one.
    * It would be usefull after gap detection in any income data.
    */
   void Reset()
      {
         mCalScale.Reset();
         mCalBase.Reset();
      }

private:
   bool referenceIsReady;
   bool sensorIsReady;
   const TSensorTypeID mType;
   const TValue mLimit;
   CNormalisation mNormScale;
   CNormalisation mNormBase;
   CCalibrationScale mCalScale;
   CCalibrationBase  mCalBase;

   inline bool IsScaleReliable() const
      {
         return mLimit <= mNormScale.GetReliable();
      }

   inline bool IsBaseReliable() const
      {
         return mLimit <= mNormBase.GetReliable();
      }

   inline void DoCalibration()
      {
         if ( sensorIsReady && referenceIsReady )
         {
            mCalScale.DoCalibration();
            if ( IsScaleReliable() )
            {
               mCalBase.AddScale( mCalScale.GetScale() );
               mCalBase.DoCalibration();
            }
            else
            {
               mCalBase.Reset();
            }
            sensorIsReady = false;
            referenceIsReady = false;
         }
      }

};
/**
 * 
 * Preconditions:
 *  - 
 *
 */
class CCore
{

friend class ::PECCoreTest;

public:

   typedef bool (PE::CCore::*TBuildCall)(PE::TTimestamp timestamp, const PE::TValue& sensor, const PE::TAccuracy& accuracy);

   typedef std::map<PE::TSensorID, std::pair< PE::CSensorEntity, PE::TBuildCall > > TSensorHandlerList;
   
   /**
    * Constructor
    */
   CCore();
   /**
    * Constructor with start position and heading
    *
    * @param position       started position
    * @param heading        started heading
    * @param reliableLimit  calibration limit which will be only used for sensors position fusion
    */
   CCore( const SPosition& position, const SBasicSensor& heading, const TValue& reliableLimit);
   /**
    * Destructor 
    */
   virtual ~CCore();
   /**
    * Writes sensors configuration
    * Does not add invalid configuration and does not overwrite existed one.
    *
    * @param id      unique identificator of sensor
    * @param cfg     configuration information which will be binded to sensor identificator
    *
    * Returns false if configuration was not written
    */
   bool WriteSensorCfg(TSensorID id, const CSensorCfg& cfg);
   /**
    * Reads sensor configuration
    * if configuration is not present - returns invalid sensor config
    */
   bool ReadSensorCfg(TSensorID id, CSensorCfg& cfg) const;
   /**
    * Adds new sensor raw value.
    *
    * Returns false if configuration of the sensor with provided id is not available or there is no new position available
    */
   bool AddSensor(TSensorID id, TTimestamp timestamp, const TValue& sensor, const TAccuracy& accuracy);
   /**
    * Returns timestamp of fusion position
    */
   const TTimestamp& GetTimestamp() const;
   /**
    * Returns Position of fusion position
    */
   const SPosition& GetPosition() const;
   /**
    * Returns Heading of fusion position
    */
   const SBasicSensor& GetHeading() const;
   /**
    * Returns Speed of fusion position
    */
   const SBasicSensor& GetSpeed() const;

private:

   TSensorHandlerList mSensors;

   const TValue mReliableLimit;

   CFusionSensor mFusion;

   bool mPositionReadyToFusion;

   bool mDistanceReadyToFusion;

   bool mHeadingReadyToFusion;

   SBasicSensor mLatitudeToFusion;

   SBasicSensor mLongitudeToFusion;

   SPosition mPositionToFusion;

   SBasicSensor mHeadingToFusion;

   SBasicSensor mSpeedToFusion;

   SBasicSensor mAngSpeedToFusion;
   /**
    * Returns true if new sensor information is enought to call fusie new position.
    *    For instance new speed and heading were received or coordinates were received
    */
   inline bool IsReadyToFusion() const;
   /**
    * Cleans distance and heading readiness flags
    */
   inline void ClearReadyToFusion();
   /**
    * Returns true if configuration for sensors id is available and valid
    */
   inline bool IsSensorCongfigured(TSensorID id) const;




   inline const TValue GetScale(TSensorID id) const;

   inline const TValue GetScaleReliable(TSensorID id) const;

   inline const TValue GetBase(TSensorID id) const;

   inline const TValue GetScaleReliable(TSensorID id) const;






   bool BuildPosition(TSensorID id, const TValue& sensor, const TAccuracy& accuracy);

   bool BuildHeading(TSensorID id, const TValue& sensor, const TAccuracy& accuracy);

   bool BuildSpeed(TSensorID id, const TValue& sensor, const TAccuracy& accuracy);

   bool BuildAngSpeed(TSensorID id, const TValue& sensor, const TAccuracy& accuracy);
};


} //namespace PE
#endif //__PE_CCore_H__
