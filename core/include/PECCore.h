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
   CSensorCfg()
      : mType(SENSOR_UNKNOWN)
      , mScale()
      , mBase()
      , mValid(false)
      {}

   CSensorCfg(TSensorTypeID type, const CNormCfg& scale, const CNormCfg& base)
      : mType(type)
      , mScale(scale)
      , mBase(base)
      , mValid(true)
      {}

   TSensorTypeID mType;
   CNormCfg mScale;
   CNormCfg mBase;
   const bool mValid;
};


class CSensorEntity
{
public:
   CSensorEntity(TSensorTypeID type, const PE::CSensorCfg& cfg)
      : mType(type)
      , mNormScale(CNormalisation(cfg.mScale.mAccValue, cfg.mScale.mAccMld, cfg.mScale.mAccRel, cfg.mScale.mCount))
      , mNormBase (CNormalisation(cfg.mBase.mAccValue,  cfg.mBase.mAccMld,  cfg.mBase.mAccRel,  cfg.mBase.mCount ))
      , mCalScale (mNormScale)
      , mCalBase  (mNormBase)
      {}

   TSensorTypeID mType;
   CNormalisation mNormScale;
   CNormalisation mNormBase;
   CCalibrationScale mCalScale;
   CCalibrationBase  mCalBase;
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
    */
   CCore( const SPosition& position, const SBasicSensor& heading, const TValue& reliableLimit);
   /**
    * Destructor 
    */
   virtual ~CCore();
   /**
    * Adds sensors configuration
    * does not add invalid configuration
    */
   void SetSensorCfg(TSensorID id, const CSensorCfg& cfg);
   /**
    * Returns sensor configuration
    * if configuration is not present - returns invalid sensor config
    */
   const CSensorCfg GetSensorCfg(TSensorID id) const;
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
