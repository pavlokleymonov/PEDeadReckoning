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

struct CNormCfg
{
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


struct CSensorCfg
{
   CSensorCfg()
      : mType(SENSOR_UNKNOWN)
      , mBase()
      , mScale()
      , mValid(false)
      {}

   CSensorCfg(TSensorTypeID type, const CNormCfg& base, const CNormCfg& scale)
      : mType(type)
      , mBase(base)
      , mScale(scale)
      , mValid(true)
      {}

   TSensorTypeID mType;
   CNormCfg mBase;
   CNormCfg mScale;
   const bool mValid;
};


struct CSensorEntity
{
   TSensorTypeID type;
   CNormalisation normScale;
   CNormalisation normBase;
   CCalibrationScale calScale;
   CCalibrationBase  calBase;
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
   /**
    * Constructor
    */
   CCore();
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
    * Returns false if configuration of the sensor with provided id is not available or there is no new position
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

   std::map<TSensorID,CSensorEntity> mSensors;

   CFusionSensor mFusion;

   bool mReadyToFusion;

   SPosition mPositionToFusion;

   SBasicSensor mHeadingToFusion;

   SBasicSensor mSpeedToFusion;

   SBasicSensor mAngSpeedToFusion;

   bool BuildPosition(TSensorID id, const TValue& sensor, const TAccuracy& accuracy);

   bool BuildHeading(TSensorID id, const TValue& sensor, const TAccuracy& accuracy);

   bool BuildSpeed(TSensorID id, const TValue& sensor, const TAccuracy& accuracy);

   bool BuildAngSpeed(TSensorID id, const TValue& sensor, const TAccuracy& accuracy);
};


} //namespace PE
#endif //__PE_CCore_H__
