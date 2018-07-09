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
#include "PETools.h"



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
   static const std::string CFG_MARKER = "CFGSENSOR";

   static const std::size_t CFG_NUMBER_ELEMENTS = 11;

   static std::string ToSTR(const CSensorCfg& cfg)
      {
         std::stringstream st;
         st << CFG_MARKER << "," << int(mType) << ",";
         st << int(mScale.mAccValue) << "," << int(mScale.mAccMld) << "," << int(mScale.mAccRel) << "," << int(mScale.mCount) << ","; //store scale configuration
         st << int(mBase.mAccValue) << "," << int(mBase.mAccMld) << "," << int(mBase.mAccRel) << "," << int(mBase.mCount) << ","; //store base configuration
         st << std::setprecision(1) << double(mReliableLimit);
         return st.str();
      }

   static CSensorCfg ToCFG(const std::string& str)
      {
         std::vector<std::string> list = PE::TOOLS::Split();
         if ( CFG_NUMBER_ELEMENTS == list.size() )
         {
            if ( CFG_MARKER == list[0] )
            {
               return CSensorCfg(
                  atoi(list[1]), //load sensor type
                  CNormCfg(atoi(list[2]), atoi(list[3]), atoi(list[4]), atoi(list[5])), //load scale configuration
                  CNormCfg(atoi(list[6]), atoi(list[7]), atoi(list[8]), atoi(list[9])), //load base configuration
                  atof(list[10]) //load reliable limit
            }
         }
         return CSensorCfg();
      }

   CSensorCfg()
      : mType(SENSOR_UNKNOWN)
      , mScale(1,0,100,1) //Scale = 1
      , mBase (0,0,100,1) //Base  = 0
      , mReliableLimit(DEFAULT_RELIABLE_LIMIT)
      {}

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

   inline bool IsValid() const
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
      : mType(cfg.GetType())
      : mLimit(cfg.GetLimit())
      , mNormScale(CNormalisation(cfg.GetScale().mAccValue, cfg.GetScale().mAccMld, cfg.GetScale().mAccRel, cfg.GetScale().mCount))
      , mNormBase (CNormalisation(cfg.GetBase().mAccValue,  cfg.GetBase().mAccMld,  cfg.GetBase().mAccRel,  cfg.GetBase().mCount ))
      , mCalScale (mNormScale)
      , mCalBase  (mNormBase)
      {}

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

   SBasicSensor Calculate(const TValue& raw, const SBasicSensor& ref)
      {
         mCalScale.AddSensor(raw);
         mCalScale.AddReference(ref.Value);
         mCalScale.DoCalibration();

         if ( IsScaleReliable() )
         {
            mCalBase.AddScale(mCalScale.GetScale());
            mCalBase.AddSensor(raw);
            mCalBase.AddReference(ref.Value);
            mCalBase.DoCalibration();

            if ( IsBaseReliable() )
            {
               SBasicSensor sen(GetValue(raw), GetAccuracy(mNormBase.GetMld()));

               TValue deltaAccuracy = fabs(ref.Value - sen.Value);

               if ( deltaAccuracy > ref.Accuracy )
               {
                  sen.Accuracy = deltaAccuracy;
               }
               return sen;
            }
         }
         return SBasicSensor(); //invalid sensor
      }

private:
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

   inline TValue GetValue(const TValue& raw) const
      {
         return TValue(raw * mCalScale.GetScale() - mCalBase.GetBase());
      }

   inline TAccuracy GetAccuracy(const TValue& mld) const
      {
         return TAccuracy(mld * 3);
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

   /**
    * Constructor with start position and heading
    *
    * @param position       started position
    * @param heading        started heading
    */
   CCore( const SPosition& position, const SBasicSensor& heading);
   /**
    * Destructor 
    */
   virtual ~CCore();
   /**
    * Sets sensors configuration
    * Does not add invalid configuration and does not overwrite existed one.
    *
    * @param id      unique identificator of sensor
    * @param cfg     configuration information which will be binded to sensor identificator
    *
    * Returns false if configuration was invalid or already set
    */
   bool SetSensorCfg(TSensorID id, const CSensorCfg& cfg);
   /**
    * Gets sensor configuration
    * Does not change cfg parapeter if configuration is invalid or not present
    *
    * @param id      unique identificator of sensor
    * @param cfg     configuration information which will be updated
    *
    * Returns false if configuration was invalid or not present
    */
   bool GetSensorCfg(TSensorID id, CSensorCfg& cfg) const;
   /**
    * DEPRICATED
    * Adds new sensor raw value.
    *
    * @param id   unique identificator of sensor
    * @param ts   timestamp of the sensor
    * @param raw  raw data of the sensor
    * @param acc  accuracy of the sensor
    *
    */
   void AddSensor(TSensorID id, TTimestamp ts, const TValue& raw, const TAccuracy& acc);
   bool AddGNSS    (const TSensorID& id, const TTimestamp& ts, const SPosition& pos, const SBasicSensor& head, const SBasicSensor& speed);
   bool AddOdoAxis (const TSensorID& id, const TTimestamp& ts, const TValue& odo);
   bool AddGyroZ   (const TSensorID& id, const TTimestamp& ts, const TValue& gyro);
   bool AddPosition(const TSensorID& id, const TTimestamp& ts, const SPosition& pos);
   bool AddHead    (const TSensorID& id, const TTimestamp& ts, const SBasicSensor& head);
   bool AddSpeed   (const TSensorID& id, const TTimestamp& ts, const SBasicSensor& speed);
   /**
    * Calculates position based on current sensors data
    *
    * Returns true if new position has been calculated
    */
   bool CalculatePosition();
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
   /**
    * Type name definition for class method which processed specific sensor
    */
   typedef bool (PE::CCore::*TBuildCall)(TTimestamp ts, const TValue& raw, const TAccuracy& acc, CSensorEntity& ent);
   /**
    * Type definition for map which binds sensor type to specific handler
    */
   typedef std::map<TSensorTypeID, TBuildCall> TSensorHandlersList;
   /**
    * Type definition for map which binds sensor identificator to specific sensor environment
    */
   typedef std::map<TSensorID, std::pair< CSensorEntity, TBuildCall > > TSensorList;
   /**
    * List of all possible sensor handlers
    */
   TSensorHandlersList mHandlers;
   /**
    * List of all configurated sensors
    */
   TSensorList mSensors;
   /**
    * Returns true if configuration for sensors id is available and valid
    */
   inline bool IsSensorCongfigured(TSensorID id) const;
   /**
    * Returns true if sensors typed has proper handler
    */
   inline bool IsSensorTypeHandled(TSensorTypeID type) const;
   /**
    * Position fusion stuff
    */
   CFusionSensor mFusion;
   /**
    * Service variable to prepare income position for fusion
    */
   SPosition mPositionToFusion;
   /**
    *
    */
   void Latitude(TTimestamp ts, const TValue& raw, const TAccuracy& acc, CSensorEntity& ent);
   /**
    *
    */
   void Longitude(TTimestamp ts, const TValue& raw, const TAccuracy& acc, CSensorEntity& ent);
   /**
    *
    */
   void Heading(TTimestamp ts, const TValue& raw, const TAccuracy& acc, CSensorEntity& ent);
   /**
    *
    */
   void Speed(TTimestamp ts, const TValue& raw, const TAccuracy& acc, CSensorEntity& ent);
   /**
    *
    */
   void OdoAxis(TTimestamp ts, const TValue& raw, const TAccuracy& acc, CSensorEntity& ent);
   /**
    *
    */
   void GyroZ(TTimestamp ts, const TValue& raw, const TAccuracy& acc, CSensorEntity& ent);
};


} //namespace PE
#endif //__PE_CCore_H__
