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
#include "PECCore.h"


PE::CCore::CCore()
: mReliableLimit(99.5) //99.5% - Default limit
, mFusion(0, SPosition(), SBasicSensor(), SBasicSensor(), SBasicSensor())
, mDistanceReadyToFusion(false)
, mHeadingReadyToFusion(false)
{
}


PE::CCore::CCore( const PE::SPosition& position, const PE::SBasicSensor& heading, const PE::TValue& reliableLimit )
: mReliableLimit(reliableLimit)
, mFusion(0, position, heading, SBasicSensor(), SBasicSensor())
, mDistanceReadyToFusion(false)
, mHeadingReadyToFusion(false)
{
}


PE::CCore::~CCore()
{
}


void PE::CCore::SetSensorCfg(PE::TSensorID id, const PE::CSensorCfg& cfg)
{
   if ( cfg.mValid )
   {
      mSensors[id] = CSensorEntity(id, cfg);
   }
}


const PE::CSensorCfg PE::CCore::GetSensorCfg(PE::TSensorID id) const
{
   CSensorCfg result;
   if ( IsSensorCongfigured(id) )
   {
      result = CSensorCfg(
            mSensors[id].mType,
            CNormCfg(mSensors[id].mNormScale.GetAccumulatedValue(), mSensors[id].mNormScale.GetAccumulatedMld(), mSensors[id].mNormScale.GetAccumulatedReliable(), mSensors[id].mNormScale.GetSampleCount()),
            CNormCfg(mSensors[id].mNormBase.GetAccumulatedValue(),  mSensors[id].mNormBase.GetAccumulatedMld(),  mSensors[id].mNormBase.GetAccumulatedReliable(),  mSensors[id].mNormBase.GetSampleCount() )
         )
   }
   return result;
}


bool PE::CCore::AddSensor(PE::TSensorID id, PE::TTimestamp timestamp, const PE::TValue& sensor, const PE::TAccuracy& accuracy)
{
   if ( IsSensorCongfigured(id) )
   {
      if      ( BuildPosition(id,sensor,accuracy) )
      {
         mFusion.AddPosition( timestamp, mPositionToFusion );
      }
      else if ( BuildHeading(id,sensor,accuracy) )
      {
         mFusion.AddHeading( timestamp, mHeadingToFusion );
      }
      else if ( BuildSpeed(id,sensor,accuracy) )
      {
         mFusion.AddSpeed( timestamp, mSpeedToFusion );
      }
      else if ( BuildAngSpeed(id,sensor,accuracy) )
      {
         mFusion.AddAngSpeed( timestamp, mAngSpeedToFusion );
      }
      else
      {
         return false;
      }
      if ( IsReadyToFusion() )
      {
         ClearReadyToFusion();
         mFusion.DoFusion();
         return true;
      }
      else
      {
         return false;
      }
   }
   else
   {
      return false;
   }
}


const PE::TTimestamp& PE::CCore::GetTimestamp() const
{
   return mFusion.GetTimestamp();
}


const PE::SPosition& PE::CCore::GetPosition() const
{
   return mFusion.GetPosition();
}


const PE::SBasicSensor& PE::CCore::GetHeading() const
{
   return mFusion.GetHeading();
}


const PE::SBasicSensor& PE::CCore::GetSpeed() const
{
   return mFusion.GetSpeed();
}


inline bool PE::CCore::IsReadyToFusion() const
{
   return ( mDistanceReadyToFusion && mHeadingReadyToFusion );
}


inline void PE::CCore::ClearReadyToFusion()
{
   mDistanceReadyToFusion = false;
   mHeadingReadyToFusion = false;
}


inline bool PE::CCore::IsSensorCongfigured(PE::TSensorID id) const
{
   bool isPresent = mSensors.end() != mSensors.find(id);
   return ( isPresent ? mSensors[id].mValid : false );
}


inline const PE::TValue PE::CCore::GetScale(PE::TSensorID id) const
{
   return mSensors[id].mCalScale.GetScale();
}


inline const PE::TValue PE::CCore::GetScaleReliable(PE::TSensorID id) const
{
   return mSensors[id].mCalScale.GetReliable();
}


inline const PE::TValue PE::CCore::GetBase(PE::TSensorID id) const
{
   return mSensors[id].mCalBase.GetScale();
}


inline const PE::TValue PE::CCore::GetScaleReliable(PE::TSensorID id) const
{
   return mSensors[id].mCalScale.GetReliable();
}


bool PE::CCore::BuildPosition(PE::TSensorID id, const PE::TValue& sensor, const PE::TAccuracy& accuracy)
{
   
   return false;
}


bool PE::CCore::BuildHeading(PE::TSensorID id, const PE::TValue& sensor, const PE::TAccuracy& accuracy)
{
   return false;
}


bool PE::CCore::BuildSpeed(PE::TSensorID id, const PE::TValue& sensor, const PE::TAccuracy& accuracy)
{
   return false;
}


bool PE::CCore::BuildAngSpeed(PE::TSensorID id, const PE::TValue& sensor, const PE::TAccuracy& accuracy)
{
   return false;
}
