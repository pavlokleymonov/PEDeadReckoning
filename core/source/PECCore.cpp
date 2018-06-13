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
{
}


PE::CCore::~CCore()
{
}


void PE::CCore::SetSensorCfg(PE::TSensorID id, const PE::CSensorCfg& cfg)
{
   if ( cfg.mValid )
   {
      CSensorEntity entity;
      entity.type      = cfg.mType;
      entity.normScale = PE::CNormalisation(cfg.mScale.mAccValue, cfg.mScale.mAccMld, cfg.mScale.mAccRel, cfg.mScale.mCount);
      entity.normBase  = PE::CNormalisation(cfg.mBase.mAccValue,  cfg.mBase.mAccMld,  cfg.mBase.mAccRel,  cfg.mBase.mCount);
      entity.calBase
      mSensors[id] = entity;
   }
}


const PE::CSensorCfg PE::CCore::GetSensorCfg(PE::TSensorID id) const
{
   //return mSensors[id];
}


bool PE::CCore::AddSensor(PE::TSensorID id, TTimestamp timestamp, const TValue& sensor, const TAccuracy& accuracy)
{
   if ( mSensors.end() != mSensors.find(id) )
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
      if ( mReadyToFusion )
      {
         mReadyToFusion = false;
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


bool PE::CCore::BuildPosition(TSensorID id, const TValue& sensor, const TAccuracy& accuracy)
{
   return false;
}


bool PE::CCore::BuildHeading(TSensorID id, const TValue& sensor, const TAccuracy& accuracy)
{
   return false;
}


bool PE::CCore::BuildSpeed(TSensorID id, const TValue& sensor, const TAccuracy& accuracy)
{
   return false;
}


bool PE::CCore::BuildAngSpeed(TSensorID id, const TValue& sensor, const TAccuracy& accuracy)
{
   return false;
}
