/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2018 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */
#include "PECCoreSimple.h"

using namespace PE;


PE::CCoreSimple::CCoreSimple( const SPosition& position, const SBasicSensor& heading )
: mFusion(0, position, heading, SBasicSensor(), SBasicSensor())
{
}


bool PE::CCoreSimple::SetOdoCfg(const CSensorCfg& cfg)
{
   if ( !mOdo.GetCfg().IsValid() )
   {
      mOdo = CSensorEntity(cfg);
      return mOdo.GetCfg().IsValid();
   }
   return false;
}


const CSensorCfg& PE::CCoreSimple::GetOdoCfg() const
{
   return mOdo.GetCfg();
}


bool PE::CCoreSimple::SetGyroCfg(const CSensorCfg& cfg)
{
   if ( !mGyro.GetCfg().IsValid() )
   {
      mGyro = CSensorEntity(cfg);
      return mGyro.GetCfg().IsValid();
   }
   return false;
}


const CSensorCfg& PE::CCoreSimple::GetGyroCfg() const
{
   return mGyro.GetCfg();
}


bool PE::CCoreSimple::AddGnss(TTimestamp ts, const SPosition& pos, const SBasicSensor& head, const SBasicSensor& speed)
{
   mFusion.AddPosition(ts, pos);
   mFusion.AddHeading(ts, head);
   mFusion.AddSpeed(ts, speed);
   mFusion.DoFusion();
   return true;
}


bool PE::CCoreSimple::AddOdo(TTimestamp ts, const SBasicSensor& odo)
{
   if ( mOdo.GetCfg().IsValid() )
   {
      if ( odo.IsValid() )
      {
         if ( mOdo.AddRaw(odo.Value) )
         {
            mFusion.AddSpeed(ts, mOdo.GetSpeed())
         }
      }
      else
      {
         mOdo.Reset();
      }
   }
   return false;
}


bool PE::CCoreSimple::AddGyro(TTimestamp ts, const SBasicSensor& gyro)
{
   return false;
}


const TTimestamp& PE::CCoreSimple::GetTimestamp() const
{
   return TTimestamp(0);
}


const SPosition& PE::CCoreSimple::GetPosition() const
{
   return SPosition();
}


const SBasicSensor& PE::CCoreSimple::GetHeading() const
{
}


const SBasicSensor& PE::CCoreSimple::GetSpeed() const
{
}


