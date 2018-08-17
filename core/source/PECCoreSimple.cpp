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


PE::CCoreSimple::CCoreSimple( const SPosition& position, const SBasicSensor& heading, TTimestamp interval)
: mInterval(interval)
, mFusion(0, position, heading, SBasicSensor(), SBasicSensor())
{
}


void PE::CCoreSimple::SetOdoCfg(const CSensorCfg& cfg, TValue ratio, TValue threshold)
{
   if ( SENSOR_ODOMETER_AXIS == cfg.GetType() )
   {
      SetCfg( cfg, ratio, threshold);
   }
}


CSensorCfg PE::CCoreSimple::GetOdoCfg() const
{
   return GetCfg(SENSOR_ODOMETER_AXIS);
}


void PE::CCoreSimple::SetGyroCfg(const CSensorCfg& cfg, TValue ratio, TValue threshold)
{
   if ( SENSOR_GYRO_Z == cfg.GetType() )
   {
      SetCfg( cfg, ratio, threshold);
   }
}


CSensorCfg PE::CCoreSimple::GetGyroCfg() const
{
   return GetCfg(SENSOR_GYRO_Z);
}


bool PE::CCoreSimple::AddGnss(TTimestamp ts, const SPosition& pos, const SBasicSensor& head, const SBasicSensor& speed)
{
   mFusion.AddPosition(ts, pos);
   mFusion.AddHeading(ts, head);
   mFusion.AddSpeed(ts, speed);
   mFusion.DoFusion();

   AddRef(SENSOR_ODOMETER_AXIS, mFusion.GetSpeed());
   AddRef(SENSOR_GYRO_Z, mFusion.GetAngSpeed());

   return true;
}


bool PE::CCoreSimple::AddOdo(TTimestamp ts, const SBasicSensor& odo)
{
   SBasicSensor speed = CalculateSensor(SENSOR_ODOMETER_AXIS, odo);

   if ( speed.IsValid() )
   {
      mFusion.AddSpeed(ts, speed);
      return UpdatePosition(ts);
   }

   return false;
}


bool PE::CCoreSimple::AddGyro(TTimestamp ts, const SBasicSensor& gyro)
{
   SBasicSensor angSpeed = CalculateSensor(SENSOR_GYRO_Z, gyro);

   if ( angSpeed.IsValid() )
   {
      mFusion.AddAngSpeed(ts, angSpeed);
      return UpdatePosition(ts);
   }

   return false;
}


const TTimestamp& PE::CCoreSimple::GetTimestamp() const
{
   return mFusion.GetTimestamp();
}


const SPosition& PE::CCoreSimple::GetPosition() const
{
   return mFusion.GetPosition();
}


const SBasicSensor& PE::CCoreSimple::GetHeading() const
{
   return mFusion.GetHeading();
}


const SBasicSensor& PE::CCoreSimple::GetSpeed() const
{
   return mFusion.GetSpeed();
}


void PE::CCoreSimple::SetCfg(const CSensorCfg& cfg, TValue ratio, TValue threshold)
{
   std::map<TSensorTypeID, CSensorEntity>::const_iterator it = mEntities.find(cfg.GetType());
   if ( mEntities.end() != it )
   {
      mEntities.erase(it);
   }
   mEntities.insert(std::make_pair(cfg.GetType(), CSensorEntity(cfg,ratio,threshold)));
}


CSensorCfg PE::CCoreSimple::GetCfg(TSensorTypeID typeId) const
{
   std::map<TSensorTypeID, CSensorEntity>::const_iterator it = mEntities.find(typeId);
   if ( mEntities.end() != it )
   {
      return CSensorCfg(
                typeId,
                CSensorCfg::ToNormCfg(it->second.GetScale()),
                CSensorCfg::ToNormCfg(it->second.GetBase()),
                it->second.GetLimit()
             );
   }
   return CSensorCfg();
}


void PE::CCoreSimple::AddRef(TSensorTypeID typeId, const SBasicSensor& ref)
{
   std::map<TSensorTypeID, CSensorEntity>::iterator it = mEntities.find(typeId);
   if ( mEntities.end() != it )
   {
      it->second.AddReference(ref);
   }
}


SBasicSensor PE::CCoreSimple::CalculateSensor(TSensorTypeID typeId, const SBasicSensor& raw)
{
   std::map<TSensorTypeID, CSensorEntity>::iterator it = mEntities.find(typeId);
   if ( mEntities.end() != it )
   {
      it->second.AddSensor(raw);
      if ( it->second.GetLimit() <= it->second.CalibratedTo() )
      {
         return it->second.GetSensor(raw);
      }
   }
   return SBasicSensor();
}


bool PE::CCoreSimple::UpdatePosition(TTimestamp ts)
{
   if ( GetTimestamp() < ts )
   {
      TTimestamp delta = ts - GetTimestamp();
      if ( mInterval <= delta )
      {
         mFusion.DoFusion();
         return true;
      }
   }
   return false;
}
