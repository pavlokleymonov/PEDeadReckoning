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


PE::CCoreSimple::CCoreSimple( const SPosition& position, const SBasicSensor& heading)
: mFusion(0, position, heading, SBasicSensor(), SBasicSensor())
{
}


PE::CCoreSimple::~CCoreSimple()
{
   while (!mEntities.empty())
   {
      delete mEntities.begin()->second;
      mEntities.erase(mEntities.begin());
   }
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


TValue PE::CCoreSimple::OdoCalibratedTo() const
{
   return CalibratedTo(SENSOR_ODOMETER_AXIS);
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


TValue PE::CCoreSimple::GyroCalibratedTo() const
{
   return CalibratedTo(SENSOR_GYRO_Z);
}


void PE::CCoreSimple::AddGnss(TTimestamp ts, const SPosition& pos, const SBasicSensor& head, const SBasicSensor& speed)
{
   mFusion.AddPosition(ts, pos);
   mFusion.AddHeading(ts, head);
   mFusion.AddSpeed(ts, speed);
   mFusion.DoFusion();

   AddRef(SENSOR_ODOMETER_AXIS, mFusion.GetSpeed());
   AddRef(SENSOR_GYRO_Z, mFusion.GetAngSpeed());
}


void PE::CCoreSimple::AddOdo(TTimestamp ts, const SBasicSensor& odo)
{
   mFusion.AddSpeed(ts, CalculateSensor(SENSOR_ODOMETER_AXIS, odo));
}


void PE::CCoreSimple::AddGyro(TTimestamp ts, const SBasicSensor& gyro)
{
   mFusion.AddAngSpeed(ts, CalculateSensor(SENSOR_GYRO_Z, gyro));
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


void PE::CCoreSimple::UpdatePosition()
{
   mFusion.DoFusion();
}


void PE::CCoreSimple::SetCfg(const CSensorCfg& cfg, TValue ratio, TValue threshold)
{
   std::map<TSensorTypeID, CSensorEntity*>::const_iterator it = mEntities.find(cfg.GetType());
   if ( mEntities.end() != it )
   {
      delete it->second;
      mEntities.erase(it);
   }
   mEntities.insert(std::make_pair(cfg.GetType(), new CSensorEntity(cfg,ratio,threshold)));
}


CSensorCfg PE::CCoreSimple::GetCfg(TSensorTypeID typeId) const
{
   std::map<TSensorTypeID, CSensorEntity*>::const_iterator it = mEntities.find(typeId);
   if ( mEntities.end() != it )
   {
      return CSensorCfg(
                typeId,
                PE::CSensorCfg::ToNormCfg(it->second->GetScale()),
                PE::CSensorCfg::ToNormCfg(it->second->GetBase()),
                it->second->GetLimit()
             );
   }
   return CSensorCfg();
}


TValue PE::CCoreSimple::CalibratedTo(TSensorTypeID typeId) const
{
   std::map<TSensorTypeID, CSensorEntity*>::const_iterator it = mEntities.find(typeId);
   if ( mEntities.end() != it )
   {
      return it->second->GetBase().GetReliable();
   }
   return 0.0;
}

void PE::CCoreSimple::AddRef(TSensorTypeID typeId, const SBasicSensor& ref)
{
   std::map<TSensorTypeID, CSensorEntity*>::iterator it = mEntities.find(typeId);
   if ( mEntities.end() != it )
   {
      it->second->AddReference(ref);
   }
}


SBasicSensor PE::CCoreSimple::CalculateSensor(TSensorTypeID typeId, const SBasicSensor& raw)
{
   std::map<TSensorTypeID, CSensorEntity*>::iterator it = mEntities.find(typeId);
   if ( mEntities.end() != it )
   {
      it->second->AddSensor(raw);
      if ( it->second->GetLimit() <= it->second->GetBase().GetReliable() )
      {
         SBasicSensor sen = it->second->GetSensor(raw);
         return sen;
      }
   }
   return SBasicSensor();
}
