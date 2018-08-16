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
, mOdoTs(0)
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
   return true;
}

//TODO!!!
//TODO: very first odo value will be skipped!!! maybe has to be improved
bool PE::CCoreSimple::AddOdo(TTimestamp ts, const SBasicSensor& odo)
{
   std::map<TSensorTypeID, CSensorEntity>::iterator it = mEntities.find(SENSOR_ODOMETER_AXIS);

   if ( mEntities.end() != it )
   {
      TTimestamp delta = 0.0;
      if ( 0.0 != mOdoTs )
      {
         delta = ts - mOdoTs;
      }
      mOdoTs = ts;
      if ( 0.0 < delta )
      {
         SBasicSensor rawSpeed(odo.Value / delta , odo.Accuracy);
         it->AddSensor(rawSpeed);
         if ( it->GetLimit() < it->CalibratedTo() )
         {
            mFusion.AddSpeed(ts, it->GetSensor(rawSpeed));
            if ( mInterval < delta )
            {
               mFusion.DoFusion();
               return true;
            }
         }
      }
   }
   return false;
}


bool PE::CCoreSimple::AddGyro(TTimestamp ts, const SBasicSensor& gyro)
{
   if ( mp_Gyro && 0 < mp_Gyro->CalibratedTo() )
   {
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
                CSensorCfg::ToNormCfg(it->GetScale()),
                CSensorCfg::ToNormCfg(it->GetBase()),
                it->GetLimit()
             );
   }
   else
   {
      return CSensorCfg();
   }
}



