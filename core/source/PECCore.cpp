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


PE::CCore::CCore( const SPosition& position, const SBasicSensor& heading )
: mFusion(0, position, heading, SBasicSensor(), SBasicSensor())
{
   mHandlers[SENSOR_LATITUDE]      = &PE::CCore::Latitude;
   mHandlers[SENSOR_LONGITUDE]     = &PE::CCore::Longitude;
   mHandlers[SENSOR_HEADING]       = &PE::CCore::Heading;
   mHandlers[SENSOR_SPEED]         = &PE::CCore::Speed;
   mHandlers[SENSOR_ODOMETER_AXIS] = &PE::CCore::OdoAxis;
   mHandlers[SENSOR_GYRO_Z]        = &PE::CCore::GyroZ;
}


PE::CCore::~CCore()
{
}


bool PE::CCore::SetSensorCfg(TSensorID id, const CSensorCfg& cfg)
{
   if ( true == cfg.IsValid() )
   {
      if ( true == IsSensorTypeHandled( cfg.GetType() ) )
      {
         if ( false == IsSensorCongfigured( id ) )
         {
            mSensors[id] = std::make_pair(CSensorEntity(cfg), mHandlers[cfg.GetType()]);
            return true;
         }
      }
   }
   return false;
}


bool PE::CCore::GetSensorCfg(TSensorID id, CSensorCfg& cfg) const
{
   if ( IsSensorCongfigured(id) )
   {
      cfg = mSensors[id].first.GetCfg();
      return true;
   }
   return false;
}


void PE::CCore::AddSensor(TSensorID id, TTimestamp timestamp, const TValue& sensor, const TAccuracy& accuracy)
{
   if ( IsSensorCongfigured(id) )
   {
      (this->*(mSensors[id].second))(timestamp, sensor, accuracy, mSensors[id].first);
   }
}


bool PE::CCore::CalculatePosition()
{
   TTimestamp prevTS = mFusion.GetTimestamp();
   mFusion.DoFusion();
   return (prevTS != mFusion.GetTimestamp());
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


inline bool PE::CCore::IsSensorCongfigured(TSensorID id) const
{
   return ( mSensors.end() != mSensors.find(id) );
}


inline bool PE::CCore::IsSensorTypeHandled(TSensorTypeID type) const
{
   return ( mHandlers.end() != mHandlers.find(type) );
}


void PE::CCore::Latitude(TTimestamp ts, const TValue& raw, const TAccuracy& acc, CSensorEntity& ent)
{
   if (SENSOR_LATITUDE == ent.GetType())
   {
      mPositionToFusion.Latitude = raw;
      mPositionToFusion.HorizontalAcc = acc;
      if ( mPositionToFusion.IsValid() )
      {
         mFusion.AddPosition(ts, mPositionToFusion);
         mPositionToFusion = SPosition(); //reset position
      }
   }
}


void PE::CCore::Longitude(TTimestamp ts, const TValue& raw, const TAccuracy& acc, CSensorEntity& ent)
{
   if (SENSOR_LONGITUDE == ent.GetType())
   {
      mPositionToFusion.Longitude = raw;
      mPositionToFusion.HorizontalAcc = acc;
      if ( mPositionToFusion.IsValid() )
      {
         mFusion.AddPosition(ts, mPositionToFusion);
         mPositionToFusion = SPosition(); //reset position
      }
   }
}


void PE::CCore::Heading(TTimestamp ts, const TValue& raw, const TAccuracy& acc, CSensorEntity& ent)
{
   if (SENSOR_HEADING == ent.GetType())
   {
      SBasicSensor head( raw, acc );
      if ( head.IsValid() )
      {
         mFusion.AddHeading( ts, head );
      }
   }
}


void PE::CCore::Speed(TTimestamp ts, const TValue& raw, const TAccuracy& acc, CSensorEntity& ent)
{
   if (SENSOR_SPEED == ent.GetType())
   {
      SBasicSensor speed( raw, acc );
      if ( speed.IsValid() )
      {
         mFusion.AddSpeed( ts, speed );
      }
   }
}


void PE::CCore::OdoAxis(TTimestamp ts, const TValue& raw, const TAccuracy& acc, CSensorEntity& ent)
{
   if (SENSOR_ODOMETER_AXIS == ent.GetType())
   {
      SBasicSensor predictedSpeed = mFusion.GetSpeed(ts);
      if ( predictedSpeed.IsValid() )
      {
         SBasicSensor speed = ent.Calculate(raw, predictedSpeed);
         if ( speed.IsValid() )
         {
            mFusion.AddSpeed( ts, speed );
         }
      }
   }
}


void PE::CCore::GyroZ(TTimestamp ts, const TValue& raw, const TAccuracy& acc, CSensorEntity& ent)
{
   if (SENSOR_GYRO_Z == ent.GetType())
   {
      SBasicSensor predictedAngSpeed = mFusion.GetAngSpeed(ts);
      if ( predictedAngSpeed.IsValid() )
      {
         SBasicSensor angSpeed = ent.Calculate(raw, predictedAngSpeed);
         if ( angSpeed.IsValid() )
         {
            mFusion.AddAngSpeed( ts, angSpeed );
         }
      }
   }
}

