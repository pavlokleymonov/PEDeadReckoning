/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2019 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */

#include "PECOdometer.h"


#define PE_ODO_RELIABLE_BIAS_LIMIT  90.0 /* 90% limit */
#define PE_ODO_RELIABLE_SCALE_LIMIT 90.0 /* 90% limit */
#define PE_ODO_SPEED_INTERVAL        1.0 /* Speed interval is 1000 milliseconds */
#define PE_ODO_INTERVAL            0.050 /* Odometer interval is 50 milliseconds */
#define PE_ODO_SPEED_ACCURACY_RATIO    5 /* Means that speed value has to be more then accuracy in 5 times. Deviation is less then 20% of the value */

using namespace PE;


PE::COdometer::COdometer()
: m_Speed_Ts(0)
, m_Odo_Ts(0)
{
}


void PE::COdometer::AddSpeed(const TTimestamp& ts, const TValue& speed, const TAccuracy& acc)
{
   if ( 0 == m_Speed_Ts )
   {
      if ( 0 < ts )
      {
         m_Speed_Ts = ts;
      }
   }
   else
   {
      TValue deltatime = ts - m_Speed_Ts;
      if ( true == IsSpeedOk(deltatime, speed, acc) )
      {
         m_Odo_Calib.AddRef( speed );
         m_Odo_Calib.Recalculate();
         if ( false == PE::isnan(m_Odo_Calib.GetBias()) )
         {
            m_Odo_Bias.AddSensor(m_Odo_Calib.GetBias());
         }
         if ( false == PE::isnan(m_Odo_Calib.GetScale()) )
         {
            m_Odo_Scale.AddSensor(m_Odo_Calib.GetScale());
         }
      }
      else
      {
         ResetUncomplitedProcessing();
      }
   }
}


void PE::COdometer::AddOdo(const TTimestamp& ts, const TValue& odo, bool IsValid )
{
   m_Last_Odo_Speed = SBasicSensor(); //Clean odo speed
   if ( 0 < m_Speed_Ts )
   {
      if ( 0 == m_Odo_Ts )
      {
         if ( 0 < ts )
         {
            m_Odo_Ts = ts;
         }
      }
      else
      {
         TValue deltatime = ts - m_Odo_Ts;
         if ( true == IsOdoOk(deltatime, odo, IsValid) )
         {
            TValue odoTickSpeed = odo / deltatime; //convert odo to speed for instance ticks per second
            m_Odo_Calib.AddRaw( odoTickSpeed );
            m_Last_Odo_Speed = CalculateOdoSpeed( odoTickSpeed );
            m_Odo_Ts = ts;
            return;
         }
         else
         {
            ResetUncomplitedProcessing();
         }
      }
   }
   else
   {
      m_Odo_Ts = 0;
   }
}


const TValue& PE::COdometer::BiasCalibartedTo() const
{
   return m_Odo_Bias.GetReliable();
}


const TValue& PE::COdometer::ScaleCalibartedTo() const
{
   return m_Odo_Scale.GetReliable();
}


const SBasicSensor& PE::COdometer::GetOdoSpeed() const
{
   return m_Last_Odo_Speed;
}


void PE::COdometer::ResetUncomplitedProcessing()
{
   m_Speed_Ts = 0;
   m_Odo_Ts = 0;
   m_Odo_Calib.CleanLastStep();
}


SBasicSensor PE::COdometer::CalculateOdoSpeed( const TValue& odoTickSpeed )
{
   SBasicSensor odoSpeed;//invalid by default
   if ( true == IsOdoCalibrated() )
   {
      odoSpeed.Value    = m_Odo_Scale.GetMean() * (odoTickSpeed - m_Odo_Bias.GetMean()); // value = scale * (odo - bias)
      odoSpeed.Accuracy = m_Odo_Bias.GetMld() * (m_Odo_Scale.GetMean() + m_Odo_Scale.GetMld()); // accuracy = bias_mld * (scale + scale_mld)
   }
   return odoSpeed;
}


bool PE::COdometer::IsSpeedOk( const TTimestamp& deltaTs, const TValue& speed, const TAccuracy& acc)
{
   if ( PE::EPSILON < speed ) //speed has to be always more then zero
   {
      if ( true == IsIntervalOk(deltaTs, PE_ODO_SPEED_INTERVAL, PE_ODO_SPEED_INTERVAL / 10) ) //hysteresis 10% - has to be adjusted
      {
         return IsAccuracyOk(speed, acc, PE_ODO_SPEED_ACCURACY_RATIO);
      }
   }
   return false;
}


bool PE::COdometer::IsOdoOk(const TTimestamp& deltaTs, const TValue& odo, bool IsValid )
{
   if ( true == IsValid )
   {
      return IsIntervalOk(deltaTs, PE_ODO_INTERVAL, PE_ODO_INTERVAL / 20); //hysteresis 5% - has to be adjusted
   }
   return false;
}


bool PE::COdometer::IsIntervalOk(const TTimestamp& deltaTs, const TValue& interval, const TValue& hysteresis)
{
   if ( PE::EPSILON < deltaTs )
   {
      if ( deltaTs < (interval + hysteresis) )
      {
         if ( deltaTs > (interval - hysteresis) )
         {
            return true;
         }
      }
   }
   return false;
}


bool PE::COdometer::IsAccuracyOk(const TValue& value, const TAccuracy& accuracy, const TValue& ratio)
{
   if ( value > (accuracy * ratio) )
   {
      return true;
   }
   return false;
}


bool PE::COdometer::IsOdoCalibrated()
{
   if ( PE_ODO_RELIABLE_BIAS_LIMIT < BiasCalibartedTo() ) // has to be adjustable
   {
      if ( PE_ODO_RELIABLE_SCALE_LIMIT < ScaleCalibartedTo() ) // has to be adjustable
      {
         return true;
      }
   }
   return false;
}
