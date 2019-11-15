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


using namespace PE;


PE::COdometer::COdometer()
: m_isInitOk(false)
, m_odoInterval(0)
, m_speedInterval(0)
, m_biasLimit(0)
, m_scaleLimit(0)
, m_speedAccuracyRatio(0)
, m_valueCalibrationCount(0)
, m_SpeedTs(0)
, m_Speed(0)
, m_OdoTs(0)
, m_OdoTickSpeed(0)
{
}


bool PE::COdometer::Init(const TValue& odoInterval, const TValue& speedInterval, const TValue& biasLimit, const TValue& scaleLimit, const uint32_t speedAccuracyRatio, const uint32_t valueCalibrationCount)
{
   if ( PE::EPSILON < odoInterval ) //all intervals have to be positive and bigger then zero
   {
      if ( odoInterval < speedInterval ) //speed interval has to be bigger then odometer interval
      {
         if ( PE::EPSILON < biasLimit ) //bias limit has to be bigger then zero
         {
            if ( PE::EPSILON < scaleLimit ) //scale limit has to be bigger then zero
            {
               if ( 0 < speedAccuracyRatio ) //ration has to be bigger then 0
               {
                  if ( 0 < valueCalibrationCount ) //calibration count has to be bigger then 0
                  {
                     m_isInitOk = true;
                     m_odoInterval = odoInterval;
                     m_speedInterval = speedInterval;
                     m_biasLimit = biasLimit;
                     m_scaleLimit = scaleLimit;
                     m_speedAccuracyRatio = speedAccuracyRatio;
                     m_valueCalibrationCount = valueCalibrationCount;
                     return true;
                  }
               }
            }
         }
      }
   }
   return false;
}


void PE::COdometer::AddSpeed(const TTimestamp& ts, const TValue& speed, const TAccuracy& acc)
{
   if ( false == m_isInitOk )
   {
      return;
   }
   if ( 0 == m_SpeedTs )
   {
      m_SpeedTs = ts;
   }
   else
   {
      TTimestamp deltatime = ts - m_SpeedTs;
      if ( true == IsSpeedOk(deltatime, speed, acc) )
      {
         m_Speed = speed;
         m_SpeedTs = ts;
      }
      else
      {
         ResetUncomplitedProcessing();
      }
   }
}


void PE::COdometer::AddOdo(const TTimestamp& ts, const TValue& ticks, bool IsValid )
{
   if ( false == m_isInitOk )
   {
      return;
   }
   if ( 0 < m_SpeedTs )
   {
      if ( 0 == m_OdoTs )
      {
         m_OdoTs = ts;
      }
      else
      {
         TTimestamp deltatime = ts - m_OdoTs;
         if ( true == IsOdoOk(deltatime, ticks, IsValid) )
         {
            TValue currentOdoTickSpeed = ticks / deltatime;
            if ( true == IsCalibrationPossible(m_SpeedTs, m_Speed, m_OdoTs, m_OdoTickSpeed, ts, currentOdoTickSpeed) )
            {
               m_OdoCalib.AddRef( m_Speed );
               m_OdoCalib.AddRaw( PredictOdoTickSpeed( m_SpeedTs, m_OdoTs, ts, m_OdoTickSpeed, currentOdoTickSpeed ) );
               m_OdoCalib.Recalculate();
               UpdateBias( m_OdoCalib.GetBias() );
               UpdateScale( m_OdoCalib.GetScale() );
            }
            m_OdoTickSpeed = currentOdoTickSpeed;
            m_OdoTs = ts;
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
      m_OdoTs = 0;
   }
}


const TTimestamp& PE::COdometer::GetOdoTimeStamp() const
{
   return m_OdoTs;
}


SBasicSensor PE::COdometer::GetOdoSpeed()
{
   return CalculateOdoSpeed(m_OdoTickSpeed);
}


const TValue& PE::COdometer::GetOdoBias() const
{
   return m_OdoBias.GetMean();
}


const TValue& PE::COdometer::BiasCalibartedTo() const
{
   return m_OdoBias.GetReliable();
}


const TValue& PE::COdometer::GetOdoScale() const
{
   return m_OdoScale.GetMean();
}


const TValue& PE::COdometer::ScaleCalibartedTo() const
{
   return m_OdoScale.GetReliable();
}


void PE::COdometer::ResetUncomplitedProcessing()
{
   m_SpeedTs = 0;
   m_Speed = 0;
   m_OdoTs = 0;
   m_OdoTickSpeed = 0;
   m_OdoCalib.CleanLastStep();
}


SBasicSensor PE::COdometer::CalculateOdoSpeed( const TValue& odoTickSpeed )
{
   SBasicSensor odoSpeed;//invalid by default
   if ( true == IsOdoCalibrated() )
   {
      odoSpeed.Value    = m_OdoScale.GetMean() * (odoTickSpeed - m_OdoBias.GetMean()); // value = scale * (odo_speed - bias)
      odoSpeed.Accuracy = m_OdoBias.GetMld() * (m_OdoScale.GetMean() + m_OdoScale.GetMld()); // accuracy = bias_mld * (scale + scale_mld)
   }
   return odoSpeed;
}


TValue PE::COdometer::PredictOdoTickSpeed( const TTimestamp& referenceSpeedTs, const TTimestamp& odoTsBefore, const TTimestamp& odoTsAfter, const TValue& odoTickSpeedBefore, const TValue& odoTickSpeedAfter )
{
   //to be more precise better to use Exponential function then proportional
   //for short distance we can ignore the differences
   TTimestamp deltaOdoTs = odoTsAfter - odoTsBefore;
   TTimestamp deltaSpeedTs = referenceSpeedTs - odoTsBefore;
   TValue deltaOdoTickSpeed = odoTickSpeedAfter - odoTickSpeedBefore;
   TValue odoTickSpeed = deltaOdoTickSpeed * deltaSpeedTs / deltaOdoTs  + odoTickSpeedBefore;
   return odoTickSpeed;
}


void PE::COdometer::UpdateBias(const TValue& bias)
{
   if ( false == PE::isnan(bias) )
   {
      m_OdoBias.AddSensor(bias);
      //printf("\nBias=%0.6f",bias);
   }
}


void PE::COdometer::UpdateScale(const TValue& scale)
{
   if ( false == PE::isnan(scale) )
   {
      m_OdoScale.AddSensor(scale);
      //printf("\nScal=%0.6f",scale);
   }
}


bool PE::COdometer::IsSpeedOk( const TTimestamp& deltaTs, const TValue& speed, const TAccuracy& acc)
{
   if ( PE::EPSILON < speed ) //speed has to be always more then zero
   {
      if ( true == IsIntervalOk(deltaTs, m_speedInterval, m_speedInterval / 10) ) //hysteresis 10% - has to be adjusted
      {
         return IsAccuracyOk(speed, acc, m_speedAccuracyRatio);
      }
   }
   return false;
}


bool PE::COdometer::IsOdoOk(const TTimestamp& deltaTs, const TValue& ticks, bool IsValid )
{
   if ( true == IsValid )
   {
      if ( PE::EPSILON < ticks )
      {
         return IsIntervalOk(deltaTs, m_odoInterval, m_odoInterval / 20); //hysteresis 5% - has to be adjusted
      }
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
   if ( m_biasLimit < BiasCalibartedTo() ) // has to be adjustable
   {
      if ( m_scaleLimit < ScaleCalibartedTo() ) // has to be adjustable
      {
         return true;
      }
   }
   return false;
}


bool PE::COdometer::IsInRange(const TTimestamp& testedTS, const TTimestamp& beginTS, const TTimestamp& endTS)
{
   if ( beginTS <= testedTS )
   {
      if ( endTS >= testedTS )
      {
         return true;
      }
   }
   return false;
}


bool PE::COdometer::IsCalibrationPossible( const TTimestamp& speedTs, const TValue& speed, const TTimestamp& OdoTsBefore, const TValue& OdoTickSpeedBefore, const TTimestamp& OdoTsAfter, const TValue& OdoTickSpeedAfter )
{
   if ( PE::EPSILON < speed )
   {
      if ( PE::EPSILON < OdoTickSpeedBefore )
      {
         if ( PE::EPSILON < OdoTickSpeedAfter )
         {
            if ( IsInRange(speedTs, OdoTsBefore, OdoTsAfter) )
            {
               return true;
            }
         }
      }
   }
   return false;
}
