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
#include "PESensorTools.h"


using namespace PE;


PE::COdometer::COdometer()
: m_isInitOk(false)
, m_odoInterval(0)
, m_speedInterval(0)
, m_biasLimit(0)
, m_scaleLimit(0)
, m_speedAccuracyRatio(0)
, m_SpeedTs(0)
, m_Speed(0)
, m_OdoTs(0)
, m_OdoTickSpeed(0)
{
}


bool PE::COdometer::Init(const double& odoInterval, const double& speedInterval, const double& biasLimit, const double& scaleLimit, const uint32_t speedAccuracyRatio)
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
                  m_isInitOk = true;
                  m_odoInterval = odoInterval;
                  m_speedInterval = speedInterval;
                  m_biasLimit = biasLimit;
                  m_scaleLimit = scaleLimit;
                  m_speedAccuracyRatio = speedAccuracyRatio;
                  return true;
               }
            }
         }
      }
   }
   return false;
}


void PE::COdometer::AddSpeed(const double& ts, const double& speed, const double& acc)
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
      double deltatime = ts - m_SpeedTs;
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


void PE::COdometer::AddOdo(const double& ts, const double& ticks, bool IsValid )
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
         double deltatime = ts - m_OdoTs;
         if ( true == IsOdoOk(deltatime, ticks, IsValid) )
         {
            double currentOdoTickSpeed = ticks / deltatime;
            if ( true == IsCalibrationPossible(m_SpeedTs, m_Speed, m_OdoTs, m_OdoTickSpeed, ts, currentOdoTickSpeed) )
            {
               m_OdoCalib.AddRef( m_Speed );
               m_OdoCalib.AddRaw( PE::Sensor::PredictValue( m_SpeedTs, m_OdoTs, ts, m_OdoTickSpeed, currentOdoTickSpeed ) );
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


const double& PE::COdometer::GetOdoTimeStamp() const
{
   return m_OdoTs;
}


bool PE::COdometer::IsOdoSpeedCalibrated() const
{
   return IsOdoCalibrated(BiasCalibartedTo(), ScaleCalibartedTo());
}


const double PE::COdometer::GetOdoSpeedValue() const
{
   return m_OdoScale.GetMean() * (m_OdoTickSpeed - m_OdoBias.GetMean()); // value = scale * (odo_speed - bias)
}


const double PE::COdometer::GetOdoSpeedAccuracy() const
{
   return m_OdoBias.GetMld() * (m_OdoScale.GetMean() + m_OdoScale.GetMld()); // accuracy = bias_mld * (scale + scale_mld)
}


const double& PE::COdometer::GetOdoBias() const
{
   return m_OdoBias.GetMean();
}


const double& PE::COdometer::BiasCalibartedTo() const
{
   return m_OdoBias.GetReliable();
}


const double& PE::COdometer::GetOdoScale() const
{
   return m_OdoScale.GetMean();
}


const double& PE::COdometer::ScaleCalibartedTo() const
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


void PE::COdometer::UpdateBias(const double& bias)
{
   if ( false == PE::isnan(bias) )
   {
      m_OdoBias.AddSensor(bias);
      printf("Bias=%0.6f\n",bias);
   }
}


void PE::COdometer::UpdateScale(const double& scale)
{
   if ( false == PE::isnan(scale) )
   {
      m_OdoScale.AddSensor(scale);
      printf("Scal=%0.6f\n",scale);
   }
}


bool PE::COdometer::IsSpeedOk( const double& deltaTs, const double& speed, const double& acc) const
{
   if ( PE::EPSILON < speed ) //speed has to be always more then zero
   {
      if ( true == PE::Sensor::IsIntervalOk(deltaTs, m_speedInterval, m_speedInterval / 10) ) //hysteresis 10% - has to be adjusted
      {
         return PE::Sensor::IsAccuracyOk(speed, acc, m_speedAccuracyRatio);
      }
   }
   return false;
}


bool PE::COdometer::IsOdoOk(const double& deltaTs, const double& ticks, bool IsValid ) const
{
   if ( true == IsValid )
   {
      if ( PE::EPSILON < ticks )
      {
         return PE::Sensor::IsIntervalOk(deltaTs, m_odoInterval, m_odoInterval / 20); //hysteresis 5% - has to be adjusted
      }
   }
   return false;
}


bool PE::COdometer::IsOdoCalibrated(const double& biasCalibartedTo, const double& scaleCalibartedTo) const
{
   if ( m_biasLimit < biasCalibartedTo )
   {
      if ( m_scaleLimit < scaleCalibartedTo )
      {
         return true;
      }
   }
   return false;
}


bool PE::COdometer::IsCalibrationPossible( const double& speedTs, const double& speed, const double& OdoTsBefore, const double& OdoTickSpeedBefore, const double& OdoTsAfter, const double& OdoTickSpeedAfter ) const
{
   if ( PE::EPSILON < speed )
   {
      if ( PE::EPSILON < OdoTickSpeedBefore )
      {
         if ( PE::EPSILON < OdoTickSpeedAfter )
         {
            return PE::Sensor::IsInRange(speedTs, OdoTsBefore, OdoTsAfter);
         }
      }
   }
   return false;
}
