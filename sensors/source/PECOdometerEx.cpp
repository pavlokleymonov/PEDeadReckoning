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

#include "PECOdometerEx.h"


using namespace PE;


PE::COdometerEx::COdometerEx()
: m_isInitOk(false)
, m_odoInterval(0)
, m_odoIntervalHysteresis(0)
, m_speedInterval(0)
, m_speedIntervalHysteresis(0)
, m_speedAccRatio(0)
{
}


bool PE::COdometerEx::Init(const TTimestamp& odoInterval, const TTimestamp& odoIntervalHysteresis, const TTimestamp& speedInterval, const TTimestamp& speedIntervalHysteresis, const TValue& speedAccRatio)
{
   m_isInitOk = false;
   if ( PE::EPSILON < odoIntervalHysteresis ) //hysteresis has to be more then zero
   {
      if ( odoIntervalHysteresis < odoInterval ) //all intervals have to be bigger then its hysteresis
      {
         if ( PE::EPSILON < speedIntervalHysteresis ) //hysteresis has to be more then zero
         {
            if ( speedIntervalHysteresis < speedInterval ) //all intervals has to be bigger then its hysteresis
            {
               if ( odoInterval < speedInterval ) //speed interval has to be bigger then odometer interval
               {
                  if ( PE::EPSILON < speedAccRatio ) //ration has to be bigger then zero
                  {
                     m_odoInterval = odoInterval;
                     m_odoIntervalHysteresis = odoIntervalHysteresis;
                     m_speedInterval = speedInterval;
                     m_speedIntervalHysteresis = speedIntervalHysteresis;
                     m_speedAccRatio = speedAccRatio;
                     m_isInitOk = true;
                  }
               }
            }
         }
      }
   }
   return m_isInitOk;
}


bool PE::COdometerEx::AddSpeed(const TTimestamp& timestamp, const TValue& speed, const TAccuracy& acc)
{
   if ( m_isInitOk )
   {
      return AddRef(timestamp, speed, acc);
   }
   return false;
}


bool PE::COdometerEx::AddTicks(const TTimestamp& timestamp, const TValue& ticks, bool valid )
{
   if ( m_isInitOk )
   {
      TTimestamp deltatime = timestamp - m_senTimestamp;
      if ( PE::EPSILON < deltatime )
      {
         return AddSen(timestamp, ticks / deltatime, valid);
      }
   }
   return false;
}


const TTimestamp& PE::COdometerEx::GetTimeStamp() const
{
   return m_senTimestamp;
}


const TValue PE::COdometerEx::GetValue() const
{
   return m_SenScale.GetMean() * (m_senValue - m_SenBias.GetMean()); // value = scale * (odo_speed - bias)
}


const TAccuracy PE::COdometerEx::GetAccuracy() const
{
   return m_SenBias.GetMld() * m_SenScale.GetMean() + m_SenScale.GetMld(); // accuracy = bias_mld * scale + scale_mld
}



const TValue& PE::COdometerEx::CalibratedTo() const
{
   return m_SenBias.GetReliable();
}


bool PE::COdometerEx::IsRefOk( const TTimestamp& refTimestamp, const TValue& refValue, const TAccuracy& refAccuracy) const
{
   if ( PE::EPSILON < refValue ) //speed has to be always more then zero
   {
      if ( IsIntervalOk( refTimestamp - m_refTimestamp, m_speedInterval, m_speedIntervalHysteresis) )
      {
         return IsAccuracyOk(refValue, refAccuracy, m_speedAccRatio);
      }
   }
   return false;
}


bool PE::COdometerEx::IsSenOk(const TTimestamp& senTimestamp, const TValue& senValue, bool senValid ) const
{
   if ( senValid )
   {
      if ( PE::EPSILON < senValue )
      {
         return IsIntervalOk( senTimestamp - m_senTimestamp, m_odoInterval, m_odoIntervalHysteresis)
      }
   }
   return false;
}


bool PE::COdometerEx::IsCalibrationPossible( const TTimestamp& senTimestamp ) const
{
   if ( PE::EPSILON < senTimestamp - m_senTimestamp ) //interval between two sensors timestamps has to be bigger then zero
   {
      if ( senTimestamp < m_refTimestamp ) //reference timestamp is out of range and bigger then last sensor timestamp
      {
         return false;
      }

      if ( m_senTimestamp > m_refTimestamp ) //reference timestamp is out of range and lower then first sensor timestamp
      {
         return false;
      }

      return true;
   }
   return false;
}


bool PE::COdometerEx::IsIntervalOk(const TTimestamp& deltaTs, const TValue& interval, const TValue& hysteresis) const
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


bool PE::COdometerEx::IsAccuracyOk(const TValue& value, const TAccuracy& accuracy, const TValue& ratio) const
{
   if ( value > (accuracy * ratio) )
   {
      return true;
   }
   return false;
}
