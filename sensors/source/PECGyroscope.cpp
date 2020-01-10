/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2020 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */

#include <math.h>
#include "PECGyroscope.h"
#include "PETools.h"
#include "PESensorTools.h"


using namespace PE;


PE::CGyroscope::CGyroscope( const TValue& headInterval,
                            const TValue& headHysteresis,
                            const TValue& headMin,
                            const TValue& headMax,
                            const TValue& headAccuracyRatio,
                            const TValue& gyroInterval,
                            const TValue& gyroHysteresis,
                            const TValue& gyroMin,
                            const TValue& gyroMax)
: m_sensor(*this)
, m_headValue(std::numeric_limits<TValue>::quiet_NaN())
, m_headAccuracy(std::numeric_limits<TAccuracy>::quiet_NaN())
, m_headAngularVelocity(std::numeric_limits<TValue>::quiet_NaN())
, m_gyroValue(std::numeric_limits<TValue>::quiet_NaN())
, m_gyroValid(false)
, m_gyroAngularVelocity(std::numeric_limits<TValue>::quiet_NaN())
, m_headInterval(headInterval)
, m_headHysteresis(headHysteresis)
, m_headMin(headMin)
, m_headMax(headMax)
, m_headAccuracyRatio(headAccuracyRatio)
, m_gyroInterval(gyroInterval)
, m_gyroHysteresis(gyroHysteresis)
, m_gyroMin(gyroMin)
, m_gyroMax(gyroMax)
{
}


bool PE::CGyroscope::AddHeading(const TTimestamp& ts, const TValue& head, const TAccuracy& acc)
{
   return m_sensor.AddRef(ts, head, acc);
}


bool PE::CGyroscope::AddGyro(const TTimestamp& ts, const TValue& gyro, bool isValid )
{
   return m_sensor.AddSen(ts, gyro, isValid);
}


const TTimestamp& PE::CGyroscope::TimeStamp() const
{
   return m_sensor.GetSenTimeStamp();
}


const TValue PE::CGyroscope::Value() const
{
   return m_sensor.GetScale().GetMean() * ( m_gyroValue - m_sensor.GetBias().GetMean()); //value = scale * (gyro - bias)
}


const TAccuracy PE::CGyroscope::Accuracy() const
{
   return m_sensor.GetBias().GetMld() * (m_sensor.GetScale().GetMean() + m_sensor.GetScale().GetMld()); // accuracy = bias_mld * (scale + scale_mld)
}


const TValue& PE::CGyroscope::Base() const
{
   return m_sensor.GetBias().GetMean();
}


const TValue& PE::CGyroscope::Scale() const
{
   return m_sensor.GetScale().GetMean();
}


const TValue& PE::CGyroscope::CalibartedTo() const
{
   return m_sensor.GetBias().GetReliable(); //consider only calibration status of the base of gyro
}

bool PE::CGyroscope::SetRefValue(const TTimestamp& oldHeadTS, const TTimestamp& newHeadTS, const TValue& head, const TAccuracy& acc)
{
   m_headAngularVelocity = std::numeric_limits<TValue>::quiet_NaN();
   bool isSuccessfully = false;

   if ( IsAllValueInRange(head, m_headValue, m_headMin, m_headMax) )
   {
      TTimestamp deltaTS = newHeadTS - oldHeadTS;
      if ( PE::Sensor::IsIntervalOk(deltaTS, m_headInterval, m_headHysteresis) )
      {
         TValue    angle = TOOLS::ToAngle(m_headValue, head);
         TAccuracy angleAccuracy = (m_headAccuracy + acc);
         if ( PE::Sensor::IsAccuracyOk( fabs(angle), angleAccuracy, m_headAccuracyRatio) )
         {
            m_headAngularVelocity = angle / deltaTS;
            isSuccessfully = true;
         }
      }
   }
   m_headValue = head;
   m_headAccuracy = acc;
   return isSuccessfully;
}


const TValue& PE::CGyroscope::GetRefValue() const
{
   return m_headAngularVelocity;
}


bool PE::CGyroscope::SetSenValue(const TTimestamp& oldHeadTS, const TTimestamp& oldGyroTS, const TTimestamp& newGyroTS, const TValue& gyro, bool IsValid)
{
   m_gyroAngularVelocity = std::numeric_limits<TValue>::quiet_NaN();
   bool isSuccessfully = false;

   if ( IsValid && m_gyroValid )
   {
      if ( IsAllValueInRange(gyro, m_gyroValue, m_gyroMin, m_gyroMax) )
      {
         if ( PE::Sensor::IsIntervalOk(newGyroTS - oldGyroTS, m_gyroInterval, m_gyroHysteresis) )
         {
            m_gyroAngularVelocity = PE::Sensor::PredictValue(oldHeadTS, oldGyroTS, newGyroTS, m_gyroValue, gyro);
            isSuccessfully = true;
         }
      }
   }
   m_gyroValue = gyro;
   m_gyroValid = IsValid;
   return isSuccessfully;
}


const TValue& PE::CGyroscope::GetSenValue() const
{
   return m_gyroAngularVelocity;
}


bool PE::CGyroscope::IsAllValueInRange(const TValue& v1, const TValue& v2, const TValue& min, const TValue& max) const
{
   if ( v1 >= min )
   {
      if ( v1 <= max )
      {
         if ( v2 >= min )
         {
            if ( v2 <= max )
            {
               return true;
            }
         }
      }
   }
   return false;
}

