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
   return m_sensor.GetBias().GetMld() * ( fabs(m_sensor.GetScale().GetMean()) + m_sensor.GetScale().GetMld() ); // accuracy = bias_mld * (|scale| + scale_mld)
}


const TValue& PE::CGyroscope::Base() const
{
   return m_sensor.GetBias().GetMean();
}


const TValue& PE::CGyroscope::Scale() const
{
   return m_sensor.GetScale().GetMean();
}


const TValue& PE::CGyroscope::CalibratedTo() const
{
   return m_sensor.GetBias().GetReliable(); //consider only calibration status of the base of gyro
}


bool PE::CGyroscope::SetRefValue(const TTimestamp& oldHeadTS, const TTimestamp& newHeadTS, const TValue& head, const TAccuracy& acc)
{
   m_headAngularVelocity = std::numeric_limits<TValue>::quiet_NaN();
   if ( PE::Sensor::IsInRange(head, m_headMin, m_headMax) )
   {
      TTimestamp deltaTS = newHeadTS - oldHeadTS;
      if ( PE::Sensor::IsIntervalOk(deltaTS, m_headInterval, m_headHysteresis) )
      {
         if ( false == PE::isnan(m_headValue) )
         {
            TValue    angle         = TOOLS::ToAngle(m_headValue, head);
            TAccuracy angleAccuracy = (m_headAccuracy + acc);
            if ( PE::Sensor::IsAccuracyOk( fabs(angle), angleAccuracy, m_headAccuracyRatio) )
            {
               m_headAngularVelocity = angle / deltaTS;
            }
         }
         m_headValue    = head;
         m_headAccuracy = acc;
         return true;
      }
   }
   m_headValue = std::numeric_limits<TValue>::quiet_NaN();
   return false;
}


const TValue& PE::CGyroscope::GetRefValue() const
{
   return m_headAngularVelocity;
}


bool PE::CGyroscope::SetSenValue(const TTimestamp& oldHeadTS, const TTimestamp& oldGyroTS, const TTimestamp& newGyroTS, const TValue& gyro, bool IsValid)
{
   m_gyroAngularVelocity = std::numeric_limits<TValue>::quiet_NaN();
   if ( IsValid )
   {
      if ( PE::Sensor::IsInRange(gyro, m_gyroMin, m_gyroMax) )
      {
         TTimestamp deltaTS = newGyroTS - oldGyroTS;
         if ( PE::Sensor::IsIntervalOk(deltaTS, m_gyroInterval, m_gyroHysteresis) )
         {
            if ( m_gyroValid )
            {
               m_gyroAngularVelocity = PE::Sensor::PredictValue(oldHeadTS, oldGyroTS, newGyroTS, m_gyroValue, gyro);
            }
            m_gyroValue = gyro;
            m_gyroValid = true;
            return true;
         }
      }
   }
   m_gyroValid = false;
   return false;
}


const TValue& PE::CGyroscope::GetSenValue() const
{
   return m_gyroAngularVelocity;
}
