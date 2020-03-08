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


PE::CGyroscope::CGyroscope( const double& headInterval,
                            const double& headHysteresis,
                            const double& headMin,
                            const double& headMax,
                            const double& headAccuracyRatio,
                            const double& gyroInterval,
                            const double& gyroHysteresis,
                            const double& gyroMin,
                            const double& gyroMax)
: m_sensor(*this)
, m_headValue(std::numeric_limits<double>::quiet_NaN())
, m_headAccuracy(std::numeric_limits<double>::quiet_NaN())
, m_headAngularVelocity(std::numeric_limits<double>::quiet_NaN())
, m_gyroValue(std::numeric_limits<double>::quiet_NaN())
, m_gyroValid(false)
, m_gyroAngularVelocity(std::numeric_limits<double>::quiet_NaN())
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


bool PE::CGyroscope::AddHeading(const double& ts, const double& head, const double& acc)
{
   return m_sensor.AddRef(ts, head, acc);
}


bool PE::CGyroscope::AddGyro(const double& ts, const double& gyro, bool isValid )
{
   return m_sensor.AddSen(ts, gyro, isValid);
}


const double& PE::CGyroscope::TimeStamp() const
{
   return m_sensor.GetSenTimeStamp();
}


const double PE::CGyroscope::Value() const
{
   return m_sensor.GetScale().GetMean() * ( m_gyroValue - m_sensor.GetBias().GetMean()); //value = scale * (gyro - bias)
}


const double PE::CGyroscope::Accuracy() const
{
   return m_sensor.GetBias().GetMld() * ( fabs(m_sensor.GetScale().GetMean()) + m_sensor.GetScale().GetMld() ); // accuracy = bias_mld * (|scale| + scale_mld)
}


const double& PE::CGyroscope::Base() const
{
   return m_sensor.GetBias().GetMean();
}


const double& PE::CGyroscope::Scale() const
{
   return m_sensor.GetScale().GetMean();
}


const double& PE::CGyroscope::CalibratedTo() const
{
   return m_sensor.GetBias().GetReliable(); //consider only calibration status of the base of gyro
}


bool PE::CGyroscope::SetRefValue(const double& oldHeadTS, const double& newHeadTS, const double& head, const double& acc)
{
   m_headAngularVelocity = std::numeric_limits<double>::quiet_NaN();
   if ( PE::Sensor::IsInRange(head, m_headMin, m_headMax) )
   {
      double deltaTS = newHeadTS - oldHeadTS;
      if ( PE::Sensor::IsIntervalOk(deltaTS, m_headInterval, m_headHysteresis) )
      {
         if ( false == PE::isnan(m_headValue) )
         {
            double    angle         = TOOLS::ToAngle(m_headValue, head);
            double angleAccuracy = (m_headAccuracy + acc);
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
   m_headValue = std::numeric_limits<double>::quiet_NaN();
   return false;
}


const double& PE::CGyroscope::GetRefValue() const
{
   return m_headAngularVelocity;
}


bool PE::CGyroscope::SetSenValue(const double& oldHeadTS, const double& oldGyroTS, const double& newGyroTS, const double& gyro, bool IsValid)
{
   m_gyroAngularVelocity = std::numeric_limits<double>::quiet_NaN();
   if ( IsValid )
   {
      if ( PE::Sensor::IsInRange(gyro, m_gyroMin, m_gyroMax) )
      {
         double deltaTS = newGyroTS - oldGyroTS;
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


const double& PE::CGyroscope::GetSenValue() const
{
   return m_gyroAngularVelocity;
}
