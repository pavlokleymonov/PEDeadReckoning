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
#include "PECOdometerEx.h"
#include "PESensorTools.h"


using namespace PE;


PE::COdometerEx::COdometerEx( const double& speedInterval,
                              const double& speedHysteresis,
                              const double& speedMin,
                              const double& speedMax,
                              const double& speedAccuracyRatio,
                              const double& odoInterval,
                              const double& odoHysteresis,
                              const double& odoMin,
                              const double& odoMax)
 : m_sensor(*this)
 , m_speed(std::numeric_limits<double>::quiet_NaN())
 , m_ticks(std::numeric_limits<double>::quiet_NaN())
 , m_ticksValid(false)
 , m_ticksPerSecond(std::numeric_limits<double>::quiet_NaN())
 , m_odoLinearVelocity(std::numeric_limits<double>::quiet_NaN())
 , m_speedInterval(speedInterval)
 , m_speedHysteresis(speedHysteresis)
 , m_speedMin(speedMin)
 , m_speedMax(speedMax)
 , m_speedAccuracyRatio(speedAccuracyRatio)
 , m_odoInterval(odoInterval)
 , m_odoHysteresis(odoHysteresis)
 , m_odoMin(odoMin)
 , m_odoMax(odoMax)
{
}


bool PE::COdometerEx::AddSpeed(const double& timestamp, const double& speed, const double& accuracy)
{
   return m_sensor.AddRef(timestamp, speed, accuracy);
}


bool PE::COdometerEx::AddTicks(const double& timestamp, const double& ticks, bool valid )
{
   return m_sensor.AddSen(timestamp, ticks, valid);
}


const double& PE::COdometerEx::TimeStamp() const
{
   return m_sensor.GetSenTimeStamp();
}


const double PE::COdometerEx::Value() const
{
   return m_sensor.GetScale().GetMean() * ( m_ticksPerSecond - m_sensor.GetBias().GetMean()); //value = scale * (m_ticksPerSecond - bias)
}


const double PE::COdometerEx::Accuracy() const
{
   return m_sensor.GetBias().GetMld() * ( fabs(m_sensor.GetScale().GetMean()) + m_sensor.GetScale().GetMld() ); // accuracy = bias_mld * (|scale| + scale_mld)
}

const double& PE::COdometerEx::Base() const
{
   return m_sensor.GetBias().GetMean();
}


const double& PE::COdometerEx::Scale() const
{
   return m_sensor.GetScale().GetMean();
}


const double& PE::COdometerEx::CalibratedTo() const
{
   return m_sensor.GetBias().GetReliable(); //consider only calibration status of the base of odometer
}


bool PE::COdometerEx::SetRefValue(const double& oldSpeedTS, const double& newSpeedTS, const double& speed, const double& accuracy)
{
   m_speed = std::numeric_limits<double>::quiet_NaN();
   if ( PE::Sensor::IsInRange(speed, m_speedMin, m_speedMax) )
   {
      if ( PE::Sensor::IsIntervalOk(newSpeedTS - oldSpeedTS, m_speedInterval, m_speedHysteresis) )
      {
         if ( PE::Sensor::IsAccuracyOk( speed, accuracy, m_speedAccuracyRatio) )
         {
            m_speed = speed;
            //printf("\nSpeed   =%.2f", m_speed);
         }
         return true;
      }
   }
   return false;
}


const double& PE::COdometerEx::GetRefValue() const
{
   return m_speed;
}


bool PE::COdometerEx::SetSenValue(const double& oldSpeedTS, const double& oldTicksTS, const double& newTicksTS, const double& ticks, bool valid)
{
   m_odoLinearVelocity = std::numeric_limits<double>::quiet_NaN();
   if ( valid )
   {
      if ( PE::Sensor::IsInRange(ticks, m_odoMin, m_odoMax) )
      {
         double deltaTS = newTicksTS - oldTicksTS;
         if ( PE::Sensor::IsIntervalOk(deltaTS, m_odoInterval, m_odoHysteresis) )
         {
            double ticksPerSecond = 0;
            if ( m_ticksValid )
            {
               if ( ticks > m_ticks )
               {
                  ticksPerSecond = (ticks - m_ticks) / deltaTS;
               }
               else if ( ticks < m_ticks )
               {
                  ticksPerSecond = (ticks + m_odoMax + 1 - m_ticks) / deltaTS;
               }
               m_odoLinearVelocity = PE::Sensor::PredictValue(oldSpeedTS, oldTicksTS, newTicksTS, m_ticksPerSecond, ticksPerSecond);
               //printf("\nodoSpeed=%.2f ts=%.3f old(%.3f, %.2f) new(%.3f, %.2f)", m_odoLinearVelocity, oldSpeedTS, oldTicksTS, m_ticksPerSecond, newTicksTS, ticksPerSecond);
            }
            m_ticks = ticks;
            m_ticksValid = true;
            m_ticksPerSecond = ticksPerSecond;
            return true;
         }
      }
   }
   m_ticksValid = false;
   return false;
}


const double& PE::COdometerEx::GetSenValue() const
{
   return m_odoLinearVelocity;
}
