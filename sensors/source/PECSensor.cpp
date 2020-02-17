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

#include "PECSensor.h"
#include "PESensorTools.h"


using namespace PE;


PE::CSensor::CSensor(ISensorAdjuster& adjuster)
: m_adjuster(adjuster)
, m_refTimestamp(0)
, m_senTimestamp(0)
{
}


bool PE::CSensor::AddRef(const TTimestamp& refTimestamp, const TValue& refValue, const TAccuracy& refAccuracy)
{
   if ( 0 == m_refTimestamp )
   {
      m_refTimestamp = refTimestamp;
   }
   else
   {
      if ( true == m_adjuster.SetRefValue(m_refTimestamp, refTimestamp, refValue, refAccuracy) )
      {
         m_refTimestamp = refTimestamp;
         return true;
      }
      else
      {
         ResetUncomplitedProcessing();
      }
   }
   return false;
}


bool PE::CSensor::AddSen(const TTimestamp& senTimestamp, const TValue& senValue, bool senValid )
{
   if ( 0 < m_refTimestamp )
   {
      if ( 0 == m_senTimestamp )
      {
         m_senTimestamp = senTimestamp;
      }
      else
      {
         if ( true == m_adjuster.SetSenValue(m_refTimestamp, m_senTimestamp, senTimestamp, senValue, senValid) )
         {
            if ( PE::Sensor::IsInRange(m_refTimestamp, m_senTimestamp, senTimestamp) )
            {
               //printf("\n=== TS:%.3f REF:%.2f SEN:%.2f", m_refTimestamp, m_adjuster.GetRefValue(), m_adjuster.GetSenValue());
               if ( false == PE::isnan(m_adjuster.GetRefValue()) && 
                    false == PE::isnan(m_adjuster.GetSenValue()) )
               {
                  m_Calibration.AddRef(m_adjuster.GetRefValue());
                  m_Calibration.AddRaw(m_adjuster.GetSenValue());
                  m_Calibration.Recalculate();
                  UpdateBias( m_Calibration.GetBias() );
                  UpdateScale( m_Calibration.GetScale() );
               }
            }
            m_senTimestamp = senTimestamp;
            return true;
         }
         else
         {
            ResetUncomplitedProcessing();
         }
      }
   }
   else
   {
      m_senTimestamp = 0;
   }
   return false;
}


const TTimestamp& PE::CSensor::GetRefTimeStamp() const
{
   return m_refTimestamp;
}


const TTimestamp& PE::CSensor::GetSenTimeStamp() const
{
   return m_senTimestamp;
}


const CNormalisation& PE::CSensor::GetBias() const
{
   return m_SenBias;
}


const CNormalisation& PE::CSensor::GetScale() const
{
   return m_SenScale;
}


void PE::CSensor::ResetUncomplitedProcessing()
{
   m_refTimestamp = 0;
   m_senTimestamp = 0;
   m_Calibration.CleanLastStep();
}


void PE::CSensor::UpdateBias(const TValue& bias)
{
   if ( false == PE::isnan(bias) )
   {
      m_SenBias.AddSensor(bias);
   }
}


void PE::CSensor::UpdateScale(const TValue& scale)
{
   if ( false == PE::isnan(scale) )
   {
      m_SenScale.AddSensor(scale);
   }
}
