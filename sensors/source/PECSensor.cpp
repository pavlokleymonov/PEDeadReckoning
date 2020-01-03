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


using namespace PE;


PE::CSensor::CSensor()
: m_refTimestamp(0)
, m_refValue(0)
, m_senTimestamp(0)
, m_senValue(0)
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
      TValue adj_refValue = AdjustRefValue(m_refTimestamp, refTimestamp, refValue, refAccuracy);
      if ( false == PE::isnan(adj_refValue) )
      {
         m_refTimestamp = refTimestamp;
         m_refValue     = adj_refValue;
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
         TValue adj_senValue = AdjustSenValue(m_senTimestamp, senTimestamp, senValue, senValid);
         if ( false == PE::isnan(adj_senValue) )
         {
            if ( PE::Sensor::IsInRange(m_refTimestamp, m_senTimestamp, senTimestamp) )
            {
               m_SenCalib.AddRef( m_refValue );
               m_SenCalib.AddRaw( PE::Sensor::PredictValue( m_refTimestamp, m_senTimestamp, senTimestamp, m_senValue, adj_senValue ) );
               m_SenCalib.Recalculate();
               UpdateBias( m_SenCalib.GetBias() );
               UpdateScale( m_SenCalib.GetScale() );
            }
            m_senTimestamp = senTimestamp;
            m_senValue     = adj_senValue;
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


void PE::CSensor::ResetUncomplitedProcessing()
{
   m_refTimestamp = 0;
   m_refValue     = 0;
   m_senTimestamp = 0;
   m_senValue     = 0;
   m_SenCalib.CleanLastStep();
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
