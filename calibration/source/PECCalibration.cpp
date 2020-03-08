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

#include "PECCalibration.h"


using namespace PE;


PE::CCalibration::CCalibration()
: m_Sum_Ref_before(0.0)
, m_Sum_Raw_before(0.0)
, m_Index_before(0)
, m_Sum_Ref_now(0.0)
, m_Sum_Raw_now(0.0)
, m_Index_now(0)
, m_Bias(std::numeric_limits<double>::quiet_NaN())
, m_Scale(std::numeric_limits<double>::quiet_NaN())
{
}


void PE::CCalibration::AddRef( const double& ref )
{
   m_Sum_Ref_now += ref;
}


void PE::CCalibration::AddRaw( const double& raw )
{
   m_Sum_Raw_now += raw;
   ++m_Index_now;
}


const double& PE::CCalibration::GetBias() const
{
   return m_Bias;
}


const double& PE::CCalibration::GetScale() const
{
   return m_Scale;
}


void PE::CCalibration::Recalculate()
{
   if ( 0 == m_Index_before )
   {
      m_Sum_Ref_before = m_Sum_Ref_now;
      m_Sum_Raw_before = m_Sum_Raw_now;
      m_Index_before   = m_Index_now;
   }
   else
   {
      CalculateBaseScale();
   }
}


void PE::CCalibration::CleanLastStep()
{
   m_Sum_Ref_now = m_Sum_Ref_before;
   m_Sum_Raw_now = m_Sum_Raw_before;
   m_Index_now = m_Index_before;
}


void PE::CCalibration::CalculateBaseScale()
{
   double divisor = ( m_Index_before * m_Sum_Ref_now - m_Index_now * m_Sum_Ref_before );
   if ( false == isepsilon( divisor ) )
   {
      m_Bias = ( m_Sum_Ref_now * m_Sum_Raw_before - m_Sum_Raw_now * m_Sum_Ref_before ) / divisor;

      divisor = m_Sum_Raw_now - m_Bias * m_Index_now;

      if ( false == isepsilon( divisor ) )
      {
         m_Scale = m_Sum_Ref_now / divisor;
         m_Sum_Ref_before = m_Sum_Ref_now;
         m_Sum_Raw_before = m_Sum_Raw_now;
         m_Index_before   = m_Index_now;
         return;
      }
   }
   m_Bias  = std::numeric_limits<double>::quiet_NaN();
   m_Scale = std::numeric_limits<double>::quiet_NaN();
}
