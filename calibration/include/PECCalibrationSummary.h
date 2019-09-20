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
#ifndef __PE_CCalibrationSummary_H__
#define __PE_CCalibrationSummary_H__

#include "PEICalibration.h"

namespace PE
{

/**
 * class for calibartion functionality
 */
class CCalibrationSummary : public ICalibration
{
public:
   /**
    * Constructor of calibartion
    */
   CCalibrationSummary();
   /**
    * Destructor of calibartion
    */
   virtual ~CCalibrationSummary();
   /**
    * Adds new reference value to the calibration
    *
    * @param  ref      reference value
    */
   virtual void AddRef( const TValue& ref );
   /**
    * Adds new raw value to the calibration
    *
    * @param  raw     raw value
    */
   virtual void AddRaw( const TValue& raw );
   /**
    * Returns bias/base of the calibration
    * @return         bias/base or NaN if there is no valid data
    */
   virtual const TValue& GetBias() const;
   /**
    * Returns scale of the calibration
    * @return         scale or NaN if there is no valid data
    */
   virtual const TValue& GetScale() const;
   /**
    * Calculates bias/base and scale of raw value.
    */
   virtual void Recalculate();

private:

   TValue m_Sum_Ref_before;
   TValue m_Sum_Raw_before;
   uint32_t m_Index_before;

   TValue m_Sum_Ref_now;
   TValue m_Sum_Raw_now;
   uint32_t m_Index_now;

   TValue m_Divisor;

   TValue m_Bias;
   TValue m_Scale;

   void CalculateBaseScale();
};

} //namespace PE


PE::CCalibrationSummary::CCalibrationSummary()
: m_Sum_Ref_before(0.0)
, m_Sum_Raw_before(0.0)
, m_Index_before(0)
, m_Sum_Ref_now(0.0)
, m_Sum_Raw_now(0.0)
, m_Index_now(0)
, m_Divisor(0.0)
, m_Bias(std::numeric_limits<TValue>::quiet_NaN())
, m_Scale(std::numeric_limits<TValue>::quiet_NaN())
{
}


PE::CCalibrationSummary::~CCalibrationSummary()
{
}


void PE::CCalibrationSummary::AddRef( const PE::TValue& ref )
{
   m_Sum_Ref_now += ref;
}


void PE::CCalibrationSummary::AddRaw( const PE::TValue& raw )
{
   m_Sum_Raw_now += raw;
   ++m_Index_now;
}


const PE::TValue& PE::CCalibrationSummary::GetBias() const
{
   return m_Bias;
}


const PE::TValue& PE::CCalibrationSummary::GetScale() const
{
   return m_Scale;
}


void PE::CCalibrationSummary::Recalculate()
{
   if ( 0 == m_Index_before )
   {
      m_Sum_Ref_before = m_Sum_Ref_now
      m_Sum_Raw_before = m_Sum_Raw_now;
      m_Index_before   = m_Index_now;
   }
   else
   {
      CalculateBaseScale()
   }
}


void PE::CCalibrationSummary::CalculateBaseScale()
{
   m_Divisor = ( m_Index_before * m_Sum_Ref_now - m_Index_now * m_Sum_Ref_before );
   if ( false == isepsilon( m_Divisor ) )
   {
      m_Bias = ( m_Sum_Ref_now * m_Sum_Raw_before - m_Sum_Raw_now * m_Sum_Ref_before ) / m_Divisor;

      m_Divisor = m_Sum_Raw_now - m_Bias * m_Index_now;

      if ( false == isepsilon( m_Divisor ) )
      {
         m_Scale = m_Sum_Ref_now / m_Divisor;
         m_Sum_Ref_before = m_Sum_Ref_now
         m_Sum_Raw_before = m_Sum_Raw_now;
         m_Index_before   = m_Index_now;
         return;
      }
   }
   m_Bias  = std::numeric_limits<TValue>::quiet_NaN();
   m_Scale = std::numeric_limits<TValue>::quiet_NaN();
}


#endif //__PE_CCalibrationSummary_H__
