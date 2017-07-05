/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2017 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */

#include "PECalibrationBase.h"
#include "PETools.h"
#include <math.h>


using namespace PE;


PE::calibration_base::calibration_base(PE::normalisation& norm, const TValue& accuracy_limit)
: m_norm(norm)
, m_accuracy_limit(accuracy_limit)
, m_last_ref_zero(false)
, m_last_sen_valid(false)
, m_sen_chunk(0)
, m_sen_chunk_cnt(0)
{
}


void PE::calibration_base::add_reference(const TValue& value, const TValue& accuracy)
{
   m_last_ref_zero = false;
   if ( m_accuracy_limit > accuracy )
   {
      if ( 0 == value )
      {
         m_last_ref_zero = true;
         if ( m_last_sen_valid )
         {
            m_norm.add_sensor(m_sen_chunk / m_sen_chunk_cnt);
         }
      }
   }
   m_last_sen_valid = false;
   m_sen_chunk = 0;
   m_sen_chunk_cnt = 0;
}


void PE::calibration_base::add_sensor(const TValue& value)
{
   if ( m_last_ref_zero )
   {
      m_last_sen_valid = true;
      m_sen_chunk += value;
      m_sen_chunk_cnt += 1;
   }
}


const TValue PE::calibration_base::get_base() const
{
   return m_norm.get_mean();
}


const TValue PE::calibration_base::get_accuracy() const
{
   return m_norm.get_sigma()*3;
}


const TValue PE::calibration_base::get_calibration() const
{
   return m_norm.get_reliable();
}

