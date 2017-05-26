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

#include "PECalibrationEx.h"
#include "PETools.h"
#include <math.h>


using namespace PE;


PE::calibration_ex::calibration_ex(PE::normalisation& norm, const TValue& accuracy_limit)
: m_norm(norm)
, m_accuracy_limit(accuracy_limit)
, m_last_ref_valid(false)
, m_last_sen_valid(false)
, m_ref_accumulated(0)
, m_sen_accumulated(0)
, m_sen_chunk(0)
{
}


void PE::calibration_ex::add_reference(const TValue& value, const TValue& accuracy)
{
   if ( m_accuracy_limit > accuracy )
   {
      if ( m_last_ref_valid )
      {
         if ( m_last_sen_valid )
         {
            m_sen_accumulated += m_sen_chunk;
            m_ref_accumulated += value;
            m_norm.add_sensor( m_sen_accumulated / m_ref_accumulated );
            m_sen_chunk = 0;
         }
      }
      else
      {
         m_last_ref_valid = true;
      }
   }
   else
   {
      m_last_ref_valid = false;
      m_sen_chunk = 0;
   }
}


void PE::calibration_ex::add_sensor(const TValue& value)
{
   m_last_sen_valid = true;
   if ( m_last_ref_valid )
   {
      m_sen_chunk += value;
   }
}


const TValue PE::calibration_ex::get_scale() const
{
   return m_norm.get_mean();
}


const TValue PE::calibration_ex::get_accuracy() const
{
   return m_norm.get_sigma()*3;
}


const TValue PE::calibration_ex::get_calibration() const
{
   return m_norm.get_reliable();
}

