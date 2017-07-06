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

#include "PECalibrationScale.h"
#include "PETools.h"
#include <math.h>


using namespace PE;


PE::calibration_scale::calibration_scale(PE::normalisation& norm, const TValue& accuracy_limit)
: m_norm(norm)
, m_accuracy_limit(accuracy_limit)
, m_last_ref_valid(false)
, m_last_sen_valid(false)
, m_ref_accumulated(0)
, m_sen_accumulated(0)
, m_sen_chunk(0)
{
}


void PE::calibration_scale::add_reference(const TValue& value, const TValue& accuracy)
{
   if ( m_accuracy_limit > accuracy )
   {
      if ( m_last_ref_valid )
      {
         if ( m_last_sen_valid )
         {
            calc_scale(value);
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


void PE::calibration_scale::add_sensor(const TValue& value)
{
   m_last_sen_valid = true;
   if ( m_last_ref_valid )
   {
      m_sen_chunk += value;
   }
}


const TValue PE::calibration_scale::get_scale() const
{
   return m_norm.get_mean();
}


const TValue PE::calibration_scale::get_accuracy() const
{
   return m_norm.get_mld();
}


const TValue PE::calibration_scale::get_calibration() const
{
   return m_norm.get_reliable();
}


void PE::calibration_scale::calc_scale(const TValue& value)
{
   m_sen_accumulated += m_sen_chunk;
   m_ref_accumulated += value;
   m_norm.add_sensor( m_sen_accumulated / m_ref_accumulated );
   m_sen_chunk = 0;
}

