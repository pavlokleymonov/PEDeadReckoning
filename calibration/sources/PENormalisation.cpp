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

#include "PENormalisation.h"
#include <math.h>


using namespace PE;

PE::normalisation::normalisation()
: m_mean(0.0)
, m_sigma(0.0)
, m_reliable(0.0)
, m_accumulated_value(0.0)
, m_accumulated_delta_sigma(0.0)
, m_accumulated_delta_reliable(0.0)
, m_sample_count(0)
{
}

PE::normalisation::normalisation(const TValue& init_mean, const TValue& init_sigma, const TValue init_reliable)
: m_mean(init_mean)
, m_sigma(init_sigma)
, m_reliable(init_reliable)
, m_accumulated_value(init_mean)
, m_accumulated_delta_sigma(init_sigma)
, m_accumulated_delta_reliable(init_reliable)
, m_sample_count(1)
{
}

void PE::normalisation::add_sensor(const TValue& value)
{
   if ( 0 < m_sample_count )
   {
      TValue old_mean = m_accumulated_value / m_sample_count;
      m_mean = (m_accumulated_value + value) / (m_sample_count + 1);

      m_accumulated_delta_sigma += fabs( m_mean - value );
      m_sigma = m_accumulated_delta_sigma / m_sample_count;

      TValue delta_mean = fabs( old_mean - m_mean );

      if ( 0.0 == m_sigma )
      {
         m_accumulated_delta_reliable += 100;
      }
      else if ( delta_mean < m_sigma )
      {
         m_accumulated_delta_reliable += 100 - delta_mean / m_sigma * 100;
      }

      m_reliable = m_accumulated_delta_reliable / m_sample_count;
   }
   m_accumulated_value += value;
   m_sample_count += 1;
}


