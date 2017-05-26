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
, m_accumulated_sigma(0.0)
, m_accumulated_reliable(0.0)
, m_sample_count(0)
{
}

PE::normalisation::normalisation(const TValue& accumulated_value, const TValue& accumulated_sigma, const TValue& accumulated_reliable, const TValue& sample_count)
: m_mean(0.0)
, m_sigma(0.0)
, m_reliable(0.0)
, m_accumulated_value(accumulated_value)
, m_accumulated_sigma(accumulated_sigma)
, m_accumulated_reliable(accumulated_reliable)
, m_sample_count(sample_count)
{
}

PE::normalisation::~normalisation()
{
}

void PE::normalisation::add_sensor(const TValue& value)
{
   if ( 0 < m_sample_count )
   {
      TValue old_mean = m_accumulated_value / m_sample_count;
      m_mean = (m_accumulated_value + value) / (m_sample_count + 1);

      m_accumulated_sigma += fabs( m_mean - value );
      m_sigma = m_accumulated_sigma / m_sample_count;

      TValue delta_mean = fabs( old_mean - m_mean );

      if ( 0.0 == m_sigma )
      {
         m_accumulated_reliable += 100;
      }
      else if ( delta_mean < m_sigma )
      {
         m_accumulated_reliable += 100 - delta_mean / m_sigma * 100;
      }

      m_reliable = m_accumulated_reliable / m_sample_count;
   }
   m_accumulated_value += value;
   m_sample_count += 1;
}

const TValue& PE::normalisation::get_mean() const
{
   return m_mean;
}

const TValue& PE::normalisation::get_sigma() const
{
   return m_sigma;
}

const TValue& PE::normalisation::get_reliable() const
{
   return m_reliable;
}

const TValue& PE::normalisation::get_accumulated_value() const
{
   return m_accumulated_value;
}

const TValue& PE::normalisation::get_accumulated_sigma() const
{
   return m_accumulated_sigma;
}

const TValue& PE::normalisation::get_accumulated_reliable() const
{
   return m_accumulated_reliable;
}

const TValue& PE::normalisation::get_sample_count() const
{
   return m_sample_count;
}

