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

PE::normalisation::normalisation(const size_t count)
: m_mean(0)
, m_sigma(0)
, m_reliable(0)
, m_min_sample_count(count)
, m_accumulated_value(0)
, m_accumulated_delta_sigma(0)
, m_sample_count(0)
{
}

PE::normalisation::normalisation(const TValue& init_mean, const TValue& init_sigma, const size_t init_reliable, const size_t count)
: m_mean(init_mean)
, m_sigma(init_sigma)
, m_reliable(init_reliable)
, m_min_sample_count(count)
, m_accumulated_value(init_mean)
, m_accumulated_delta_sigma(init_sigma)
, m_sample_count(1)
{
}

void PE::normalisation::add_sensor(const TValue& value)
{
   if ( 0 != m_sample_count )
   {
      TValue old_mean = m_accumulated_value / m_sample_count;

      TValue new_mean = (m_accumulated_value + value) / (m_sample_count + 1);

      m_accumulated_delta_sigma += fabs(new_mean - value);

      printf("old_mean=%f new_mean=%f m_accumulated_value=%f m_sample_count=%d m_accumulated_delta_sigma=%f\n"
        , old_mean
        , new_mean
        , m_accumulated_value
        , m_sample_count
        , m_accumulated_delta_sigma );

      if ( m_min_sample_count <= m_sample_count )
      {
         TValue delta_mean = fabs(old_mean - new_mean );

         m_mean = new_mean;

         m_sigma = m_accumulated_delta_sigma / m_sample_count;

         if ( 0.0 == m_sigma )
         {
            m_reliable = 100;
         }
         else if ( delta_mean > m_sigma )
         {
            m_reliable = 0;
         }
         else
         {
            m_reliable = floor(100.5 - delta_mean / m_sigma * 100.0);
         }

	 printf("m_mean=%f m_sigma=%f m_reliable=%d\n",m_mean, m_sigma, m_reliable);
      }
   }
   m_accumulated_value += value;
   m_sample_count++;
}


