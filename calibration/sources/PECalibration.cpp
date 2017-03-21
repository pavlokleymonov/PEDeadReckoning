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

#include "PECalibration.h"
#include "PETools.h"
#include <math.h>


#define CALIBRATION_RELIABLE_SCALE_FACTOR 0.01

using namespace PE;


PE::calibration::calibration(const TValue& init_values_per_step, const TAccuracy& accuracy_limit)
: m_values_per_step(init_values_per_step)
, m_sensor_accuracy(PE::MAX_ACCURACY)
, m_sensor_calibration(0)
, m_accuracy_limit(accuracy_limit)
{
   _clear_reference();
   _clear_sensor();
}

void PE::calibration::add_reference_value(const TValue& reference_values, const TAccuracy& reference_accuracy)
{
   if ( m_accuracy_limit < reference_accuracy )
   {
      _clear_sensor();
      _clear_reference();
   }
   else if ( 0 == m_reference_accumulated_count )
   {
      m_reference_accumulated_values = reference_values;
      m_reference_accumulated_accuracy = reference_accuracy;
      m_reference_accumulated_count = 1;
   }
   else
   {
      m_reference_accumulated_values += reference_values;
      m_reference_accumulated_accuracy += reference_accuracy;
      m_reference_accumulated_count++;
      _process();
   }
}


void PE::calibration::add_sensor_steps(const TValue& sensor_steps, const TAccuracy& sensor_accuracy)
{
   m_sensor_accumulated_steps += sensor_steps;
   m_sensor_accumulated_accuracy += sensor_accuracy;
   m_sensor_accumulated_count++;
}


const TValue& PE::calibration::get_values_per_step() const
{
   return m_values_per_step;
}


const TAccuracy& PE::calibration::get_sensor_accuracy() const
{
   return m_sensor_accuracy;
}


const TValue& PE::calibration::get_sensor_calibration() const
{
   return m_sensor_calibration;
}


void PE::calibration::_clear_reference()
{
   m_reference_accumulated_values = 0;
   m_reference_accumulated_accuracy = 0;
   m_reference_accumulated_count = 0;
}


TAccuracy PE::calibration::_get_reference_accuracy()
{
   if ( 0 >= m_reference_accumulated_count )
   {
      return PE::MAX_ACCURACY;
   }
   else if ( 0 >= m_reference_accumulated_accuracy )
   {
      return PE::MAX_ACCURACY;
   }
   else
   {
      return m_reference_accumulated_accuracy / m_reference_accumulated_count;
   }
}


void PE::calibration::_clear_sensor()
{
   m_sensor_accumulated_steps = 0;
   m_sensor_accumulated_accuracy = 0;
   m_sensor_accumulated_count = 0;
}


TAccuracy PE::calibration::_get_sensor_accuracy()
{
   if ( 0 >= m_sensor_accumulated_count )
   {
      return PE::MAX_ACCURACY;
   }
   else if ( 0 >= m_sensor_accumulated_accuracy )
   {
      return PE::MAX_ACCURACY;
   }
   else
   {
      return m_sensor_accumulated_accuracy / m_sensor_accumulated_count;
   }
}


void PE::calibration::_process()
{
   if ( fabs(m_reference_accumulated_values * CALIBRATION_RELIABLE_SCALE_FACTOR) > _get_reference_accuracy() )
   {
      if ( fabs(m_sensor_accumulated_steps * CALIBRATION_RELIABLE_SCALE_FACTOR) > _get_sensor_accuracy() )
      {
         TValue raw_values_per_step = m_reference_accumulated_values / m_sensor_accumulated_steps;
         m_sensor_accuracy = _get_reference_accuracy();
         m_sensor_calibration = PE::TOOLS::to_convergence(m_values_per_step,raw_values_per_step);
         m_values_per_step = raw_values_per_step;
      }
   }
}

