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
#include "PETools.h"
#include <math.h>


using namespace PE;

PE::normalisation::normalisation(const TValue& init_null)
: m_null(init_null)
, m_reliable(0)
, m_sensor_accumulated_steps(0)
, m_sensor_accumulated_count(0)
{
}

PE::normalisation::~normalisation()
{
}

void PE::normalisation::add_sensor_steps(const TValue& sensor_steps)
{
   m_sensor_accumulated_steps += sensor_steps;
   m_sensor_accumulated_count++;
   TValue new_null = m_sensor_accumulated_steps / m_sensor_accumulated_count;
   m_reliable = PE::TOOLS::to_convergence(m_null,new_null);
   m_null = new_null;
}

const TValue& PE::normalisation::get_null()
{
   return m_null;
}

const TValue& PE::normalisation::get_null_reliable()
{
   return m_reliable;
}

