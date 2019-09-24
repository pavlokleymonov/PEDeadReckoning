/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2018 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */
#ifndef __PE_ICalibration_H__
#define __PE_ICalibration_H__

#include "PETypes.h"

namespace PE
{

/**
 * Interface class for calibartion basic functionality
 * The scaled value could be calculated by this formula:
 *
 *   value = ( raw - bias ) * scale
 */
class ICalibration
{
public:
   /**
    * Destructor of calibartion
    */
   virtual ~ICalibration(){};
   /**
    * Adds new reference value to the calibration
    *
    * @param  ref      reference value
    */
   virtual void AddRef( const TValue& ref ) = 0;
   /**
    * Adds new raw value to the calibration
    *
    * @param  raw     raw value
    */
   virtual void AddRaw( const TValue& raw ) = 0;
   /**
    * Returns bias/base of the calibration
    * @return         bias/base or NaN if there is no valid data
    */
   virtual const TValue& GetBias() const = 0;
   /**
    * Returns scale of the calibration
    * @return         scale or NaN if there is no valid data
    */
   virtual const TValue& GetScale() const = 0;
   /**
    * Calculates bias/base and scale of raw value.
    */
   virtual void Recalculate() = 0;
};


} //namespace PE
#endif //__PE_ICalibration_H__
