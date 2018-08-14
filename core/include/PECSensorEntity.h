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
#ifndef __PE_CSensorEntity_H__
#define __PE_CSensorEntity_H__

#include "PETypes.h"
#include "PECSensorCfg.h"
<<<<<<< HEAD
#include "PECNormalisation.h"
#include "PECCalibrationScale.h"
#include "PECCalibrationBase.h"
=======
>>>>>>> 6122ae5278dcd2e88f474ae363dfc06d42d03d30


class PECSensorEntityTest; //to get possibility for test class

namespace PE
{

class CSensorEntity
{
   friend class ::PECSensorEntityTest;

public:

   CSensorEntity( const CSensorCfg& cfg, TValue ratio, TValue threshold );

   const CNormalisation& GetScale() const;

   const CNormalisation& GetBase() const;

   const TValue& GetLimit() const;

   const TValue& GetRatio() const;

   const TValue& GetThreshold() const;

   const TValue& CalibratedTo() const;

   void AddReference( const SBasicSensor& ref );

   void AddSensor( const SBasicSensor& raw );

   SBasicSensor GetSensor( const SBasicSensor& raw ) const;

private:

   TValue             mLimit;
   TValue             mRatio;
   TValue             mThreshold;

   CNormalisation     mScaleNorm;
   CCalibrationScale  mScaleCalib;

   CNormalisation     mBaseNorm;
   CCalibrationBase   mBaseCalib;

};

} //namespace PE
#endif //__PE_CSensorEntity_H__
