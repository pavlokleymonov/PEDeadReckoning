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
#include "PECSensorEntity.h"


using namespace PE;


PE::CSensorEntity::CSensorEntity( const CSensorCfg& cfg, TValue ratio, TValue threshold )
: mLimit(cfg.GetLimit())
, mRatio(ratio)
, mThreshold(threshold)
, mScaleNorm(CSensorCfg::ToNormalisation(cfg.GetScale()))
, mScaleCalib(mScaleNorm, ratio, threshold)
, mBaseNorm(CSensorCfg::ToNormalisation(cfg.GetBase()))
, mBaseCalib(mBaseNorm, ratio, threshold)
{
}


const CNormalisation& PE::CSensorEntity::GetScale() const
{
   return mScaleNorm;
}


const CNormalisation& PE::CSensorEntity::GetBase() const
{
   return mBaseNorm;
}


const TValue& PE::CSensorEntity::GetLimit() const
{
   return mLimit;
}


const TValue& PE::CSensorEntity::GetRatio() const
{
   return mRatio;
}


const TValue& PE::CSensorEntity::GetThreshold() const
{
   return mThreshold;
}


const TValue& PE::CSensorEntity::CalibratedTo() const
{
   return mBaseCalib.CalibratedTo();
}


void PE::CSensorEntity::AddReference( const SBasicSensor& ref )
{
   mScaleCalib.AddReference(ref);
   if ( mLimit <= mScaleCalib.CalibratedTo() )
   {
      mBaseCalib.AddReference(ref);
   }
}


void PE::CSensorEntity::AddSensor( const SBasicSensor& raw )
{
   mScaleCalib.AddSensor(raw);
   if ( mLimit <= mScaleCalib.CalibratedTo() )
   {
      //Add unscaled value
      mBaseCalib.AddSensor(mScaleCalib.GetSensor(raw));
   }
}


SBasicSensor PE::CSensorEntity::GetSensor( const SBasicSensor& raw ) const
{
   return mBaseCalib.GetSensor(mScaleCalib.GetSensor(raw));
}


