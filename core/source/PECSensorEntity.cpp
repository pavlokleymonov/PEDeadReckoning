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
, mpScaleNorm(new CNormalisation(cfg.GetScale().mAccValue, cfg.GetScale().mAccMld, cfg.GetScale().mAccRel, cfg.GetScale().mCount ))
, mScaleCalib(mpScaleNorm, ratio, threshold)
, mpBaseNorm(new CNormalisation(cfg.GetBase().mAccValue, cfg.GetBase().mAccMld, cfg.GetBase().mAccRel, cfg.GetBase().mCount ))
, mBaseCalib(mpBaseNorm,ratio,threshold)
{
}


PE::CSensorEntity::~CSensorEntity()
{
   delete mpScaleNorm;
   mpScaleNorm = 0;
   delete mpBaseNorm;
   mpBaseNorm = 0;
}


const CNormalisation& PE::CSensorEntity::GetScale() const
{
   return *mpScaleNorm;
}


const CNormalisation& PE::CSensorEntity::GetBase() const
{
   return *mpBaseNorm;
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


void PE::CSensorEntity::AddReference( const SBasicSensor& ref )
{
   mScaleCalib.AddReference(ref);
   if ( mLimit <= mpScaleNorm->GetReliable() )
   {
      mBaseCalib.AddReference(ref);
   }
}


void PE::CSensorEntity::AddSensor( const SBasicSensor& raw )
{
   mScaleCalib.AddSensor(raw);
   if ( mLimit <= mpScaleNorm->GetReliable() )
   {
      //Add unscaled value
      mBaseCalib.AddSensor(mScaleCalib.GetSensor(raw));
   }
}


SBasicSensor PE::CSensorEntity::GetSensor( const SBasicSensor& raw ) const
{
   return mBaseCalib.GetSensor(mScaleCalib.GetSensor(raw));
}


