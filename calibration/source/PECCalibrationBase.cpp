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

#include "PECCalibrationBase.h"
#include "PETools.h"
#include <math.h>


using namespace PE;


PE::CCalibrationBase::CCalibrationBase(CNormalisation& norm)
: mNorm(norm)
, mBase(0)
, mRefAcc(0.0)
, mRefCnt(0)
, mSenAcc(0.0)
, mSenCnt(0)
{
}


void PE::CCalibrationBase::AddReference(const TValue& value)
{
   mRefAcc += value;
   ++mRefCnt;
}


void PE::CCalibrationBase::AddSensor(const TValue& value)
{
   mSenAcc += value;
   ++mSenCnt;
}


void PE::CCalibrationBase::AddScale(const TValue& value)
{
   mScale = value;
}


const TValue PE::CCalibrationBase::GetBase() const
{
   return mBase;
}


const TValue PE::CCalibrationBase::GetMean() const
{
   return mNorm.GetMean();
}


const TValue PE::CCalibrationBase::GetMld() const
{
   return mNorm.GetMld();
}


const TValue PE::CCalibrationBase::GetReliable() const
{
   return mNorm.GetReliable();
}


void PE::CCalibrationBase::DoCalibration()
{
   if ( 0.0 < mScale  &&
          0 < mSenCnt &&
          0 < mRefCnt)
   {
      mBase = (mSenAcc / mSenCnt) * mScale - (mRefAcc / mRefCnt);
      mNorm.AddSensor(mBase);
   }
}
