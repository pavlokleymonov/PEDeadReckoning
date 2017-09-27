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


PE::CCalibrationBase::CCalibrationBase(PE::CNormalisation& norm, const TValue& limit)
: mNorm(norm)
, mAccuracyLimit(limit)
, mLastRefZero(false)
, mLastSenValid(false)
, mSenChunk(0)
, mSenChunkCnt(0)
{
}


void PE::CCalibrationBase::AddReference(const TValue& value, const TValue& accuracy)
{
   mLastRefZero = false;
   if ( mAccuracyLimit > accuracy )
   {
      if ( 0 == value )
      {
         mLastRefZero = true;
         if ( mLastSenValid )
         {
            mNorm.AddSensor(mSenChunk / mSenChunkCnt);
         }
      }
   }
   mLastSenValid = false;
   mSenChunk = 0;
   mSenChunkCnt = 0;
}


void PE::CCalibrationBase::AddSensor(const TValue& value)
{
   if ( mLastRefZero )
   {
      mLastSenValid = true;
      mSenChunk += value;
      mSenChunkCnt += 1;
   }
}


const TValue PE::CCalibrationBase::GetBase() const
{
   return mNorm.GetMean();
}


const TValue PE::CCalibrationBase::GetAccuracy() const
{
   return mNorm.GetMld();
}


const TValue PE::CCalibrationBase::GetCalibration() const
{
   return mNorm.GetReliable();
}

