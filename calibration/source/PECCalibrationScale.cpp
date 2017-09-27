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

#include "PECCalibrationScale.h"
#include "PETools.h"
#include <math.h>


using namespace PE;


PE::CCalibrationScale::CCalibrationScale(PE::CNormalisation& norm, const TValue& limit)
: mNorm(norm)
, mAccuracyLimit(limit)
, mLastRefValid(false)
, mLastSenValid(false)
, mRefAccumulated(0)
, mSenAccumulated(0)
, mSenChunk(0)
{
}


void PE::CCalibrationScale::AddReference(const TValue& value, const TValue& accuracy)
{
   if ( mAccuracyLimit > accuracy )
   {
      if ( mLastRefValid )
      {
         if ( mLastSenValid )
         {
            CalcScale(value);
         }
      }
      else
      {
         mLastRefValid = true;
      }
   }
   else
   {
      mLastRefValid = false;
      mSenChunk = 0;
   }
}


void PE::CCalibrationScale::AddSensor(const TValue& value)
{
   mLastSenValid = true;
   if ( mLastRefValid )
   {
      mSenChunk += value;
   }
}


const TValue PE::CCalibrationScale::GetScale() const
{
   return mNorm.GetMean();
}


const TValue PE::CCalibrationScale::GetAccuracy() const
{
   return mNorm.GetMld();
}


const TValue PE::CCalibrationScale::GetCalibration() const
{
   return mNorm.GetReliable();
}


void PE::CCalibrationScale::CalcScale(const TValue& value)
{
   mSenAccumulated += mSenChunk;
   mRefAccumulated += value;
   mNorm.AddSensor( mSenAccumulated / mRefAccumulated );
   mSenChunk = 0;
}

