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

#include "PECNormalisation.h"
#include <math.h>


using namespace PE;

PE::CNormalisation::CNormalisation()
: mMean(0.0)
, mMld(0.0)
, mReliable(0.0)
, mAccumulatedValue(0.0)
, mAccumulatedMld(0.0)
, mAccumulatedReliable(0.0)
, mSampleCount(0)
{
}

PE::CNormalisation::CNormalisation(const TValue& accumulatedValue, const TValue& accumulatedMld, const TValue& accumulatedReliable, const std::size_t& sampleCount)
: mMean(0.0)
, mMld(0.0)
, mReliable(0.0)
, mAccumulatedValue(accumulatedValue)
, mAccumulatedMld(accumulatedMld)
, mAccumulatedReliable(accumulatedReliable)
, mSampleCount(sampleCount)
{
}

PE::CNormalisation::~CNormalisation()
{
}

void PE::CNormalisation::AddSensor(const TValue& value)
{
   if ( 0    < mSampleCount && 
        0.0 <= mAccumulatedReliable )
   {
      TValue oldMean = mAccumulatedValue / mSampleCount;
      mMean = (mAccumulatedValue + value) / (mSampleCount + 1);

      mAccumulatedMld += fabs( mMean - value );
      mMld = mAccumulatedMld / mSampleCount;

      if ( 0.0 == mMld )
      {
         mAccumulatedReliable += 100;
      }
      else
      {
         TValue deltaMean = fabs( oldMean - mMean );
         //If differences between new mean and previouse mean values less then mld(sigma) then value increase reliability
         if ( deltaMean < mMld )
         {
            mAccumulatedReliable += 100 - deltaMean / mMld * 100;
         }
      }

      mAccumulatedValue += value;
      mSampleCount += 1;
      mReliable = mAccumulatedReliable / mSampleCount;
   }
   else
   {
      mMean = 0.0;
      mMld = 0.0;
      mReliable = 0.0;
      mAccumulatedValue = value;
      mAccumulatedMld = 0.0;
      mAccumulatedReliable = 0.0;
      mSampleCount = 1;
   }
}

const TValue& PE::CNormalisation::GetMean() const
{
   return mMean;
}

const TValue& PE::CNormalisation::GetMld() const
{
   return mMld;
}

const TValue& PE::CNormalisation::GetReliable() const
{
   return mReliable;
}

const TValue& PE::CNormalisation::GetAccumulatedValue() const
{
   return mAccumulatedValue;
}

const TValue& PE::CNormalisation::GetAccumulatedMld() const
{
   return mAccumulatedMld;
}

const TValue& PE::CNormalisation::GetAccumulatedReliable() const
{
   return mAccumulatedReliable;
}

const std::size_t& PE::CNormalisation::GetSampleCount() const
{
   return mSampleCount;
}

