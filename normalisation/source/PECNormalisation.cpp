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
, mSampleCount(0.0)
{
}

PE::CNormalisation::CNormalisation(const double& accumulatedValue, const double& accumulatedMld, const double& accumulatedReliable, const double& sampleCount)
: mMean(0.0)
, mMld(0.0)
, mReliable(0.0)
, mAccumulatedValue(accumulatedValue)
, mAccumulatedMld(accumulatedMld)
, mAccumulatedReliable(accumulatedReliable)
, mSampleCount(sampleCount)
{
}

void PE::CNormalisation::AddSensor(const double& value)
{
   if ( 0.0 < mSampleCount && 
        0.0 <= mAccumulatedReliable )
   {
      double oldMean = mAccumulatedValue / mSampleCount;
      mMean = (mAccumulatedValue + value) / (mSampleCount + 1.0);

      mAccumulatedMld += fabs( mMean - value );
      mMld = mAccumulatedMld / mSampleCount;

      if ( 0.0 == mMld )
      {
         mAccumulatedReliable += 100;
      }
      else
      {
         double deltaMean = fabs( oldMean - mMean );
         //If differences between new mean and previouse mean values less then mld(sigma) then value increase reliability
         if ( deltaMean < mMld )
         {
            mAccumulatedReliable += 100 - deltaMean / mMld * 100;
         }
      }

      mAccumulatedValue += value;
      mSampleCount += 1.0;
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
      mSampleCount = 1.0;
   }
}

const double& PE::CNormalisation::GetMean() const
{
   return mMean;
}

const double& PE::CNormalisation::GetMld() const
{
   return mMld;
}

const double& PE::CNormalisation::GetReliable() const
{
   return mReliable;
}

const double& PE::CNormalisation::GetAccumulatedValue() const
{
   return mAccumulatedValue;
}

const double& PE::CNormalisation::GetAccumulatedMld() const
{
   return mAccumulatedMld;
}

const double& PE::CNormalisation::GetAccumulatedReliable() const
{
   return mAccumulatedReliable;
}

const double& PE::CNormalisation::GetSampleCount() const
{
   return mSampleCount;
}

