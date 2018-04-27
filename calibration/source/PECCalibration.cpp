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

#include "PECCalibration.h"
#include "PETools.h"
#include <math.h>


using namespace PE;


PE::CCalibration::CCalibration(PE::CNormalisation& normScale, PE::CNormalisation& normBasePositive, PE::CNormalisation& normBaseNegative, const PE::TValue& limit)
: mStarted(false)
, mStartLimit(limit)
, mRefCountPositive(0)
, mRefAccumPositive(0.0)
, mRefCountNegative(0)
, mRefAccumNegative(0.0)
, mSenCountPositive(0)
, mSenAccumPositive(0.0)
, mSenCountNegative(0)
, mSenAccumNegative(0.0)
, mNormScale(normScale)
, mNormBasePositive(normBasePositive)
, mNormBaseNegative(normBaseNegative)
, mpNormScale(0)
, mpNormBasePositive(0)
, mpNormBaseNegative(0)
{
}


void PE::CCalibration::AddReference(const TValue& value)
{
   if (false == mStarted)
   {
      if (fabs(mStartLimit) < fabs(value) )
      {
         mStarted = true;
      }
   }
   if (mStarted)
   {
      if (0.0 < value)
      {
         ++mRefCountPositive;
         mRefAccumPositive += value;
      }
      else
      {
         ++mRefCountNegative;
         mRefAccumNegative += value;
      }
   }
}


void PE::CCalibration::AddSensor(const TValue& value)
{
   if (mStarted)
   {
      if (0.0 < value)
      {
         ++mSenCountPositive;
         mSenAccumPositive += value;
      }
      else
      {
         ++mSenCountNegative;
         mSenAccumNegative += value;
      }
      TValue SenAccumABS = (mSenAccumPositive - mSenAccumNegative) / (mSenCountPositive + mSenCountNegative);
      TValue RefAccumABS = (mRefAccumPositive - mRefAccumNegative) / (mRefCountPositive + mRefCountNegative);
      if ( 0.0 < RefAccumABS && 0.0 < SenAccumABS )
      {
         TValue scale = RefAccumABS / SenAccumABS;
         mNormScale.AddSensor(scale);
         if ( 0.0 < mSenAccumPositive )
         {
            TValue basePositive = mRefAccumPositive / mRefCountPositive / scale - mSenAccumPositive / mSenCountPositive;
            mNormBasePositive.AddSensor(basePositive);
         }
         if ( 0.0 > mSenAccumNegative )
         {
            TValue baseNegative = mRefAccumNegative / mRefCountNegative / scale - mSenAccumNegative / mSenCountNegative;
            mNormBaseNegative.AddSensor(baseNegative);
         }
      }
   }
}


const TValue PE::CCalibration::GetScale() const
{
   return mNormScale.GetMean();
}


const TValue PE::CCalibration::GetScaleMld() const
{
   return mNormScale.GetMld();
}


const TValue PE::CCalibration::GetScaleReliable() const
{
   return mNormScale.GetReliable();
}


const TValue PE::CCalibration::GetBasePositive() const
{
   return mNormBasePositive.GetMean();
}


const TValue PE::CCalibration::GetBasePositiveMld() const
{
   return mNormBasePositive.GetMld();
}


const TValue PE::CCalibration::GetBasePositiveReliable() const
{
   return mNormBasePositive.GetReliable();
}


const TValue PE::CCalibration::GetBaseNegative() const
{
   return mNormBaseNegative.GetMean();
}


const TValue PE::CCalibration::GetBaseNegativeMld() const
{
   return mNormBaseNegative.GetMld();
}


const TValue PE::CCalibration::GetBaseNegativeReliable() const
{
   return mNormBaseNegative.GetReliable();
}
