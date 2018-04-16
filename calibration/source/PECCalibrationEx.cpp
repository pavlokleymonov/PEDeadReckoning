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

#include "PECCalibrationEx.h"
#include "PETools.h"
#include <math.h>


using namespace PE;


PE::CCalibrationEx::CCalibrationEx(PE::CNormalisation& normScale, PE::CNormalisation& normBase, const PE::TValue& limit)
: mStarted(false)
, mStartLimit(limit)
, mRefCount(0)
, mRefAccumPositive(0.0)
, mRefAccumNegative(0.0)
, mSenCount(0)
, mSenAccumPositive(0.0)
, mSenAccumNegative(0.0)
, mNormScale(normScale)
, mNormBase(normBase)
{
}


void PE::CCalibrationEx::AddReference(const TValue& value)
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
      mRefCount++;

      if (0 < value)
      {
         mRefAccumPositive += value;
      }
      else
      {
         mRefAccumNegative += value;
      }
      
   }
}


void PE::CCalibrationEx::AddSensor(const TValue& value)
{
   if (mStarted)
   {
      mSenCount++;

      if (0 < value)
      {
         mSenAccumPositive += value;
      }
      else
      {
         mSenAccumNegative += value;
      }
      TValue SenAccumABS = (mSenAccumPositive - mSenAccumNegative) / mSenCount;
      TValue RefAccumABS = (mRefAccumPositive - mRefAccumNegative) / mRefCount;
      if ( 0.0<RefAccumABS && 0.0<SenAccumABS )
      {
         TValue scale        = RefAccumABS / SenAccumABS;
         TValue basePositive = mRefAccumPositive / mRefCount / scale - mSenAccumPositive / mSenCount;
         TValue baseNegative = mRefAccumNegative / mRefCount / scale - mSenAccumNegative / mSenCount;
         TValue base         = (basePositive + baseNegative) / 2;
         mNormScale.AddSensor(scale);
         mNormBase.AddSensor(base);
      }
   }
}


const TValue PE::CCalibrationEx::GetScale() const
{
   return mNormScale.GetMean();
}


const TValue PE::CCalibrationEx::GetBase() const
{
   return mNormBase.GetMean();
}


const TValue PE::CCalibrationEx::GetAccuracy() const
{
   return mNormScale.GetMld() + mNormBase.GetMld(); //??????
}


const std::size PE::CCalibrationEx::GetStatus() const
{
   return round(mNormScale.GetReliable() + mNormBase.GetReliable()) / 2;
}

