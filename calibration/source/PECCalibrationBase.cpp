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


PE::CCalibration::CCalibration(CNormalisation& normScale, CNormalisation& normBase)
: mNormScale(normScale)
, mNormBase(normBase)
, mRefMin(MAX_VALUE)
, mRefMax(MAX_VALUE)
, mRefLast(MAX_VALUE)
, mInstRefAcc(0.0)
, mInstRefCnt(0)
, mRefAcc(0.0)
, mRefCnt(0)
, mRefDelatAcc(0.0)
, mRefDeltaCnt(0)
, mSenMin(MAX_VALUE)
, mSenMax(MAX_VALUE)
, mSenLast(MAX_VALUE)
, mInstSenAcc(0.0)
, mInstSenCnt(0)
, mSenAcc(0.0)
, mSenCnt(0)
, mSenDeltaAcc(0.0)
, mSenDeltaCnt(0)
{
}


void PE::CCalibration::AddReference(const TValue& value)
{
   mInstRefAcc += value;
   ++mInstRefCnt;
   mRefAcc += value;
   ++mRefCnt;
}


void PE::CCalibration::AddSensor(const TValue& value)
{
   mInstSenAcc += value;
   ++mInstSenCnt;
   mSenAcc += value;
   ++mSenCnt;
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


const TValue PE::CCalibration::GetBase() const
{
   return mNormBase.GetMean();
}


const TValue PE::CCalibration::GetBaseMld() const
{
   return mNormBase.GetMld();
}


const TValue PE::CCalibration::GetBaseReliable() const
{
   return mNormBase.GetReliable();
}


void PE::CCalibration::DoCalibration()
{
   if (0 < mInstRefCnt && 
       0 < mInstSenCnt )
   {
      processValue(mRefLast,mRefMax,mRefMin,mInstRefAcc / mInstRefCnt);
      processValue(mSenLast,mSenMax,mSenMin,mInstSenAcc / mInstSenCnt);
      processScale();
      processBase();
      cleanInst();
   }
}


void PE::CCalibration::processValue(TValue& last, TValue& max, TValue& min, const TValue& value)
{
   if ( MAX_VALUE == last )
   {
      max = value;
      min = value;
   }
   else
   {
      if ( last < value )
      {
         max = value;
         min = last;
      }
      else
      {
         max = last;
         min = value;
      }
   }
   last = value;
}


void PE::CCalibration::processScale()
{
   if ( MAX_VALUE!=mRefLast &&
        MAX_VALUE!=mSenLast )
   {
      TValue refDelta = mRefMax - mRefMin;
      TValue senDelta = mSenMax - mSenMin;

      if ( 0 < refDelta )
      {
         mRefDelatAcc += refDelta;
         ++mRefDeltaCnt;
      }
      if ( 0 < senDelta )
      {
         mSenDeltaAcc += senDelta;
         ++mSenDeltaCnt;
      }
      if ( 0 < refDelta &&
           0 < senDelta )
      {
         TValue scale = ( mRefDelatAcc / mRefDeltaCnt ) / ( mSenDeltaAcc / mSenDeltaCnt );
         mNormScale.AddSensor(scale);
      }
   }
}


void PE::CCalibration::processBase()
{
   if ( 0 != mNormScale.GetMean() &&
        0 != mRefCnt &&
        0 != mSenCnt )
   {
      TValue base = (mSenAcc / mSenCnt) * mNormScale.GetMean() - (mRefAcc / mRefCnt);
      mNormBase.AddSensor(base);
   }
}


void PE::CCalibration::cleanInst()
{
   mInstRefCnt = 0;
   mInstSenCnt = 0;
   mInstRefAcc = 0.0;
   mInstSenAcc = 0.0;
}
