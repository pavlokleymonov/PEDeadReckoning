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


using namespace PE;


PE::CCalibrationScale::CCalibrationScale(CNormalisation& norm)
: mNorm(norm)
, mScale(0)
, mRefMin(MAX_VALUE)
, mRefMax(MAX_VALUE)
, mRefLast(MAX_VALUE)
, mInstRefAcc(0.0)
, mInstRefCnt(0)
, mRefDelatAcc(0.0)
, mRefDeltaCnt(0)
, mSenMin(MAX_VALUE)
, mSenMax(MAX_VALUE)
, mSenLast(MAX_VALUE)
, mInstSenAcc(0.0)
, mInstSenCnt(0)
, mSenDeltaAcc(0.0)
, mSenDeltaCnt(0)
{
}


void PE::CCalibrationScale::AddReference(const TValue& value)
{
   mInstRefAcc += value;
   ++mInstRefCnt;
}


void PE::CCalibrationScale::AddSensor(const TValue& value)
{
   mInstSenAcc += value;
   ++mInstSenCnt;
}


const TValue& PE::CCalibrationScale::GetScale() const
{
   return mScale;
}


void PE::CCalibrationScale::DoCalibration()
{
   TValue deltaRef = 0.0;
   if ( 0 < mInstRefCnt )
   {
      deltaRef = processValue( mRefLast, mRefMax, mRefMin, mInstRefAcc / mInstRefCnt );
   }
   
   if ( 0.0 < deltaRef )
   {
      mRefDelatAcc += deltaRef;
      ++mRefDeltaCnt;
   }

   TValue deltaSen = 0.0;
   if ( 0 < mInstSenCnt )
   {
      deltaSen = processValue( mSenLast, mSenMax, mSenMin, mInstSenAcc / mInstSenCnt );
   }
   
   if ( 0.0 < deltaSen )
   {
      mSenDeltaAcc += deltaSen;
      ++mSenDeltaCnt;
   }

   if ( 0.0 < deltaRef && 0.0 < deltaSen )
   {
      mScale = ( mRefDelatAcc / mRefDeltaCnt ) / ( mSenDeltaAcc / mSenDeltaCnt );
      mNorm.AddSensor(mScale);
      clearInst();
   }
}


void PE::CCalibrationScale::Reset()
{
   mRefMin  = MAX_VALUE;
   mRefMax  = MAX_VALUE;
   mRefLast = MAX_VALUE;
   mSenMin  = MAX_VALUE;
   mSenMax  = MAX_VALUE;
   mSenLast = MAX_VALUE;
   clearInst();
}


TValue PE::CCalibrationScale::processValue(TValue& last, TValue& max, TValue& min, const TValue& value)
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
   return (max - min);
}


void PE::CCalibrationScale::clearInst()
{
   mInstRefCnt = 0;
   mInstSenCnt = 0;
   mInstRefAcc = 0.0;
   mInstSenAcc = 0.0;
}
