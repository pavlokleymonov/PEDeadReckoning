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


PE::CCalibrationScale::CCalibrationScale( CNormalisation& norm, TValue ratio, TValue threshold )
: mNorm(norm)
, mRatio(ratio)
, mThreshold(threshold)
, mRefMin(MAX_VALUE)
, mRefMax(MAX_VALUE)
, mRefLast(MAX_VALUE)
, mRefInstAcc(0.0)
, mRefInstCnt(0.0)
, mRefDeltaAcc(0.0)
, mRefDeltaCnt(0.0)
, mRawMin(MAX_VALUE)
, mRawMax(MAX_VALUE)
, mRawLast(MAX_VALUE)
, mRawInstAcc(0.0)
, mRawInstCnt(0.0)
, mRawDeltaAcc(0.0)
, mRawDeltaCnt(0.0)
{
}


void PE::CCalibrationScale::AddReference(const SBasicSensor& ref)
{
   if ( ref.IsValid() )
   {
      mRefInstAcc += ref.Value;
      mRefInstCnt += 1.0;
      DoCalibration();
   }
}


void PE::CCalibrationScale::AddSensor( const SBasicSensor& raw )
{
   if ( raw.IsValid() )
   {
      mRawInstAcc += raw.Value;
      mRawInstCnt += 1.0;
      DoCalibration();
   }
}


SBasicSensor PE::CCalibrationScale::GetSensor( const SBasicSensor& raw ) const
{
   if ( raw.IsValid() && 0.0 != mNorm.GetMean() )
   {
      return SBasicSensor( raw.Value * mNorm.GetMean(), mNorm.GetMean() + mNorm.GetMld() );
   }
   else
   {
      return SBasicSensor();
   }
}


const TValue& PE::CCalibrationScale::CalibratedTo() const
{
   return mNorm.GetReliable();
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
   mRefInstAcc = 0.0;
   mRefInstCnt = 0.0;

   mRawInstAcc = 0.0;
   mRawInstCnt = 0.0;
}


bool PE::CCalibrationScale::IsOverRatio()
{
   if      ( mRatio + mThreshold < mRawInstCnt / mRefInstCnt )
   {
      return true;
   }
   else if ( mRatio - mThreshold > mRawInstCnt / mRefInstCnt )
   {
      return true;
   }
   else
   {
      return false;
   }
}


void PE::CCalibrationScale::DoCalibration()
{
   if ( 0.0 != mRefInstCnt && 0.0 != mRawInstCnt )
   {
      if ( !IsOverRatio() )
      {
         TValue deltaRef = processValue( mRefLast, mRefMax, mRefMin, mRefInstAcc / mRefInstCnt );
         if ( 0.0 != deltaRef )
         {
            mRefDeltaAcc += deltaRef;
            mRefDeltaCnt += 1.0;
         }
         
         TValue deltaRaw = processValue( mRawLast, mRawMax, mRawMin, mRawInstAcc / mRawInstCnt );
         if ( 0.0 != deltaRaw )
         {
            mRawDeltaAcc += deltaRaw;
            mRawDeltaCnt += 1.0;
         }
         
         if ( 0.0 != deltaRef || 0.0 != deltaRaw )
         {
            if ( 0.0 != mRefDeltaCnt && 0.0 != mRawDeltaCnt )
            {
               mNorm.AddSensor( ( mRefDeltaAcc / mRefDeltaCnt ) / ( mRawDeltaAcc / mRawDeltaCnt ) );
            }
         }
      }
      clearInst();
   }
}

