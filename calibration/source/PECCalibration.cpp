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


using namespace PE;


PE::CCalibration::CCalibration( CNormalisation* norm, TValue ratio, TValue threshold )
: mpNorm(norm)
, mRatio(ratio)
, mThreshold(threshold)
, mRefInstAcc(0.0)
, mRefInstCnt(0.0)
, mRawInstAcc(0.0)
, mRawInstCnt(0.0)
{
}


PE::CCalibration::~CCalibration()
{
}


void PE::CCalibration::AddReference(const SBasicSensor& ref)
{
   if ( ref.IsValid() )
   {
      mRefInstAcc += ref.Value;
      mRefInstCnt += 1.0;
      DoCalibration();
   }
}


void PE::CCalibration::AddSensor( const SBasicSensor& raw )
{
   if ( raw.IsValid() )
   {
      mRawInstAcc += raw.Value;
      mRawInstCnt += 1.0;
      DoCalibration();
   }
}


TValue PE::CCalibration::processValue(TValue& last, TValue& max, TValue& min, const TValue& value)
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


void PE::CCalibration::clearInst()
{
   mRefInstAcc = 0.0;
   mRefInstCnt = 0.0;

   mRawInstAcc = 0.0;
   mRawInstCnt = 0.0;
}


bool PE::CCalibration::IsOverRatio()
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
