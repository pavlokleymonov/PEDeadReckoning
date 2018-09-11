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


using namespace PE;


PE::CCalibrationBase::CCalibrationBase( CNormalisation* norm, TValue ratio, TValue threshold )
: PE::CCalibrationScale(norm,ratio,threshold)
, mRefAcc(0.0)
, mRefCnt(0.0)
, mRawAcc(0.0)
, mRawCnt(0.0)
{
}


PE::CCalibrationBase::~CCalibrationBase()
{
}


SBasicSensor PE::CCalibrationBase::GetSensor( const SBasicSensor& raw ) const
{
   if ( mpNorm )
   {
      if ( raw.IsValid() && 0.0 != mpNorm->GetMean() )
      {
         return SBasicSensor( raw.Value - mpNorm->GetMean(), raw.Accuracy + mpNorm->GetMld() );
      }
   }
   return SBasicSensor();
}


void PE::CCalibrationBase::DoCalibration()
{
   if ( mpNorm )
   {
      if ( 0.0 != mRefInstCnt && 0.0 != mRawInstCnt )
      {
         if ( !IsOverRatio() )
         {
            mRefAcc += mRefInstAcc / mRefInstCnt;
            mRefCnt += 1.0;

            mRawAcc += mRawInstAcc / mRawInstCnt;
            mRawCnt += 1.0;

            mpNorm->AddSensor( mRawAcc / mRawCnt - mRefAcc / mRefCnt );
         }
         clearInst();
      }
   }
}
