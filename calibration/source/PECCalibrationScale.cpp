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


PE::CCalibrationScale::CCalibrationScale( CNormalisation* norm, TValue ratio, TValue threshold )
: CCalibration(norm, ratio, threshold)
, mRefMin(MAX_VALUE)
, mRefMax(MAX_VALUE)
, mRefLast(MAX_VALUE)
, mRefDeltaAcc(0.0)
, mRefDeltaCnt(0.0)
, mRawMin(MAX_VALUE)
, mRawMax(MAX_VALUE)
, mRawLast(MAX_VALUE)
, mRawDeltaAcc(0.0)
, mRawDeltaCnt(0.0)
{
}


PE::CCalibrationScale::~CCalibrationScale()
{
}


SBasicSensor PE::CCalibrationScale::GetSensor( const SBasicSensor& raw ) const
{
   if ( mpNorm )
   {
      if ( raw.IsValid() && 0.0 != mpNorm->GetMean() )
      {
         return SBasicSensor( raw.Value * mpNorm->GetMean(), mpNorm->GetMean() + mpNorm->GetMld() );
      }
   }
   return SBasicSensor();
}


void PE::CCalibrationScale::DoCalibration()
{
   if ( mpNorm )
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
                  mpNorm->AddSensor( ( mRefDeltaAcc / mRefDeltaCnt ) / ( mRawDeltaAcc / mRawDeltaCnt ) );
               }
            }
         }
         clearInst();
      }
   }
}
