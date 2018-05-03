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
, mSenMin(MAX_VALUE)
, mSenMax(MAX_VALUE)
, mSenLast(MAX_VALUE)
{
}


void PE::CCalibration::AddReference(const TValue& value)
{
   if ( MAX_VALUE == mRefLast )
   {
      mRefMax = value;
      mRefMin = value;
      mRefLast = value;
   }
   else
   {
      if ( mRefLast < value )
      {
         mRefMax = value;
         mRefMin = mRefLast;
      }
      else
      {
         mRefMax = mRefLast;
         mRefMin = value;
      }
      mRefLast = value;
      //processScale();
      //processBase();
   }
}


void PE::CCalibration::AddSensor(const TValue& value)
{
   if ( MAX_VALUE == mSenLast )
   {
      mSenMax = value;
      mSenMin = value;
      mSenLast = value;
   }
   else
   {
      if ( mSenLast < value )
      {
         mSenMax = value;
         mSenMin = mSenLast;
      }
      else
      {
         mSenMax = mSenLast;
         mSenMin = value;
      }
      mSenLast = value;
      processScale();
      processBase();
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


void PE::CCalibration::processScale()
{
   if ( MAX_VALUE!=mRefLast &&
        MAX_VALUE!=mSenLast )
   {
      TValue refDelta = mRefMax - mRefMin;
      TValue senDelta = mSenMax - mSenMin;
      if ( 0 < refDelta && 
           0 < senDelta )
      {
         TValue scale = refDelta / senDelta;
         mNormScale.AddSensor(scale);
      }
   }
}


void PE::CCalibration::processBase()
{
   if ( 0 != mNormScale.GetMean() )
   {
      if ( MAX_VALUE != mRefLast &&
           MAX_VALUE != mSenLast)
      {
         TValue base = mSenLast - mRefLast / mNormScale.GetMean();
         //printf("base=%0.9f\n", base);
         mNormBase.AddSensor(base);
      }
   }
}
