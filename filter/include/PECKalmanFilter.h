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
#ifndef __PE_CKalmanFilter_H__
#define __PE_CKalmanFilter_H__

#include "PETypes.h"
namespace PE
{

/**
 * Kalman Filter
 *
 */
class CKalmanFilter
{
public:
   /**
    * Constructor 
    */
   CKalmanFilter();
   /**
    * Adds new 
    *
    * @param  value    
    * @param  accuracy 
    */
   void AddData(const TValue& value, const TAccuracy& accuracy);
   /**
    * Returns 
    *
    * @return    
    */
   const TValue& GetValue() const;
   /**
    * Returns 
    *
    * @return    
    */
   const TValue& GetAccuracy() const;

private:
   TValue       mValue;
   TAccuracy    mAccuracy;
};

CKalmanFilter::CKalmanFilter()
: mValue(0)
, mAccuracy(0)
{
}


void CKalmanFilter::AddData(const TValue& value, const TAccuracy& accuracy)
{
   if (0 < accuracy)
   {
      if (0 == mAccuracy)
      {
         mValue = value;
         mAccuracy = accuracy;
      }
      else
      {
         TAccuracy K = mAccuracy / (mAccuracy+accuracy);
         mValue = mValue * (1 - K) + value * K;
         mAccuracy = mAccuracy * (1 - K) + accuracy * K;
      }
   }
}

} //namespace PE
#endif //__PE_CKalmanFilter_H__
