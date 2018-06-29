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
#ifndef __PE_CNormCfg_H__
#define __PE_CNormCfg_H__

#include "PETypes.h"

class PECNormCfgTest; //to get possibility for test class

namespace PE
{

class CNormCfg
{
public:
   CNormCfg()
      : mAccValue(0.0)
      , mAccMld(0.0)
      , mAccRel(0.0)
      , mCount(0)
      {}

   CNormCfg( const TValue& accumulatedValue, const TValue& accumulatedMld, const TValue& accumulatedReliable, const std::size_t& sampleCount )
      : mAccValue(accumulatedValue)
      , mAccMld(accumulatedMld)
      , mAccRel(accumulatedReliable)
      , mCount(sampleCount)
      {}

   TValue mAccValue;
   TValue mAccMld;
   TValue mAccRel;
   std::size_t mCount;
};

} //namespace PE
#endif //__PE_CNormCfg_H__
