/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2018 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */
#ifndef __PE_CSensorCfg_H__
#define __PE_CSensorCfg_H__

#include "PETypes.h"
#include "PECNormCfg.h"
#include "PECNormalisation.h"

class PECSensorCfgTest; //to get possibility for test class

namespace PE
{

class CSensorCfg
{
public:

   static const std::string CFG_MARKER;

   static const std::size_t CFG_NUMBER_ELEMENTS;

   static std::string ToSTR(const CSensorCfg& cfg);

   static CSensorCfg ToCFG(const std::string& str);

   static CNormalisation ToNormalisation(const CNormCfg& normCfg);

   CSensorCfg();

   CSensorCfg( TSensorTypeID type, const CNormCfg& scale, const CNormCfg& base, TValue reliableLimit = DEFAULT_RELIABLE_LIMIT);

   const TSensorTypeID& GetType() const;

   bool IsValid() const;

   const CNormCfg& GetScale() const;

   const CNormCfg& GetBase() const;

   const TValue& GetLimit() const;

private:
   TSensorTypeID mType;
   CNormCfg mScale;
   CNormCfg mBase;
   TValue mReliableLimit;
};

} //namespace PE
#endif //__PE_CSensorCfg_H__
