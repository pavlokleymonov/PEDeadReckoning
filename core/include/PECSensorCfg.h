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

class PECSensorCfgTest; //to get possibility for test class

namespace PE
{

class CSensorCfg
{
public:
   static const std::string CFG_MARKER = "CFGSENSOR";

   static const std::size_t CFG_NUMBER_ELEMENTS = 11;

   static std::string ToSTR(const CSensorCfg& cfg)
      {
         std::stringstream st;
         st << CFG_MARKER << "," << int(mType) << ",";
         st << int(mScale.mAccValue) << "," << int(mScale.mAccMld) << "," << int(mScale.mAccRel) << "," << int(mScale.mCount) << ","; //store scale configuration
         st << int(mBase.mAccValue) << "," << int(mBase.mAccMld) << "," << int(mBase.mAccRel) << "," << int(mBase.mCount) << ","; //store base configuration
         st << std::setprecision(1) << double(mReliableLimit);
         return st.str();
      }

   static CSensorCfg ToCFG(const std::string& str)
      {
         std::vector<std::string> list = PE::TOOLS::Split();
         if ( CFG_NUMBER_ELEMENTS == list.size() )
         {
            if ( CFG_MARKER == list[0] )
            {
               return CSensorCfg(
                  atoi(list[1]), //load sensor type
                  CNormCfg(atoi(list[2]), atoi(list[3]), atoi(list[4]), atoi(list[5])), //load scale configuration
                  CNormCfg(atoi(list[6]), atoi(list[7]), atoi(list[8]), atoi(list[9])), //load base configuration
                  atof(list[10]) //load reliable limit
            }
         }
         return CSensorCfg();
      }

   CSensorCfg()
      : mType(SENSOR_UNKNOWN)
      , mScale(1,0,100,1) //Scale = 1
      , mBase (0,0,100,1) //Base  = 0
      , mReliableLimit(DEFAULT_RELIABLE_LIMIT)
      {}

   CSensorCfg( TSensorTypeID type, const CNormCfg& scale, const CNormCfg& base, TValue reliableLimit = DEFAULT_RELIABLE_LIMIT)
      : mType(type)
      , mScale(scale)
      , mBase(base)
      , mReliableLimit(reliableLimit)
      {}

   const TSensorTypeID& GetType() const
      {
         return mType;
      }

   inline bool IsValid() const
      {
         return ( SENSOR_UNKNOWN != mType );
      }

   const CNormCfg& GetScale() const
      {
         return mScale;
      }

   const CNormCfg& GetBase() const
      {
         return mBase;
      }

   const TValue& GetLimit() const
      {
         return mReliableLimit;
      }

private:
   TSensorTypeID mType;
   CNormCfg mScale;
   CNormCfg mBase;
   TValue mReliableLimit;
};

} //namespace PE
#endif //__PE_CSensorCfg_H__
