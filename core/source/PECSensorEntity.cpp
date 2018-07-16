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
#include "PECSensorEntity.h"


using namespace PE;


const std::string PE::CSensorCfg::CFG_MARKER = "CFGSENSOR";

const std::size_t PE::CSensorCfg::CFG_NUMBER_ELEMENTS = 12;

std::string PE::CSensorCfg::ToSTR(const CSensorCfg& cfg)
{
   std::stringstream st;
   st << CFG_MARKER << "," << int(cfg.mType) << ",";
   st << std::fixed;
   st << std::setprecision(8) << cfg.mScale.mAccValue << "," << cfg.mScale.mAccMld << "," << std::setprecision(1) << cfg.mScale.mAccRel << "," << std::setprecision(0) << cfg.mScale.mCount << ","; //store scale configuration
   st << std::setprecision(8) << cfg.mBase.mAccValue  << "," << cfg.mBase.mAccMld  << "," << std::setprecision(1) << cfg.mBase.mAccRel  << "," << std::setprecision(0) << cfg.mBase.mCount  << ","; //store base configuration
   st << std::setprecision(1) << cfg.mReliableLimit << ",XX" ;
   return st.str();
}


CSensorCfg PE::CSensorCfg::ToCFG(const std::string& str)
{
   std::vector<std::string> list = PE::TOOLS::Split(str, ',');
   if ( CFG_NUMBER_ELEMENTS == list.size() &&
        CFG_MARKER == list[0])
   {
      return CSensorCfg( TSensorTypeID(atoi(list[1].c_str())), //load sensor type
                         CNormCfg(atof(list[2].c_str()), atof(list[3].c_str()), atof(list[4].c_str()), atof(list[5].c_str())), //load scale configuration
                         CNormCfg(atof(list[6].c_str()), atof(list[7].c_str()), atof(list[8].c_str()), atof(list[9].c_str())), //load base configuration
                         atof(list[10].c_str()) ); //load reliable limit
   }
   return CSensorCfg();
}


CNormalisation PE::CSensorCfg::ToNormalisation(const CNormCfg& normCfg)
{
   return CNormalisation(normCfg.mAccValue, normCfg.mAccMld, normCfg.mAccRel, normCfg.mCount );
}

PE::CSensorCfg::CSensorCfg()
: mType(SENSOR_UNKNOWN)
, mScale(1,0,100,1) //Scale = 1
, mBase (0,0,100,1) //Base  = 0
, mReliableLimit(DEFAULT_RELIABLE_LIMIT)
{
}


PE::CSensorCfg::CSensorCfg( TSensorTypeID type, const CNormCfg& scale, const CNormCfg& base, TValue reliableLimit)
: mType(type)
, mScale(scale)
, mBase(base)
, mReliableLimit(reliableLimit)
{
}


const TSensorTypeID& PE::CSensorCfg::GetType() const
{
   return mType;
}


bool PE::CSensorCfg::IsValid() const
{
   return ( SENSOR_UNKNOWN != mType );
}


const CNormCfg& PE::CSensorCfg::GetScale() const
{
   return mScale;
}


const CNormCfg& PE::CSensorCfg::GetBase() const
{
   return mBase;
}


const TValue& PE::CSensorCfg::GetLimit() const
{
   return mReliableLimit;
}
