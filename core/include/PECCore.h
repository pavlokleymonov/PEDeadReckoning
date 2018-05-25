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
#ifndef __PE_CCore_H__
#define __PE_CCore_H__

#include "PETypes.h"
#include "PECNormalisation.h"
#include "PECCalibrationScale.h"
#include "PECCalibrationBase.h"
#include "PECFusionSensor.h"



class PECCoreTest; //to get possibility for test class

namespace PE
{

struct CSensorEntity
{
   TSensorTypeID type;
   CNormalisation normScale;
   CNormalisation normBase;
   CCalibrationScale calScale;
   CCalibrationBase  calBase;
};
/**
 * 
 * Preconditions:
 *  - 
 *
 */
class CCore
{

friend class ::PECCoreTest;

public:
   /**
    * Constructor
    */
   CCore();
   /**
    * Destructor 
    */
   virtual ~CCore();

   virtual bool SetSensorCfg(TSensorTypeID type, TSensorID id, const CNormalisation& norm);

   virtual const CNormalisation& GetSensorCfg(TSensorTypeID type, TSensorID id);

   virtual bool AddSensorValue(TSensorTypeID type, TSensorID id, TTimestamp timestamp, TValue value);

   virtual TTimestamp   GetTimestamp();

   virtual SPosition    GetPosition(TTimestamp timestap);

   virtual SBasicSensor GetHeading(TTimestamp timestap);

   virtual SBasicSensor GetSpeed(TTimestamp timestap);

private:
   std::map<TSensorID,>
};


} //namespace PE
#endif //__PE_CCore_H__
