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
#ifndef __PE_CCoreSimple_H__
#define __PE_CCoreSimple_H__

#include <map>
#include "PETypes.h"
#include "PECSensorCfg.h"
#include "PECSensorEntity.h"
#include "PECFusionSensor.h"



class PECCoreSimpleTest; //to get possibility for test class

namespace PE
{

/**
 * 
 * Preconditions:
 *  - 
 *
 */
class CCoreSimple
{

friend class ::PECCoreSimpleTest;

public:

   /**
    * Constructor with start position and heading
    *
    * @param position       started position
    * @param heading        started heading
    * @param interval       interval for position generation in [s]
    */
   CCoreSimple( const SPosition& position, const SBasicSensor& heading, TTimestamp interval);
   /**
    * Sets sensor configuration
    */
   void SetOdoCfg(const CSensorCfg& cfg, TValue ratio, TValue threshold);
   /**
    * Gets sensor configuration
    */
   CSensorCfg GetOdoCfg() const;
   /**
    * Sets sensor configuration
    */
   void SetGyroCfg(const CSensorCfg& cfg, TValue ratio, TValue threshold);
   /**
    * Gets sensor configuration
    */
   CSensorCfg GetGyroCfg() const;
   /**
    * Returns true if new position has been calculated
    */
   bool AddGnss(TTimestamp ts, const SPosition& pos, const SBasicSensor& head, const SBasicSensor& speed);
   /**
    * Returns true if new position has been calculated
    */
   bool AddOdo(TTimestamp ts, const SBasicSensor& odo);
   /**
    * Returns true if new position has been calculated
    */
   bool AddGyro(TTimestamp ts, const SBasicSensor& gyro);
   /**
    * Returns timestamp of fusion position
    */
   const TTimestamp& GetTimestamp() const;
   /**
    * Returns Position of fusion position
    */
   const SPosition& GetPosition() const;
   /**
    * Returns Heading of fusion position
    */
   const SBasicSensor& GetHeading() const;
   /**
    * Returns Speed of fusion position
    */
   const SBasicSensor& GetSpeed() const;

private:

   TTimestamp mInterval;

   std::map<TSensorTypeID, CSensorEntity> mEntities;
   /**
    * Position fusion stuff
    */
   CFusionSensor mFusion;
   /**
    * Sets sensor configuration
    */
   void SetCfg(const CSensorCfg& cfg, TValue ratio, TValue threshold);
   /**
    * Gets sensor configuration
    */
   CSensorCfg GetCfg(TSensorTypeID typeId) const;

   void AddRef(TSensorTypeID typeId, const SBasicSensor& ref);

   SBasicSensor CalculateSensor(TSensorTypeID typeId, const SBasicSensor& raw);

   bool UpdatePosition(TTimestamp ts);

};


} //namespace PE
#endif //__PE_CCoreSimple_H__
