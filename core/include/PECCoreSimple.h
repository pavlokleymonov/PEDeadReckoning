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

#include "PECSensorCfg.h"
#include "PECFusionSensor.h"
#include "PECCalibrationScale.h"
#include "PECCalibrationBase.h"



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
    */
   CCoreSimple( const SPosition& position, const SBasicSensor& heading);
   /**
<<<<<<< HEAD
    * Sets configuration for Odometer sensor 100ms interval
    */
   bool SetOdoCfg(const CSensorCfg& cfg);
   /**
    * Gets configuration for Odometer sensor
    */
   bool GetOdoCfg(CSensorCfg& cfg) const;
   /**
    * Sets configuration for Gyroscope sensor Z-axe 100ms interval
    */
   bool SetGyroCfg(const CSensorCfg& cfg);
   /**
    * Gets configuration for Gyroscope sensor Z-axe
    */
   bool GetGyroCfg(CSensorCfg& cfg) const;
=======
    * Set configuration for Odometer sensor
    */
   bool SetOdoCfg(const CSensorCfg& cfg);
   /**
    * Returns Odometer sensor configuration
    */
   const CSensorCfg& GetOdoCfg() const;
   /**
    * Set configuration for Gyroscope sensor Z-axe
    */
   bool SetGyroCfg(const CSensorCfg& cfg);
   /**
    * Returns Gyroscope sensor configuration
    */
   const CSensorCfg& GetGyroCfg() const;
>>>>>>> 6122ae5278dcd2e88f474ae363dfc06d42d03d30
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

   std::map<TSensorTypeID, CSensorEntity> mSensors;

   /**
    * Position fusion stuff
    */
   CFusionSensor mFusion;
};


} //namespace PE
#endif //__PE_CCoreSimple_H__
