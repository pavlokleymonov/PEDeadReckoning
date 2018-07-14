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

#include "PECNormCfg.h"
#include "PECSensorCfg.h"
#include "PECSensorEntity.h"



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
   CCoreSimple( const SPosition& position, const SBasicSensor& heading)
   : mFusion(0, position, heading, SBasicSensor(), SBasicSensor())
   {
   }
   /**
    *
    */
   bool SetOdoCfg(const CSensorCfg& cfg)
   {
      if ( !mOdo.GetCfg().IsValid() )
      {
         mOdo = CSensorEntity(cfg);
         return mOdo.GetCfg().IsValid();
      }
      return false;
   }
   /**
    *
    */
   const CSensorCfg& GetOdoCfg() const
   {
      return mOdo.GetCfg();
   }
   /**
    *
    */
   bool SetGyroCfg(const CSensorCfg& cfg)
   {
      if ( !mGyro.GetCfg().IsValid() )
      {
         mGyro = CSensorEntity(cfg);
         return mGyro.GetCfg().IsValid();
      }
      return false;
   }
   /**
    *
    */
   const CSensorCfg& GetGyroCfg() const
   {
      return mGyro.GetCfg();
   }
   /**
    *
    * Returns true if new position has been calculated
    */
   bool AddGnss(TTimestamp ts, const SPosition& pos, const SBasicSensor& head, const SBasicSensor& speed)
   {
      mFusion.AddPosition(ts, pos);
      mFusion.AddHeading(ts, head);
      mFusion.AddSpeed(ts, speed);
      mFusion.DoFusion();
      return true;
   }
   /**
    *
    * Returns true if new position has been calculated
    */
   bool AddOdo(TTimestamp ts, const SBasicSensor& odo)
      {
         if ( mOdo.GetCfg().IsValid() )
         {
            if ( odo.IsValid() )
            {
               if ( mOdo.AddRaw(odo.Value) )
               {
                  mFusion.AddSpeed(ts, mOdo.GetSpeed())
               }
            }
            else
            {
               mOdo.Reset();
            }
         }
         return false;
      }
   /**
    *
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
   /**
    *
    */
   CSensorEntity mOdo;
   /**
    *
    */
   CSensorEntity mGyro;
   /**
    * Position fusion stuff
    */
   CFusionSensor mFusion;
};


} //namespace PE
#endif //__PE_CCoreSimple_H__
