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
#ifndef __PE_CFusionSensor_H__
#define __PE_CFusionSensor_H__

#include "PETypes.h"
#include "PESPosition.h"
#include "PESBasicSensor.h"

namespace PE
{
/**
 * 
 * Preconditions:
 *  - 
 *
 */
class CFusionSensor
{
public:
   /**
    * Constructor 
    *
    * @param position     based position
    * @param heading      heading of based position in degree (0 - Nord, 90 - East, 180 - South, 270 - West)
    */
   CFusionSensor(const SPosition& position, const SBasicSensor& heading);
   /**
    * Destructor 
    */
   virtual ~CFusionSensor();
   /**
    * Adds new position.
    *
    * @param timestamp    timestamp in seconds
    * @param position     position information
    * @param heading      heading of based position in degree (0 - Nord, 90 - East, 180 - South, 270 - West)
    */
   void AddPosition(const TTimestamp& timestamp, const SPosition& position, const SBasicSensor& heading);
   /**
    * Adds new linear velocity
    *
    * @param timestamp    timestamp in seconds
    * @param sensor       linear velocity in meter/seconds
    */
   void AddVelocity(const TTimestamp& timestamp, const SBasicSensor& sensor);
   /**
    * Adds new angularr velocity
    *
    * @param timestamp    timestamp in seconds
    * @param sensor       angular velocity in degree/seconds
    */
   void AddAngularVelocity(const TTimestamp& timestamp, const SBasicSensor& sensor);
   /**
    * Returns timestamp of latest fusioned position.
    *
    * @return         position timestamp in seconds
    */
   const TTimestamp& GetTimestamp() const;
   /**
    * Returns heading of latest fusioned position.
    *
    * @return         heading in degree (0 - Nord, 90 - East, 180 - South, 270 - West)
    */
   const SBasicSensor& GetHeading() const;
   /**
    * Returns latest fusioned position.
    *
    * @return         position.
    */
   const SPosition& GetPosition() const;

private:

   TTimestamp m_Timestamp;

   SPosition m_Position;

   SBasicSensor m_Heading;

   SBasicSensor m_AngSpeed;

   SBasicSensor m_Speed;

   TTimestamp   GetDeltaTime(const TTimestamp& timestamp) const;

   SBasicSensor GetHeading(const TTimestamp& deltaTime) const;

   SPosition    GetPosition(const TTimestamp& deltaTime, const SBasicSensor& heading) const;

};


} //namespace PE
#endif //__PE_CFusionSensor_H__
