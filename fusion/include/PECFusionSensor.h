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

class PECFusionSensorTest; //to get possibility for test class

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

friend class ::PECFusionSensorTest;

public:
   /**
    * Constructor 
    *
    * @param timestamp    timestamp in seconds. HINT: for first initialisation please provide stored position and latest timestamp
    * @param position     based position
    * @param heading      heading of based position in degree (0 - Nord, 90 - East, 180 - South, 270 - West)
    * @param angSpeed     angular velocity in degree/second turning left("+") - positive, turning right("-") - negative
    * @param speed        linear velocity in meter/seconds
    */
   CFusionSensor(const TTimestamp& timestamp, const SPosition& position, const SBasicSensor& heading, const SBasicSensor& angSpeed, const SBasicSensor& speed);
   /**
    * Destructor 
    */
   virtual ~CFusionSensor();
   /**
    * Adds new position.
    *
    * @param timestamp    timestamp in seconds
    * @param position     position information
    */
   void AddPosition(const TTimestamp& timestamp, const SPosition& position);
   /**
    * Adds new heading.
    *
    * @param timestamp    timestamp in seconds
    * @param heading      heading of based position in degree (0 - Nord, 90 - East, 180 - South, 270 - West)
    */
   void AddHeading(const TTimestamp& timestamp, const SBasicSensor& heading);
   /**
    * Adds new linear velocity
    *
    * @param timestamp    timestamp in seconds
    * @param speed        linear velocity in meter/seconds
    */
   void AddSpeed(const TTimestamp& timestamp, const SBasicSensor& speed);
   /**
    * Adds new angularr velocity
    *
    * @param timestamp    timestamp in seconds
    * @param angSpeed     angular velocity in degree/second turning left("+") - positive, turning right("-") - negative
    */
   void AddAngSpeed(const TTimestamp& timestamp, const SBasicSensor& angSpeed);
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
   /**
    * Returns latest fusioned speed.
    *
    * @return         speed.
    */
   const SBasicSensor& GetSpeed() const;
   /**
    * Returns latest fusioned speed.
    *
    * @param timestamp    timestamp of predicted speed in seconds
    * @return             predicted speed in m/s.
    */
   const SBasicSensor GetSpeed(const TTimestamp& timestamp) const;
   /**
    * Returns latest fusioned angular velocity.
    *
    * @return         angSpeed.
    */
   const SBasicSensor& GetAngSpeed() const;
   /**
    * Returns latest fusioned angular velocity.
    *
    * @param timestamp    timestamp of predicted speed in seconds
    * @return             predicted angSpeed in degree/second turning left("+") - positive, turning right("-") - negative.
    */
   const SBasicSensor GetAngSpeed(const TTimestamp& timestamp) const;
   /**
    * Fuses all available sensors into current position
    */
   void DoFusion();

private:
   /**
    * Structure hold allbasic sensors information with the same timestamp
    */
   struct SSensorItem
   {
      SSensorItem(const TTimestamp& ts, const SPosition& pos, const SBasicSensor& head, const SBasicSensor& sp, const SBasicSensor& asp)
         : timestamp(ts)
         , position (pos)
         , heading  (head)
         , speed    (sp)
         , angSpeed (asp)
      {}

      TTimestamp timestamp;
      SPosition position;
      SBasicSensor heading;
      SBasicSensor speed;
      SBasicSensor angSpeed;
   };

   typedef std::vector<SSensorItem> TSensorsList;

   /**
    * The timestamp of the latest position in seconds
    */
   TTimestamp m_Timestamp;
   /**
    * The latest position
    */
   SPosition m_Position;
   /**
    * The heading of position in degree (0 - Nord, 90 - East, 180 - South, 270 - West)
    */
   SBasicSensor m_Heading;
   /**
    * The angular velocity in degree/second turning left("+") - positive, turning right("-") - negative
    */
   SBasicSensor m_AngSpeed;
   /**
    * TODO: has to be removed
    */
   TValue m_AngAcceleration;
   /**
    * The linear velocity in meter/seconds
    */
   SBasicSensor m_Speed;
   /**
    * TODO: has to be removed
    */
   TValue m_LineAcceleration;

   TSensorsList m_SensorsList;
   /**
    * Fused position based on one sensor item information
    */
   void DoOneItemFusion(const TTimestamp& timestamp, const SPosition& position, const SBasicSensor& heading, const SBasicSensor& speed, const SBasicSensor& angSpeed);
};


} //namespace PE
#endif //__PE_CFusionSensor_H__
