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
    */
   CFusionSensor(const TTimestamp& timestamp, const SPosition& position, const SBasicSensor& heading);
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
   void AddPosition(const TTimestamp& timestamp, const SPosition& position, const SBasicSensor& heading, const SBasicSensor& speed);
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

   SBasicSensor PredictSensorAccuracy(const TTimestamp& timestampFirst, const SBasicSensor& sensor, const TTimestamp& timestampLast);
   SBasicSensor PredictHeading(const TTimestamp& timestampFirst, const SBasicSensor& heading, const TTimestamp& timestampLast, const SBasicSensor& angSpeed);
   SBasicSensor PredictHeading(const SPosition& positionFirst, const SPosition& positionLast);
   SPosition    PredictPosition(const TTimestamp& timestampFirst, const SPosition& position, const SBasicSensor& heading, const TTimestamp& timestampLast, const SBasicSensor& speed);
   SPosition    PredictPosition(const TTimestamp& timestampFirst, const SBasicSensor& headingFirst, const TTimestamp& timestampLast, const SBasicSensor& headingLast, const SPosition& position, const SBasicSensor& speed);
   SBasicSensor PredictSpeed(const TTimestamp& timestampFirst, const SPosition& positionFirst, const TTimestamp& timestampLast, const SPosition& positionLast);
   SBasicSensor PredictAngSpeed(const TTimestamp& timestampFirst, const SBasicSensor& headingFirst, const TTimestamp& timestampLast, const SBasicSensor& headingLast);

   SBasicSensor MergeSensor(const SBasicSensor& sen1, const SBasicSensor& sen2);
   SBasicSensor MergeHeading(const SBasicSensor& head1, const SBasicSensor& head2);
   SPosition    MergePosition(const SPosition& pos1, const SPosition& pos2);
private:

   TTimestamp m_Timestamp;

   SPosition m_Position;

   SBasicSensor m_Heading;

   SBasicSensor m_AngSpeed;

   SBasicSensor m_Speed;

};


} //namespace PE
#endif //__PE_CFusionSensor_H__
