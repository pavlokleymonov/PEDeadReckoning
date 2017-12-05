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
#include "PESLocation.h"
#include "PESGnss.h"
#include "PESOdometer.h"
#include "PESGyro.h"
#include "PECCalibration.h"



class PECCoreTest; //to get possibility for test class

namespace PE
{
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
    * Constructor calibartion will create from the scratch
    */
   CCore();
   /**
    * Constructor with already known calibartion information
    */
   explicit CCore(const CCalibration& calibration);
   /**
    * Destructor 
    */
   virtual ~CCore();
   /**
    * Returns current location information.
    *
    * @return       Location information
    */
   virtual SLocation GetLocation() const;
   /**
    * Returns calibration data.
    *
    * @return       calibration information
    */
   virtual SCalibration GetCalibration() const;
   /**
    * Set new GNSS data information.
    *
    * @param timestamp    timestamp in seconds
    * @param gnss         GNSS data information
    */
   virtual void SetGnss(const TTimestamp& timestamp, const SGnss& gnss);
   /**
    * Set new RAW odometer information with driving dirrection
    *
    * @param timestamp    timestamp in seconds
    * @param odo          Odometer data information
    */
   virtual void SetOdometer(const TTimestamp& timestamp, const SOdometer& odo);
   /**
    * Set new RAW gyroscope information
    *
    * @param timestamp    timestamp in seconds
    * @param gyro         Gyroscope data information
    */
   virtual void SetGyro(const TTimestamp& timestamp, const SGyro& gyro);
private:

   CCalibration   m_Calibration;
   CFusionSensor  m_Fusion;
};


} //namespace PE
#endif //__PE_CCore_H__
