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
#ifndef __PE_CSensor_H__
#define __PE_CSensor_H__

#include "PETypes.h"

namespace PE
{
   /**
    * Basic sensor
    *
    */
   class CSensor
   {
   public:
      /**
       * The sensor value. Unit is related to data meaning. Prefereable in system SI
       */
      TValue Value;
      /**
       * The sensor accuracy in same unit like a value
       */
      TAccuracy Accuracy;
      /**
       * Constructor
       */
      CSensor();
      /**
       * Constructor
       */
      CSensor(const TValue& value, const TAccuracy& accuracy);
      /**
       * Constructor
       */
      virtual ~CSensor();
      /**
       * Is sensor valid. The value and accuracy have to be valid
       */
      virtual bool IsValid() const;
      /**
       * Is sensor valid. The value and accuracy have to be valid
       */
      virtual bool operator==(const PE::CSensor& sensor);
   };

   class CHeading : public CSensor
   {
      public:
         /**
          * Is sensor valid. The value and accuracy have to be valid
          */
         virtual bool IsValid() const;
   };

   class CSpeed : public CSensor
   {
      public:
         /**
          * Is sensor valid. The value and accuracy have to be valid
          */
         virtual bool IsValid() const;
   };

   class CDistance : public CSensor
   {
      public:
         /**
          * Is sensor valid. The value and accuracy have to be valid
          */
         virtual bool IsValid() const;
   };

   class CAngle : public CSensor
   {
      public:
         /**
          * Is sensor valid. The value and accuracy have to be valid
          */
         virtual bool IsValid() const;
   };

} //namespace PE
#endif //__PE_CSensor_H__
