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
#ifndef __PE_CSensorEntity_H__
#define __PE_CSensorEntity_H__

#include "PETypes.h"


class PECSensorEntityTest; //to get possibility for test class

namespace PE
{

class CSensorEntity
{
public:
   CSensorEntity( const CSensorCfg& cfg )
      : mType(cfg.GetType())
      : mLimit(cfg.GetLimit())
      , mNormScale(CNormalisation(cfg.GetScale().mAccValue, cfg.GetScale().mAccMld, cfg.GetScale().mAccRel, cfg.GetScale().mCount))
      , mNormBase (CNormalisation(cfg.GetBase().mAccValue,  cfg.GetBase().mAccMld,  cfg.GetBase().mAccRel,  cfg.GetBase().mCount ))
      , mCalScale (mNormScale)
      , mCalBase  (mNormBase)
      {}

   CSensorCfg GetCfg() const
      {
         return CSensorCfg(
            mType,
            CNormCfg(mNormScale.GetAccumulatedValue(), mNormScale.GetAccumulatedMld(), mNormScale.GetAccumulatedReliable(), mNormScale.GetSampleCount()),
            CNormCfg(mNormBase.GetAccumulatedValue(), mNormBase.GetAccumulatedMld(), mNormBase.GetAccumulatedReliable(), mNormBase.GetSampleCount()),
            mLimit
         );
      }

   const TSensorTypeID& GetType() const
      {
         return mType;
      }

   TValue CalibartedTo() const
      {
         return (mNormScale.GetReliable() + mNormBase.GetReliable()) / 2;
      }

   const TValue& GetScale() const
      {
         return mCalScale.GetScale();
      }

   const TValue& GetBase() const
      {
         return mCalBase.GetBase();
      }


   SBasicSensor Calculate(const TValue& raw, const SBasicSensor& ref)
      {
         mCalScale.AddSensor(raw);
         mCalScale.AddReference(ref.Value);
         mCalScale.DoCalibration();

         if ( IsScaleReliable() )
         {
            mCalBase.AddScale(mCalScale.GetScale());
            mCalBase.AddSensor(raw);
            mCalBase.AddReference(ref.Value);
            mCalBase.DoCalibration();

            if ( IsBaseReliable() )
            {
               SBasicSensor sen(GetValue(raw), GetAccuracy(mNormBase.GetMld()));

               TValue deltaAccuracy = fabs(ref.Value - sen.Value);

               if ( deltaAccuracy > ref.Accuracy )
               {
                  sen.Accuracy = deltaAccuracy;
               }
               return sen;
            }
         }
         return SBasicSensor(); //invalid sensor
      }

private:
   const TSensorTypeID mType;
   const TValue mLimit;
   CNormalisation mNormScale;
   CNormalisation mNormBase;
   CCalibrationScale mCalScale;
   CCalibrationBase  mCalBase;

   inline bool IsScaleReliable() const
      {
         return mLimit <= mNormScale.GetReliable();
      }

   inline bool IsBaseReliable() const
      {
         return mLimit <= mNormBase.GetReliable();
      }

   inline TValue GetValue(const TValue& raw) const
      {
         return TValue(raw * mCalScale.GetScale() - mCalBase.GetBase());
      }

   inline TAccuracy GetAccuracy(const TValue& mld) const
      {
         return TAccuracy(mld * 3);
      }
};

} //namespace PE
#endif //__PE_CSensorEntity_H__
