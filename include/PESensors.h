#ifndef __PE_Sensor_H__
#define __PE_Sensor_H__

#include "PETypes.h"

namespace PE
{

//Position is valid only if at least one of 
class Position
{
public:

   Position(
      const TTimestamp& timestamp,
      const TValue&     utc,
      const TValue&     latitude,
      const TAccuracy&  latitude_accuracy,
      const TValue&     longitude,
      const TAccuracy&  longitude_accuracy,
      const TValue&     altitude,
      const TAccuracy&  altitude_accuracy,
      const TValue&     heading,
      const TAccuracy&  heading_accuracy,
   )
      : m_ts(timestamp)
      , m_utc(utc)
      , m_lat(latitude)
      , m_lat_acc(latitude_accuracy)
      , m_lon(longitude)
      , m_lon_acc(longitude_accuracy)
      , m_alt(altitude)
      , m_alt_acc(altitude_accuracy)
      , m_head(heading)
      , m_head_acc(heading_accuracy)
      {
      }

   const TValue& get_lat() const
      {
         return m_lat;
      }
   const TValue& get_lon() const
      {
         return m_lon;
      }
   const TValue& get_alt() const
      {
         return m_alt;
      }

private:

   TTimestamp m_ts;
   TValue     m_utc;
   TValue     m_lat;
   TAccuracy  m_lat_acc;
   TValue     m_lon;
   TAccuracy  m_lon_acc;
   TValue     m_alt;
   TAccuracy  m_alt_acc;
   TValue     m_head;
   TAccuracy  m_head_acc;

};


/*
class CalcSensor
{
public:
   CalcSensor(); //build invalid sensor
   CalcSensor( 
      const TSensor& type,
      const TTimestamp& timestamp,
      const TValue& value,
      const TAccuracy& accuracy
   );
   bool       is_valid() const;
   TSensor    get_type() const;
   TTimestamp get_timestamp() const;
   TValue     get_value() const;
   TAccuracy  get_accuracy() const;
private:

   TSensor    m_Type;
   TTimestamp m_Timestamp;
   TValue     m_Value;
   TAccuracy  m_Accuracy;
};


//Position is valid only if at least one of 
class Position
{
public:

   Position(
      const TTimestamp& timestamp,
      const TValue&     utc,
      const TValue&     latitude,
      const TAccuracy&  latitude_accuracy,
      const TValue&     longitude,
      const TAccuracy&  longitude_accuracy,
      const TValue&     altitude,
      const TAccuracy&  altitude_accuracy,
      const TValue&     heading,
      const TAccuracy&  heading_accuracy,
   );

   bool       is_valid() const;
   const TValue& get_utc() const;
   CalcSensor    get_lat() const;
   CalcSensor    get_lon() const;
   CalcSensor    get_alt() const;
   CalcSensor    get_head() const;

private:

   TTimestamp m_ts;
   TValue     m_utc;
   TValue     m_lat;
   TAccuracy  m_lat_acc;
   TValue     m_lon;
   TAccuracy  m_lon_acc;
   TValue     m_alt;
   TAccuracy  m_alt_acc;
   TValue     m_head;
   TAccuracy  m_head_acc;

};


// vehicle coordinate systems: ISO 8855 
class Gyro1D
{
public:

   Gyro1D(
      const TTimestamp& timestamp,    // timestamp [s]
      const TValue&     yaw,          // Yaw rate angular velocity [deg/s]
   );

   bool       is_valid() const;
private:
   TTimestamp m_ts;
   TValue     m_yaw;
};

class Odo1Wheel
{
public:
   Odo1Wheel(
      const TTimestamp& timestamp,      //timestamp [s]
      const TValue&     ticks,          //distance since last update [m or ticks].
   );
   bool       is_valid() const;
private:
   TTimestamp m_ts;
   TValue     m_ticks;
};
*/

} //namespace PE
#endif //__PE_Sensor_H__
