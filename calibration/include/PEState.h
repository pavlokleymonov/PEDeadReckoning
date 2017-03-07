#ifndef __PE_State_H__
#define __PE_State_H__

#include <vector>
#include "PETypes.h"
namespace PE
{

/**
 * Makes calibartion of incoming sensor value to the reference value
 *
 */
class calibration
{
public:
   /**
    * Constructor of calibration
    *
    * @param  init_values_per_step     initial values per one step of the sensor
    * @param  init_sensor_accuracy     initial accuracy of the sensor according to values
    * @param  init_sensor_calibration  initial sensor calibration status. range [0..100] percent.
    *                                  0% - uncallibrated, 100% - fully calibrated
    * @param  accuracy_limit           accuracy limit till wich all incoming values will not be used
    */
   calibration(const TValue& init_values_per_step, const TAccuracy& init_sensor_accuracy, const TValue& init_sensor_calibration, const TAccuracy& accuracy_limit);
   /**
    * Adds new reference value to the calibration
    *
    * @param  reference_values     values of reference signal
    * @param  reference_accuracy   accuracy of the reference signal
    */
   void set_reference_value(const TValue& reference_values, const TAccuracy& reference_accuracy);
   /**
    * Adds new sensor steps to the calibration
    *
    * @param  sensor_steps     sensor steps
    */
   void set_sensor_steps(const TValue& sensor_steps);
   /**
    * Returns reference values per one sensor step
    *
    * @return    how many values per one sensor step
    */
   const TValue& get_values_per_step() const;
   /**
    * Returns sensor accuracy
    *
    * @return    accuracy of the sensors according to values
    */
   const TAccuracy& get_sensor_accuracy() const;
   /**
    * Returns sensor calibartion status
    *
    * @return    calibration status. range [0..100] percent.
    */
   const TValue& get_sensor_calibration() const;
private:
   TValue    m_value_per_step;
   TAccuracy m_sensor_accuracy;
   TValue    m_sensor_calibration;
   TAccuracy m_accuracy_limit;
};


/*
class State
{
public:
   State( const TValue& distance_per_tick, const TAccuracy& acc, const TAccuracy& acc_limit, const TValue& calibration );

   void reset();

   void set_src_distance(const TValue& distance, const TAccuracy& acc);

   void set_odo_ticks(const TValue& ticks);

   const TValue& get_distance_per_tick() const;

   const TAccuracy& get_accuracy() const; //in meter

   const TValue& get_calibartion() const; //in %
private:

   TValue m_distance_per_tick;
   TAccuracy m_acc;
   TAccuracy m_acc_limit;
   TValue m_callibration;


   TValue m_src_distnace;
   TValue m_src_acc;
   TValue m_src_acc_sum;
   uint64_t m_src_acc_count;
   TValue m_odo_ticks;

   void _clear();
   void _process();

};
*/







} //namespace PE
#endif //__PE_State_H__
