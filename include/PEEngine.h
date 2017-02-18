#ifndef __PE_Engine_H__
#define __PE_Engine_H__

#include <vector>
#include "PETypes.h"
#include "PESensors.h"
namespace PE
{


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

} //namespace PE
#endif //__PE_Engine_H__
