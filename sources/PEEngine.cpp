
#include "PEEngine.h"

using namespace PE;

State::State( const TValue& distance_per_tick, const TAccuracy& acc, const TAccuracy& acc_limit, const TValue& calibration )
: m_distance_per_tick(distance_per_tick)
, m_acc(acc)
, m_acc_limit(acc_limit)
, m_callibration(calibration)
{
   _clear();
}

void State::reset()
{
   _clear();
}


void State::set_src_distance(const TValue& distance, const TAccuracy& acc)
{
   if (m_acc_limit<acc)
   {
      _clear();
   }
   else if (PE::INVALID == m_src_distnace)
   {
      _clear();
      m_src_distnace  = 0;
      m_src_acc_sum   = acc;
      m_src_acc_count = 1;
      m_src_acc       = acc;
   }
   else
   {
      m_src_distnace+=distance;
      _process();
      m_src_acc_sum+=acc;
      m_src_acc_count++;
      m_src_acc=m_src_acc_sum / m_src_acc_count;
   }
}

void State::set_odo_ticks(const TValue& ticks)
{
   m_odo_ticks+=ticks;
}

const TValue& State::get_distance_per_tick() const
{
   return m_distance_per_tick;
}

const TAccuracy& State::get_accuracy() const
{
   return m_acc;
}

const TValue& State::get_calibartion() const
{
   return m_callibration;
}

void State::_clear()
{
   m_src_distnace = PE::INVALID;
   m_src_acc_sum = 0;
   m_src_acc_count = 0;
   m_src_acc = m_acc_limit;
   m_odo_ticks = 0;
}


void State::_process()
{
   if (0 < m_src_acc && 0 < m_odo_ticks)
   {
      TValue raw_calibration       = m_src_distnace / m_src_acc;
      TValue raw_distance_per_tick = m_src_distnace / m_odo_ticks;
      if ( raw_distance_per_tick > 0 &&
           raw_distance_per_tick != m_distance_per_tick &&
           raw_calibration > m_callibration &&
           m_src_acc > m_acc )
      {
         m_distance_per_tick = raw_distance_per_tick;
         m_acc = m_src_acc;
         m_callibration = 100.0 < raw_calibration ? 100.0 : raw_calibration;
      }
   }
}


