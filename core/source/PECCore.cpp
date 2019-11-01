/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2019 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */
#include "PECCore.h"

PECCore::PECCore()
// : m_Odo_Ts(std::numeric_limits<PE::TTimestamp>::quiet_NaN())
// , m_Speed_Ts(std::numeric_limits<PE::TTimestamp>::quiet_NaN())
{
}


PECCore::~PECCore()
{
}


bool PECCore::Start(const std::string& cfg)
{
   m_Cfg_Str = cfg;
   return true;
}


const std::string& PECCore::Stop()
{
   return m_Cfg_Str;
}


void PECCore::Clean()
{
}


void PECCore::Calculate()
{
}


void PECCore::SendCoordinates( const double& timestamp, const double& latitude, const double& longitude, const double& accuracy)
{
}


void PECCore::SendHeading( const double& timestamp, const double& heading, const double& accuracy)
{
}


// void PECCore::CleanOdoStep()
// {
//    m_Odo_Ts = std::numeric_limits<PE::TTimestamp>::quiet_NaN();
//    m_Odo_Calib.CleanLastStep();
//    m_Speed_Ts = std::numeric_limits<PE::TTimestamp>::quiet_NaN();
// }
// 
// 
// bool PECCore::IsSpeedTimestampOk( const double& timestamp )
// {
//    return ( PE::MAX_TIMESTAMP > timestamp && PE::MIN_TIMESTAMP < timestamp && timestamp > m_Speed_Ts ) //comparing with NaN aways false
// }
// 
// 
// bool PECCore::IsSpeedOk( const double& speed )
// {
//    return ( PE::MAX_SPEED > speed && PE::MIN_SPEED < speed ); //comparing with NaN aways false
// }

/*
 * ToBe activate when accuracy handling will be finished
 * bool PECCore::IsSpeedAccuracyOk( const double& accuracy )
 * {
 *    if ( PE::MAX_ACCURACY > accuracy && PE::MIN_ACCURACY < accuracy )//comparing with NaN aways false
 *    {
 *       if ( PE::DEFAULT_RELIABLE_LIMIT < m_Speed_Acc.GetReliable() )
 *       {
 *          if ( (accuracy m_Speed_Acc.GetMean() + m_Speed_Acc.GetMld()) < accuracy )
 *          {
 *             return false;
 *          }
 *       }
 *       return true;
 *    }
 *    return false;
 * }
 *
 */

void PECCore::SendSpeed( const double& timestamp, const double& speed, const double& accuracy/* maybe not needed */)
{
//    if ( PE::isnan( m_Speed_Ts ) )
//    {
//       m_Speed_Ts = timestamp;
//       return;
//    }
// 
// //    if ( IsSpeedTimestampOk(timestamp) && IsSpeedOk(speed) && IsSpeedAccuracyOk(accuracy) )
//    if ( IsSpeedTimestampOk(timestamp) && IsSpeedOk(speed) )
//    {
//       m_Odo_Calib.AddRef( speed * (timestamp - m_Speed_Ts) );
// //       m_Speed_Acc.AddSensor( accuracy );
//       m_Speed_Ts = timestamp;
//    }
//    else
//    {
//       CleanOdoStep(); //clean current step of calculation
//    }
}


void PECCore::SendGyro( const double& timestamp, const double& gyro)
{
}


// bool PECCore::IsOdoTimestampOk( const double& timestamp )
// {
//    return ( PE::MAX_TIMESTAMP > timestamp && PE::MIN_TIMESTAMP < timestamp && timestamp > m_Odo_Ts ) //comparing with NaN aways false
// }
// 
// 
// bool PECCore::IsOdoOk( const double& odo )
// {
//    return ( PE::MAX_VALUE > odo && PE::MIN_VALUE < odo ); //comparing with NaN aways false
// }
// 
// 
void PECCore::SendOdo( const double& timestamp, const double& odo)
{
//    if ( PE::isnan( m_Odo_Ts ) )
//    {
//       m_Odo_Ts = timestamp;
//       return;
//    }
// 
//    if ( IsOdoTimestampOk(timestamp) && IsOdoOk(odo) )
//    {
//       m_Odo_Calib.AddRaw( odo );
//       m_Odo_Ts = timestamp;
//    }
//    else //NaN or time back jump
//    {
//       CleanOdoStep(); //clean current step of calculation
//    }
}


bool PECCore::ReceivePosition( double& timestamp, double& latitude, double& longitude, double& coordinatesAccuracy, double& heading, double& headingAccuracy, double& speed, double& speedAccuracy)
{
   return true;
}


bool PECCore::ReceiveDistance( double& distance, double& accuracy)
{
   return true;
}


bool PECCore::ReceiveGyroStatus( double& bias, double& scale, double& reliable, double& accuracy)
{
   return true;
}


bool PECCore::ReceiveOdoStatus( double& bias, double& scale, double& reliable, double& accuracy)
{
//    if ( 0 == m_Odo_Scale.GetSampleCount() || 0 == m_Odo_Scale.GetSampleCount() )
//    {
//       return false;
//    }
//    bias = m_Odo_Bias.GetMean();
//    scale = m_Odo_Scale.GetMean();
//    reliable = (m_Odo_Bias.GetReliable() + m_Odo_Scale.GetReliable()) / 2;
//    accuracy = m_Odo_Bias.GetMld() * m_Odo_Scale.GetMean();
   return true;
}
