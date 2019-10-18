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
{
}


PECCore::~PECCore()
{
}


bool PECCore::Start(const std::string& cfg)
{
   m_cfg_str = cfg;
   return true;
}


const std::string& PECCore::Stop()
{
   return m_cfg_str;
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


void PECCore::SendSpeed( const double& timestamp, const double& speed, const double& accuracy)
{
}


void PECCore::SendGyro( const double& timestamp, const double& gyro)
{
}


void PECCore::SendOdo( const double& timestamp, const double& odo)
{
}


bool PECCore::ReceivePosition( double& timestamp, double& latitude, double& longitude, double& coordinatesAccuracy, double& heading, double& headingAccuracy, double& speed, double& speedAccuracy)
{
   return true;
}


bool PECCore::ReceiveDistance( double& distance, double& accuracy)
{
   return true;
}


bool PECCore::ReceiveGyroStatus( double& base, double& scale, double& reliable, double& accuracy)
{
   return true;
}


bool PECCore::ReceiveOdoStatus( double& base, double& scale, double& reliable, double& accuracy)
{
   return true;
}
