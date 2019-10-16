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
#include "PECore.h"


PECCore* PEStart(const char* cfg)
{
   return 0;
}


const char* PEStop(PECCore* core)
{
   return 0;
}


bool PEClean(PECCore* core)
{
   return false;
}


bool PECalculate(PECCore* core)
{
   return false;
}


bool PESendCoordinates(PECCore* core, const double& timestamp, const double& latutude, const double& longitude, const double& accuracy)
{
   return false;
}


bool PESendHeading(PECCore* core, const double& timestamp, const double& heading, const double& accuracy)
{
   return false;
}


bool PESendSpeed(PECCore* core, const double& timestamp, const double& speed, const double& accuracy)
{
   return false;
}


bool PESendGyro(PECCore* core, const double& timestamp, const double& gyro)
{
   return false;
}


bool PESendOdo(PECCore* core, const double& timestamp, const double& odo)
{
   return false;
}


bool PEReceivePosition(PECCore* core, double& timestamp, double& latutude, double& longitude, double& coordinatesAccuracy, double& heading, double& headingAccuracy, double& speed, double& speedAccuracy)
{
   return false;
}


bool PEReceiveDistance(PECCore* core, double& distance, double& accuracy)
{
   return false;
}


bool PEReceiveGyroStatus(PECCore* core, double& base, double& scale, double& reliable, double& accuracy)
{
   return false;
}


bool PEReceiveOdoStatus(PECCore* core, double& base, double& scale, double& reliable, double& accuracy)
{
   return false;
}
