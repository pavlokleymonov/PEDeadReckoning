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
#include <set>
#include <string>
#include "PECore.h"
#include "PECCore.h"

typedef std::set<PECCore*> PETInstanceList;

static PETInstanceList m_list;

static std::string m_last_cfg;

PECCore* PEStart(const char* cfg)
{
   PETInstanceList::iterator it = m_list.insert(new PECCore()).first;
   (*it)->Start(std::string(cfg));
   return *it;
}


const char* PEStop(PECCore* core)
{
   PETInstanceList::iterator it = m_list.find(core);
   if ( m_list.end() != it )
   {
      m_last_cfg = (*it)->Stop();
      delete (*it);
      m_list.erase(it);
      return m_last_cfg.c_str();
   }
   return 0;
}


bool PEClean(PECCore* core)
{
   PETInstanceList::iterator it = m_list.find(core);
   if ( m_list.end() != it )
   {
      (*it)->Clean();
      return true;
   }
   return false;
}


bool PECalculate(PECCore* core)
{
   PETInstanceList::iterator it = m_list.find(core);
   if ( m_list.end() != it )
   {
      (*it)->Calculate();
      return true;
   }
   return false;
}


bool PESendCoordinates(PECCore* core, const double& timestamp, const double& latitude, const double& longitude, const double& accuracy)
{
   PETInstanceList::iterator it = m_list.find(core);
   if ( m_list.end() != it )
   {
      (*it)->SendCoordinates(timestamp,latitude,longitude,accuracy);
      return true;
   }
   return false;
}


bool PESendHeading(PECCore* core, const double& timestamp, const double& heading, const double& accuracy)
{
   PETInstanceList::iterator it = m_list.find(core);
   if ( m_list.end() != it )
   {
      (*it)->SendHeading(timestamp,heading,accuracy);
      return true;
   }
   return false;
}


bool PESendSpeed(PECCore* core, const double& timestamp, const double& speed, const double& accuracy)
{
   PETInstanceList::iterator it = m_list.find(core);
   if ( m_list.end() != it )
   {
      (*it)->SendSpeed(timestamp,speed,accuracy);
      return true;
   }
   return false;
}


bool PESendGyro(PECCore* core, const double& timestamp, const double& gyro)
{
   PETInstanceList::iterator it = m_list.find(core);
   if ( m_list.end() != it )
   {
      (*it)->SendGyro(timestamp,gyro);
      return true;
   }
   return false;
}


bool PESendOdo(PECCore* core, const double& timestamp, const double& odo)
{
   PETInstanceList::iterator it = m_list.find(core);
   if ( m_list.end() != it )
   {
      (*it)->SendOdo(timestamp,odo);
      return true;
   }
   return false;
}


bool PEReceivePosition(PECCore* core, double& timestamp, double& latitude, double& longitude, double& coordinatesAccuracy, double& heading, double& headingAccuracy, double& speed, double& speedAccuracy)
{
   PETInstanceList::iterator it = m_list.find(core);
   if ( m_list.end() != it )
   {
      return (*it)->ReceivePosition(timestamp, latitude, longitude, coordinatesAccuracy, heading, headingAccuracy, speed, speedAccuracy);
   }
   return false;
}


bool PEReceiveDistance(PECCore* core, double& distance, double& accuracy)
{
   PETInstanceList::iterator it = m_list.find(core);
   if ( m_list.end() != it )
   {
      return (*it)->ReceiveDistance(distance, accuracy);
   }
   return false;
}


bool PEReceiveGyroStatus(PECCore* core, double& base, double& scale, double& reliable)
{
   PETInstanceList::iterator it = m_list.find(core);
   if ( m_list.end() != it )
   {
      return (*it)->ReceiveGyroStatus(base, scale, reliable);
   }
   return false;
}


bool PEReceiveOdoStatus(PECCore* core, double& base, double& scale, double& reliable)
{
   PETInstanceList::iterator it = m_list.find(core);
   if ( m_list.end() != it )
   {
      return (*it)->ReceiveOdoStatus(base, scale, reliable);
   }
   return false;
}
