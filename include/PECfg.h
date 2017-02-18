#ifndef __PE_CfgSensor_H__
#define __PE_CfgSensor_H__

#include "PETypes.h"

namespace PE
{


class CfgSensor
{
public:
   CfgSensor( TSensorID id, double x, double y, double z, double xRot, double yRot, double zRot );
   TSensorID get_id() const;
   double getX() const;
   double getY() const;
   double getZ() const;
   double getXRot() const;
   double getYRot() const;
   double getZRot() const;
   double getKalmanGain() const;
   void   setKalmanGain(double KalmanGain);
   double getCalibrated() const;
   void   setCalibrated(double Calibrated);

private:
   TSensorID m_id;
   double m_x;
   double m_y;
   double m_z;
   double m_xRot;
   double m_yRot;
   double m_zRot;
   double m_Kalman_Gain;
   double m_Calibrated;
};


} //namespace PE
#endif //__PE_CfgSensor_H__
