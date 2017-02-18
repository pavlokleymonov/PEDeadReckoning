
#include "PECfg.h"

using namespace PE;


CfgSensor::CfgSensor( TSensorID id, double x, double y, double z, double xRot, double yRot, double zRot )
: m_id(id)
, m_x(x)
, m_y(y)
, m_z(z)
, m_xRot(xRot)
, m_yRot(yRot)
, m_zRot(zRot)
, m_Kalman_Gain(UNKNOWN_KALMAN_GAIN)
, m_Calibrated(UNCALIBRATED_SENSOR)
{}

TSensorID CfgSensor::get_id() const
{
   return m_id;
}

double CfgSensor::getX() const
{
   return m_x;
}

double CfgSensor::getY() const
{
   return m_y;
}

double CfgSensor::getZ() const
{
   return m_z;
}

double CfgSensor::getXRot() const
{
   return m_xRot;
}

double CfgSensor::getYRot() const
{
   return m_yRot;
}

double CfgSensor::getZRot() const
{
   return m_zRot;
}

double CfgSensor::getKalmanGain() const
{
   return m_Kalman_Gain;
}

void   CfgSensor::setKalmanGain(double KalmanGain)
{
   m_Kalman_Gain = KalmanGain;
}

double CfgSensor::getCalibrated() const
{
   return m_Calibrated;
}

void   CfgSensor::setCalibrated(double Calibrated)
{
   m_Calibrated = Calibrated;
}

