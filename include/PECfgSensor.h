#ifndef __PECfgSensor_H__
#define __PECfgSensor_H__
namespace PE
{

class PECfgSensor
{
public:
	PECfgSensor(
		double x = 0.0,
		double y = 0.0,
		double z = 0.0,
		double xRot = 0.0,
		double yRot = 0.0,
		double zRot = 0.0
	)
	: m_x(x)
	, m_y(y)
	, m_z(z)
	, m_xRot(xRot)
	, m_yRot(yRot)
	, m_zRot(zRot)
	{};

	const double& getX() const
	{
		return m_x;
	};
	const double& getY() const
	{
		return m_y;
	};
	const double& getZ() const
	{
		return m_z;
	};
	const double& getXRot() const
	{
		return m_xRot;
	};
	const double& getYRot() const
	{
		return m_yRot;
	};
	const double& getZRot() const
	{
		return m_zRot;
	};

private:
	double m_x; 
	double m_y; 
	double m_z; 
	double m_xRot;
	double m_yRot;
	double m_zRot;
};
}
#endif //__PECfgSensor_H__