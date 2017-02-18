#ifndef __PESensor_H__
#define __PESensor_H__

#include <cstdint>
#include "PECfgSensor.h"
namespace PE
{
class PESensor
{
public:

	PESensor(
		uint32_t id, 
		PECfgSensor cfg
	)
	{}

	virtual ~PESensor(void)
	{
	}
private:
	uint32_t m_id;
	uint32_t m_timestamp;
	PECfgSensor m_cfg;
};
}
#endif //__PESensor_H__
