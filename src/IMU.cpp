#include "IMU.h"

THD_WORKING_AREA(waIMUThread, 2048);

THD_FUNCTION(IMUThread, arg)
{
	(void)arg;

	systime_t wakeTime = chVTGetSystemTimeX();

	while (true)
	{

#ifdef DEBUG_IMU
		counter++;
#endif
		wakeTime += MS2ST((uint32_t)1000 / F_IMU_THREAD);
		chThdSleepUntil(wakeTime);
	}
}
