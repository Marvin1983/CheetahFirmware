#include "IMU.h"

kalman::kalman(float dt)
{
	F << 1, dt,
		0, 1;
}

kalman::~kalman()
{
}

MPU9250 IMU(SPI1, 31);

THD_WORKING_AREA(waIMUThread, 2048);

THD_FUNCTION(IMUThread, arg)
{
	(void)arg;
	int status = IMU.begin();

	if (status < 0)
	{
		Serial.println("IMU initialization unsuccessful");
		Serial.println("Check IMU wiring or try cycling power");
		Serial.print("Stop!!!");
		while (1)
			;
	}
	systime_t wakeTime = chVTGetSystemTimeX();

	while (true)
	{
		IMU.readSensor();
		// display the data
		Serial.print(IMU.getAccelX_mss(), 6);
		Serial.print("\t");
		Serial.print(IMU.getAccelY_mss(), 6);
		Serial.print("\t");
		Serial.print(IMU.getAccelZ_mss(), 6);
		Serial.print("\t");
		Serial.print(IMU.getGyroX_rads(), 6);
		Serial.print("\t");
		Serial.print(IMU.getGyroY_rads(), 6);
		Serial.print("\t");
		Serial.print(IMU.getGyroZ_rads(), 6);
		Serial.print("\t");
		Serial.print(IMU.getMagX_uT(), 6);
		Serial.print("\t");
		Serial.print(IMU.getMagY_uT(), 6);
		Serial.print("\t");
		Serial.println(IMU.getMagZ_uT(), 6);

#ifdef DEBUG_IMU
		counter++;
#endif
		wakeTime += MS2ST((uint32_t)1000 / F_IMU_THREAD);
		chThdSleepUntil(wakeTime);
	}
}
