#include "IMU.h"

kalman::kalman(float dT, float qAngle, float qBias, float r)
{
	F << 1, dT,
		0, 1;
	H << 1, 0,
		0, 1;
	Q << qAngle, 0,
		0, qBias;
}

kalman::~kalman()
{
}

void kalman::predict()
{
	X = F * X;
	P = F * P * F.transpose();
}
void kalman ::update(float angle, float velocity)
{
	Z << angle, velocity;
	S = H * P * H.transpose() + R;
	K = P * H.transpose() * S.inverse();
	X = X + K * (Z - H * X);
	P = P - K * H * P;
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
		Serial.println("Stop!!!");
		while (1)
			;
	}
	IMU.setSrd(F_IMU_THREAD - 1);
	kalman roll(1.0f / F_IMU_THREAD, 0.001f, 0.003f, 0.3f);

	systime_t wakeTime = chVTGetSystemTimeX();

	while (true)
	{
		IMU.readSensor();
		float accXYZ[3] = {IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss()};
		float accNorm = sqrt(accXYZ[0] * accXYZ[0] + accXYZ[1] * accXYZ[1] + accXYZ[2] * accXYZ[2]);
		float rollFAcc = getRoll(accXYZ, accNorm); //roll angle form accXYZ
		Serial.println();

#ifdef DEBUG_IMU
		counter++;
#endif
		wakeTime += MS2ST((uint32_t)1000 / F_IMU_THREAD);
		chThdSleepUntil(wakeTime);
	}
}
float getRoll(float *read, float norm)
{
	float fNormXZ = sqrt(read[0] * read[0] + read[2] * read[2]);
	float fCos = fNormXZ / norm;
	return acos(fCos) * 57.295779513f;
}

float getPitch(float *read, float norm)
{
	float fNormYZ = sqrt(read[1] * read[1] + read[2] * read[2]);
	float fCos = fNormYZ / norm;
	return acos(fCos) * 57.295779513f;
}
