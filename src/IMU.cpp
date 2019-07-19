#include "IMU.h"
MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

THD_WORKING_AREA(waIMUThread, 20480);

THD_FUNCTION(IMUThread, arg)
{
	(void)arg;
	// Read the WHO_AM_I register, this is a good test of communication
	Wire.begin();
	byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	if (c == 0x71) // WHO_AM_I should always be 0x71
	{

		// Calibrate gyro and accelerometers, load biases in bias registers
		// Start by performing self test and reporting values
		myIMU.MPU9250SelfTest(myIMU.selfTest);

		myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
	}
	else
	{
		Serial.println("[IMU]\tConnection failed, abort!");
		abort();
	}
	myIMU.initMPU9250();
	byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
	if (d != 0x48)
	{
		// Communication failed, stop here
		Serial.println(F("[IMU] \tCommunication failed, abort!"));
		Serial.flush();
		abort();
	}
	myIMU.initAK8963(myIMU.factoryMagCalibration);

	// Get sensor resolutions, only need to do this once
	myIMU.getAres();
	myIMU.getGres();
	myIMU.getMres();

	Serial.println("[IMU]\tConnect and set up IMU successfully, IMU is runing.");

	systime_t wakeTime = chVTGetSystemTimeX();
	while (true)
	{
#ifdef DEBUG_IMU
		counter++;
#endif
		if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
		{
			myIMU.readAccelData(myIMU.accelCount); // Read the x/y/z adc values

			// Now we'll calculate the accleration value into actual g's
			// This depends on scale being set
			myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
			myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
			myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

			myIMU.readGyroData(myIMU.gyroCount); // Read the x/y/z adc values

			// Calculate the gyro value into actual degrees per second
			// This depends on scale being set
			myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
			myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
			myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

			myIMU.readMagData(myIMU.magCount); // Read the x/y/z adc values

			// Calculate the magnetometer values in milliGauss
			// Include factory calibration per data sheet and user environmental
			// corrections
			// Get actual magnetometer value, this depends on scale being set
			myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
			myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
			myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
		} // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
		  // Must be called before updating quaternions!
		myIMU.updateTime();
		MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
							   myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
							   myIMU.mx, myIMU.mz, myIMU.deltat);
		// Serial print and/or display at 0.5 s rate independent of data rates
		myIMU.delt_t = millis() - myIMU.count;

		// update LCD once per half-second independent of read rate
		if (myIMU.delt_t > 500)
		{

			Serial.print("ax = ");
			Serial.print((int)1000 * myIMU.ax);
			Serial.print(" ay = ");
			Serial.print((int)1000 * myIMU.ay);
			Serial.print(" az = ");
			Serial.print((int)1000 * myIMU.az);
			Serial.println(" mg");

			Serial.print("gx = ");
			Serial.print(myIMU.gx, 2);
			Serial.print(" gy = ");
			Serial.print(myIMU.gy, 2);
			Serial.print(" gz = ");
			Serial.print(myIMU.gz, 2);
			Serial.println(" deg/s");

			Serial.print("mx = ");
			Serial.print((int)myIMU.mx);
			Serial.print(" my = ");
			Serial.print((int)myIMU.my);
			Serial.print(" mz = ");
			Serial.print((int)myIMU.mz);
			Serial.println(" mG");

			Serial.print("q0 = ");
			Serial.print(*getQ());
			Serial.print(" qx = ");
			Serial.print(*(getQ() + 1));
			Serial.print(" qy = ");
			Serial.print(*(getQ() + 2));
			Serial.print(" qz = ");
			Serial.println(*(getQ() + 3));

			// Define output variables from updated quaternion---these are Tait-Bryan
			// angles, commonly used in aircraft orientation. In this coordinate system,
			// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
			// x-axis and Earth magnetic North (or true North if corrected for local
			// declination, looking down on the sensor positive yaw is counterclockwise.
			// Pitch is angle between sensor x-axis and Earth ground plane, toward the
			// Earth is positive, up toward the sky is negative. Roll is angle between
			// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
			// arise from the definition of the homogeneous rotation matrix constructed
			// from quaternions. Tait-Bryan angles as well as Euler angles are
			// non-commutative; that is, the get the correct orientation the rotations
			// must be applied in the correct order which for this configuration is yaw,
			// pitch, and then roll.
			// For more see
			// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
			// which has additional links.
			myIMU.yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() * *(getQ() + 3)), *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
			myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() * *(getQ() + 2)));
			myIMU.roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) * *(getQ() + 3)), *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
			myIMU.pitch *= RAD_TO_DEG;
			myIMU.yaw *= RAD_TO_DEG;

			// Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
			//    8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
			// - http://www.ngdc.noaa.gov/geomag-web/#declination
			myIMU.yaw -= 0;
			myIMU.roll *= RAD_TO_DEG;

			Serial.print("Yaw, Pitch, Roll: ");
			Serial.print(myIMU.yaw, 2);
			Serial.print(", ");
			Serial.print(myIMU.pitch, 2);
			Serial.print(", ");
			Serial.println(myIMU.roll, 2);

			Serial.print("rate = ");
			Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
			Serial.println(" Hz");
			myIMU.count = millis();
			myIMU.sumCount = 0;
			myIMU.sum = 0;
		}

		wakeTime += MS2ST((uint32_t)1000 / F_IMU_THREAD);
		chThdSleepUntil(wakeTime);
	}
}