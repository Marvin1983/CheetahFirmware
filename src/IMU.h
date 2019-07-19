#ifndef IMU_H
#define IMU_H
#include <Arduino.h>
#include <Eigen.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <ChRt.h>
#include <MPU9250.h>
#include "fastMath.h"
#include "config.h"

using namespace Eigen;
class kalman
{
private:
	Matrix2f F; //State-transition model
	Matrix2f H; //Observation model
	Matrix2f K; //Kalman gain
	Matrix2f Q;
	Matrix2f R;
	Matrix2f P;
	Vector2f Z;
	Matrix2f S;
	void predict();

public:
	Vector2f X; // State
	kalman(float dt, float qAngle, float qBias, float r);
	~kalman();
	void update(float anlge, float velocity);
};

float getRoll(float *read, float norm);
float getPitch(float *read, float norm);


extern THD_WORKING_AREA(waIMUThread, 2048);

extern THD_FUNCTION(IMUThread, arg);
#endif