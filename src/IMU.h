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
	Vector2f pX; //Predicted state
	Matrix2f F;  //State-transition model
	Matrix2f H;  //Observation model
	Matrix2f K;  //Kalman gain

public:
	Vector2f fX; //Fixed state
	kalman(float dt);
	~kalman();
};

extern THD_WORKING_AREA(waIMUThread, 2048);

extern THD_FUNCTION(IMUThread, arg);
#endif