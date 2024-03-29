#ifndef IMU_H
#define IMU_H
#include <Arduino.h>
#include <Eigen.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <ChRt.h>
#include <quaternionFilters.h>
#include <MPU9250.h>
#include "fastMath.h"
#include "config.h"

using namespace Eigen;

extern THD_WORKING_AREA(waIMUThread, 20480);

extern THD_FUNCTION(IMUThread, arg);
#endif