#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H
#include <FlexCAN.h>

#include <Eigen.h>
#include <Eigen/Core>
using namespace Eigen;

extern FlexCAN CANbus0;

class motorController
{
private:
	int ID;

public:
	float kp, kd;						  // position gain and velocity gain
	float posEst, velocityEst, touqueEst; // estimate position, velocity, touque
	float posDis, velocityDis, touqueDis; // reference position, velocity, touque

	motorController(int canID, float initPos);
	~motorController();
	void powerOn();
	void powerOff();
};

class legController
{
private:
	bool isCANInit;
	bool type;

	float rollOffset, hipOffset, kneeOffset;
	float baseLength, upperLength, lowerLength; //WARNING!!!!!! the upper length is the distance between hip joint and knee joint, the lower length is the distance between knee joint and feet!!!!!

public:
	motorController *abad;
	motorController *hip;
	motorController *knee;

	bool isContact;
	Vector3f posEst, velocityEst, forceEst; //the estimate position, velocity, force of feet
	legController(int canID[3], int initPos[3], float length, bool legType);
	~legController();
	void powerOn();
	void powerOff();
	void CANInit();
	void forwardKine();
};

#ifdef DEBUG_LEG
void print_mtxf(const MatrixXf &X);
#endif

#endif
