#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H
#include <FlexCAN.h>

#include "fastMath.h"

#include <Eigen.h>
#include <Eigen/Core>
using namespace Eigen;

extern FlexCAN CANbus0;


class jointController
{
private:
	int ID;

public:
	float kp, kd;			// position gain and velocity gain
	float pEst, vEst, tEst; // estimate position, velocity, touque
	float pDes, vDes, tFF;  // reference position, velocity, touque

	jointController(int canID, float initPos);
	~jointController();
	void packCmd(CAN_message_t *msg);

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
	jointController *abad;
	jointController *hip;
	jointController *knee;

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
