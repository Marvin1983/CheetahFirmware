#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H
#include <FlexCAN.h>

#include "fastMath.h"

#include <Eigen.h>
#include <Eigen/Core>
using namespace Eigen;

class jointController
{
private:
public:
	CAN_message_t txMsg;
	float kp, kd;			// position gain and velocity gain
	float pEst, vEst, tEst; // estimate position, velocity, touque
	float pDes, vDes, tFF;  // reference position, velocity, touque

	jointController(uint32_t canID, float initPos);
	~jointController();
	void packCmd();
};

class legController
{
private:
	bool isCANInit;
	int type;
	int port;
	CAN_message_t rxMsg; //receive message

	float rollOffset, hipOffset, kneeOffset;
	float baseLength, upperLength, lowerLength; //WARNING!!!!!! the upper length is the distance between hip joint and knee joint, the lower length is the distance between knee joint and feet!!!!!

public:
	jointController *abad;
	jointController *hip;
	jointController *knee;

	bool isContact;
	Vector3f posEst, velocityEst, forceEst; //the estimate position, velocity, force of feet
	legController(uint32_t canID[3], int initPos[3], float length, int legType, int CANPort);
	~legController();
	void CANInit();

	void unpackReply(CAN_message_t msg);
	void packAll();
	void writeAll();
	void forwardKine();
};

#ifdef DEBUG_LEG
void print_mtxf(const MatrixXf &X);
#endif

#endif
