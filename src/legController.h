#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H
#include <FlexCAN.h>

#include "fastMath.h"

#include <Eigen.h>
#include <Eigen/Core>
using namespace Eigen;

typedef struct CANMessage
{
	uint32_t id;	  // can identifier
	uint8_t ext;	  // identifier is extended
	uint8_t len;	  // length of data
	uint16_t timeout; // milliseconds, zero will disable waiting
	uint8_t buf[8];
} CANMessage;

class jointController
{
private:
	CANMessage canTX;

public:
	float kp, kd;			// position gain and velocity gain
	float pEst, vEst, tEst; // estimate position, velocity, touque
	float pDes, vDes, tFF;  // reference position, velocity, touque

	jointController(uint32_t canID, float initPos);
	~jointController();
	void packCmd(CANMessage *msg);

	void powerOn();
	void powerOff();
};

class legController
{
private:
	bool isCANInit;
	int type;
	CANMessage canRX; //receive message

	float rollOffset, hipOffset, kneeOffset;
	float baseLength, upperLength, lowerLength; //WARNING!!!!!! the upper length is the distance between hip joint and knee joint, the lower length is the distance between knee joint and feet!!!!!

public:
	jointController *abad;
	jointController *hip;
	jointController *knee;

	bool isContact;
	Vector3f posEst, velocityEst, forceEst; //the estimate position, velocity, force of feet
	legController(uint32_t canID[3], int initPos[3], float length, bool legType);
	~legController();
	void unpackReply(CANMessage msg);
	void packAll();
	void powerOn();
	void powerOff();
	void CANInit();
	void forwardKine();
};

#ifdef DEBUG_LEG
void print_mtxf(const MatrixXf &X);
#endif

#endif
