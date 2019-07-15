#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H
#include <FlexCAN.h>
#include <Eigen.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include "fastMath.h"

using namespace Eigen;

class jointController
{
private:
	int mode;

public:
	CAN_message_t txMsg;
	float kp, kd;			// position gain and velocity gain
	float pEst, vEst, tEst; // estimated position, velocity, touque
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
	int mode;
	CAN_message_t rxMsg; //receive message
	Matrix3f jacobian, inverseJacobian;

	float rollInit, hipInit, kneeInit; //the inital angle of three joint
	float baseOffset[2], l1, l2;
	//WARNING!!!!!! the upper length(l1) is the distance between hip joint and knee joint, the lower length(l2) is the distance between knee joint and feet!!!!!
	//baseOffset[0]: X Offset ;baseOffset[1]: Y OffSet
public:
	jointController *abad;
	jointController *hip;
	jointController *knee;

	bool isContact;
	Vector3f pEst, vEst, fEst; //the estimate position, velocity, force of feet
	Vector3f vEstM, tEstM;	 // velocity and touque of each motor
	legController(uint32_t canID[3], float initPos[3], float length[2], float offset[2], int legType, int CANPort);
	~legController();
	void CANInit();

	void unpackReply(CAN_message_t msg);
	void packAll();
	void writeAll();
	void zeroAll();
	void motorOnAll();
	void motorOffAll();
	void updateState();
};

#ifdef DEBUG_LEG
void print_mtxf(const MatrixXf &X);
#endif

#endif
