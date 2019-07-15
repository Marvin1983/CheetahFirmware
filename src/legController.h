#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H
#include <FlexCAN.h>
#include <Eigen.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include "fastMath.h"

using namespace Eigen;

struct MotorPDGain
{
	float kpAbad;
	float kdAbad;
	float kpHip;
	float kdHip;
	float kpKnee;
	float kdKnee;
};
struct feetPDGain
{
	float kpLeg;
	float kdLeg;
};
extern struct MotorPDGain motorGain[4];
extern struct feetPDGain feetGain[4];

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
	void setGain(struct MotorPDGain);
};

class legController
{
private:
	int id;
	int port;
	int mode;
	bool isCANInit;
	float rollInit, hipInit, kneeInit; //the inital angle of three joint
	float baseOffset[2], l1, l2;
	float kp, kd;
	jointController *abad;
	jointController *hip;
	jointController *knee;
	CAN_message_t rxMsg; //receive message
	Matrix3f jacobian, inverseJacobian;

	void CANInit();
	void packAll();
	void writeAll();
	void zeroAll();

	//WARNING!!!!!! the upper length(l1) is the distance between hip joint and knee joint, the lower length(l2) is the distance between knee joint and feet!!!!!
	//baseOffset[0]: X Offset ;baseOffset[1]: Y OffSet
public:
	bool isContact;
	Vector3f pDes, vDes, fFF;
	Vector3f pEst, vEst, fEst; //the estimate position, velocity, force of feet
	Vector3f vEstM, tEstM;	 // velocity and touque of each motor
	Vector3f fOut, tOut;
	legController(uint32_t canID[3], float initPos[3], float length[2], float offset[2], int legID, int CANPort);
	~legController();

	void unpackReply(CAN_message_t msg);
	void motorOnAll();
	void motorOffAll();
	void changesMode(int mode);
	void updateState();

	void control();
};

#ifdef DEBUG_LEG
void print_mtxf(const MatrixXf &X);
#endif

#endif
