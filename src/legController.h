#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H
#include <FlexCAN.h>
#include <Eigen.h>
#include <Eigen/Core>

extern FlexCAN CANbus0;

class motorController
{
private:
	int ID;

public:
	float kp, kd;						  // position gain and velocity gain
	float posEst, velocityEst, touqueEst; // estimate position, velocity, touque
	float posRef, velocityRef, touqueRef; // reference position, velocity, touque

	motorController(int canID, float initPos);
	~motorController();
	void powerOn();
	void powerOff();
	void send();
	void receive();
};

class legController
{
private:
	bool isCANInit;
	float rollOffset, hipOffset, kneeOffset;

public:
	motorController *roll;
	motorController *hip;
	motorController *knee;

	bool isContact;

	legController(int canID[3], int initPos[3]);
	~legController();
	void powerOn();
	void powerOff();
	void CANInit();
};

extern void print_mtxf(const Eigen::MatrixXf &X);

#endif
