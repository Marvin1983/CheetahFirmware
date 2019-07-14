#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

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
};

class legController
{
private:
	bool canIsInit;

public:
	motorController *roll;
	motorController *hip;
	motorController *knee;

	bool contact;
	legController(int canID[3], int initPos[3]);
	~legController();
	void powerOn();
	void powerOff();
	void canInit();
};

#endif
