#include "legController.h"
#include "Arduino.h"
#include "config.h"
#include <FlexCAN.h>

FlexCAN CANbus0(1000000, 0);

motorController::motorController(int canID, float initPos)
{
	ID = canID;
}

motorController::~motorController()
{
}

void motorController::powerOn(){};
void motorController::powerOff(){};

legController::legController(int canID[3], int initPos[3])
{
	roll = new motorController(canID[0], initPos[0]);
	hip = new motorController(canID[1], initPos[1]);
	knee = new motorController(canID[2], initPos[2]);
}

legController::~legController()
{
	delete roll, hip, knee;
}

void legController::powerOn()
{
	roll->powerOn();
	hip->powerOn();
	knee->powerOn();
}
void legController::powerOff()
{
	roll->powerOff();
	hip->powerOff();
	knee->powerOff();
}
void legController::canInit()
{
	if (canIsInit)
		return;
	CANbus0.begin();
}