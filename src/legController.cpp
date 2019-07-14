#include "legController.h"
#include "Arduino.h"
#include "config.h"
#include <FlexCAN.h>
#include <Eigen.h>
#include <Eigen/Core>

using namespace Eigen;

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

legController::legController(int canID[3], int initPos[3], float length, bool legType)
{
	abad = new motorController(canID[0], initPos[0]);
	hip = new motorController(canID[1], initPos[1]);
	knee = new motorController(canID[2], initPos[2]);
	type = legType;
}

legController::~legController()
{
	delete abad;
	delete hip;
	delete knee;
}

void legController::powerOn()
{
	abad->powerOn();
	hip->powerOn();
	knee->powerOn();
}
void legController::powerOff()
{
	abad->powerOff();
	hip->powerOff();
	knee->powerOff();
}
void legController::CANInit()
{
	if (isCANInit)
		return;
	CANbus0.begin();
}

void legController::forwardKine()
{
	if (type = FRONT_LEG)
		posEst(0) = -cos(hip->posEst) * upperLength - cos(hip->posEst + knee->posEst) * lowerLength + baseLength;

	else
		posEst(0) = -cos(hip->posEst) * upperLength - cos(hip->posEst + knee->posEst) * lowerLength - baseLength;
	float L = -sin(hip->posEst) * upperLength - sin(hip->posEst + knee->posEst);
	posEst(1) = sin(abad->posEst) * baseLength;
	posEst(2) = cos(abad->posEst) * baseLength;
}

#ifdef DEBUG_LEG
void print_mtxf(const MatrixXf &X)
{
	int i, j, nrow, ncol;

	nrow = X.rows();
	ncol = X.cols();

	Serial.print("nrow: ");
	Serial.println(nrow);
	Serial.print("ncol: ");
	Serial.println(ncol);
	Serial.println();

	for (i = 0; i < nrow; i++)
	{
		for (j = 0; j < ncol; j++)
		{
			Serial.print(X(i, j), 6); // print 6 decimal places
			Serial.print(", ");
		}
		Serial.println();
	}
	Serial.println();
}
#endif