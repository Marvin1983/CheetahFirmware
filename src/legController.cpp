#include "legController.h"
#include "Arduino.h"
#include "config.h"
#include <FlexCAN.h>
#include <Eigen.h>
#include <Eigen/Core>

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
void motorController::send(){};
void motorController::receive(){};

legController::legController(int canID[3], int initPos[3])
{
	roll = new motorController(canID[0], initPos[0]);
	hip = new motorController(canID[1], initPos[1]);
	knee = new motorController(canID[2], initPos[2]);
}

legController::~legController()
{
	delete roll;
	delete hip;
	delete knee;
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
void legController::CANInit()
{
	if (isCANInit)
		return;
	CANbus0.begin();
}

void print_mtxf(const Eigen::MatrixXf &X)
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