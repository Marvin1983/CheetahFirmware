#include "legController.h"
#include "Arduino.h"
#include "config.h"
#include "fastMath.h"
#include <FlexCAN.h>
#include <Eigen.h>
#include <Eigen/Core>

using namespace Eigen;
FlexCAN CANbus0(1000000, 0);
FlexCAN CANbus1(1000000, 1);

/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]

void jointController::packCmd()
{

	/// limit data to be within bounds ///
	pDes = fminf(fmaxf(P_MIN, pDes), P_MAX);
	vDes = fminf(fmaxf(V_MIN, vDes), V_MAX);
	kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
	kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
	tFF = fminf(fmaxf(T_MIN, tFF), T_MAX);
	/// convert floats to unsigned ints ///
	uint16_t pInt = float_to_uint(pDes, P_MIN, P_MAX, 16);
	uint16_t vInt = float_to_uint(vDes, V_MIN, V_MAX, 12);
	uint16_t kpInt = float_to_uint(kp, KP_MIN, KP_MAX, 12);
	uint16_t kdInt = float_to_uint(kd, KD_MIN, KD_MAX, 12);
	uint16_t tInt = float_to_uint(tFF, T_MIN, T_MAX, 12);
	/// pack ints into the can buffer ///
	txMsg.buf[0] = pInt >> 8;
	txMsg.buf[1] = pInt & 0xFF;
	txMsg.buf[2] = vInt >> 4;
	txMsg.buf[3] = ((vInt & 0xF) << 4) | (kpInt >> 8);
	txMsg.buf[4] = kpInt & 0xFF;
	txMsg.buf[5] = kdInt >> 4;
	txMsg.buf[6] = ((kdInt & 0xF) << 4) | (tInt >> 8);
	txMsg.buf[7] = tInt & 0xff;
}

jointController::jointController(uint32_t canID, float initPos)
{
	txMsg.len = 8;
	txMsg.id = canID;
}

jointController::~jointController()
{
}

legController::legController(uint32_t canID[3], int initPos[3], float length, int legType, int CANPort)
{
	abad = new jointController(canID[0], initPos[0]);
	hip = new jointController(canID[1], initPos[1]);
	knee = new jointController(canID[2], initPos[2]);
	type = legType;
	port = CANPort;
	rxMsg.len = 6;
}

legController::~legController()
{
	delete abad;
	delete hip;
	delete knee;
}
void legController::unpackReply(CAN_message_t msg)
{
	/// unpack ints from can buffer ///
	uint16_t id = msg.buf[0];
	if ((type == FRONT_LEG && id > 4) || (type == BACK_LEG && id < 4))
		return;
	uint16_t pInt = (msg.buf[1] << 8) | msg.buf[2];
	uint16_t vInt = (msg.buf[3] << 4) | (msg.buf[4] >> 4);
	uint16_t iInt = ((msg.buf[4] & 0xF) << 8) | msg.buf[5];
	/// convert uints to floats ///
	float p = uint_to_float(pInt, P_MIN, P_MAX, 16);
	float v = uint_to_float(vInt, V_MIN, V_MAX, 12);
	float t = uint_to_float(iInt, -T_MAX, T_MAX, 12);

	if (id == 1 || id == 4)
	{
		abad->pEst = p;
		abad->vEst = v;
		abad->tEst = t;
	}
	else if (id == 2 || id == 5)
	{
		hip->pEst = p;
		hip->vEst = v;
		hip->tEst = t;
	}
	else if (id == 3 || id == 6)
	{
		knee->pEst = p;
		knee->vEst = v;
		knee->tEst = t;
	}
}
void legController::packAll()
{
	abad->packCmd();
	hip->packCmd();
	knee->packCmd();
}
void legController::writeAll()
{
	if (port == CAN_0)
	{
		CANbus0.write(abad->txMsg);
		delayMicroseconds(1);
		CANbus0.write(hip->txMsg);
		delayMicroseconds(1);
		CANbus0.write(knee->txMsg);
		delayMicroseconds(1);
	}
	else if (port == CAN_1)
	{
		CANbus1.write(abad->txMsg);
		delayMicroseconds(1);
		CANbus1.write(hip->txMsg);
		delayMicroseconds(1);
		CANbus1.write(knee->txMsg);
		delayMicroseconds(1);
	}
}

void legController::zeroAll()
{
	for (int i = 0; i < 7; i++)
	{
		abad->txMsg.buf[i] = 0xFF;
		hip->txMsg.buf[i] = 0xFF;
		knee->txMsg.buf[i] = 0xFF;
	}
	abad->txMsg.buf[7] = 0xFE;
	hip->txMsg.buf[7] = 0xFE;
	knee->txMsg.buf[7] = 0xFE;

	writeAll();
}
void legController::motorOnAll()
{
	for (int i = 0; i < 7; i++)
	{
		abad->txMsg.buf[i] = 0xFF;
		hip->txMsg.buf[i] = 0xFF;
		knee->txMsg.buf[i] = 0xFF;
	}
	abad->txMsg.buf[7] = 0xFC;
	hip->txMsg.buf[7] = 0xFC;
	knee->txMsg.buf[7] = 0xFC;

	writeAll();
}
void legController::motorOffAll()
{
	for (int i = 0; i < 7; i++)
	{
		abad->txMsg.buf[i] = 0xFF;
		hip->txMsg.buf[i] = 0xFF;
		knee->txMsg.buf[i] = 0xFF;
	}
	abad->txMsg.buf[7] = 0xFD;
	hip->txMsg.buf[7] = 0xFD;
	knee->txMsg.buf[7] = 0xFD;

	//writeAll();
}

void legController::CANInit()
{
	if (isCANInit)
		return;
	CANbus0.begin();
	CANbus1.begin();
}

void legController::forwardKine()
{
	if (type == FRONT_LEG)
		posEst(0) = -cos(hip->pEst) * upperLength - cos(hip->pEst + knee->pEst) * lowerLength + baseLength;
	else
		posEst(0) = -cos(hip->pEst) * upperLength - cos(hip->pEst + knee->pEst) * lowerLength - baseLength;
	float L = -sin(hip->pEst) * upperLength - sin(hip->pEst + knee->pEst);
	posEst(1) = sin(abad->pEst) * L;
	posEst(2) = cos(abad->pEst) * L;
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