#include "legController.h"
#include "Arduino.h"
#include "config.h"
#include "fastMath.h"
#include <FlexCAN.h>
#include <Eigen.h>
#include <Eigen/Core>
#include <Eigen/LU>

using namespace Eigen;
FlexCAN CANbus0(1000000, 0);
FlexCAN CANbus1(1000000, 1);

struct motorPDGain_t motorGain[] = {
	//	abad 		hip	 		knee
	//p    d	 p     d	 p     d
	{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}, //font left
	{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}, //font right
	{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}, //back left
	{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}  //back right
};
struct motorInitPos_t motorInitPos[] = {
	//abad hip	 knee
	{0.0f, 0.0f, PI / 6.0f}, //font left
	{0.0f, 0.0f, PI / 6.0f}, //font right
	{0.0f, 0.0f, PI / 6.0f}, //back left
	{0.0f, 0.0f, PI / 6.0f}  //back right
};
struct motorCANID_t motorCANID[4]{
	//abad,hip,knee
	{1, 2, 3}, //font left
	{4, 5, 6}, //font right
	{1, 2, 3}, //back left
	{4, 5, 6}  //back right
};
struct feetPDGain_t feetGain[] = {
	{1.0f, 1.0f}, //font left	feet
	{1.0f, 1.0f}, //font right feet
	{1.0f, 1.0f}, //back left feet
	{1.0f, 1.0f}  //back right feet
};
struct legLength_t legLength[] = {
	//baseOffset0, baseOffset1, upperLength,lowerLength
	{0.02f, -0.02f, 0.15f, 0.15f}, //font left
	{0.02f, 0.02f, 0.15f, 0.15f},  //font right
	{-0.02f, 0.02f, 0.15f, 0.15f}, //back left
	{-0.02f, -0.02f, 0.15f, 0.15f} //back right
};

int CANPort[4] =
	//FL, FR, BL,BR
	{0, 0, 1, 1};

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

legController::legController(int legID)
{
	id = legID;
	abad = new jointController(motorCANID[id].abad, motorInitPos[id].abad);
	hip = new jointController(motorCANID[id].hip, motorInitPos[id].hip);
	knee = new jointController(motorCANID[id].knee, motorInitPos[id].knee);
	baseOffset[0] = legLength[id].baseOffset0;
	baseOffset[1] = legLength[id].baseOffset1;
	l1 = legLength[id].upperLength;
	l2 = legLength[id].lowerLength;
	port = CANPort[id];
	rxMsg.len = 6;

	CANInit();
	zeroAll();
}

legController::~legController()
{
	delete abad;
	delete hip;
	delete knee;
}
void legController::CANInit()
{
	if (isCANInit)
		return;
	CANbus0.begin();
	CANbus1.begin();
}

void legController::unpackReply(CAN_message_t msg)
{
	/// unpack ints from can buffer ///
	uint16_t msgId = msg.buf[0];
	if (((id == FL_LEG_ID || id == FR_LEG_ID) && msgId > 4) || ((id == BL_LEG_ID || id == BR_LEG_ID) && msgId < 4))
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
		vEstM(0) = v;
		tEstM(0) = t;
	}
	else if (id == 2 || id == 5)
	{
		hip->pEst = p;
		hip->vEst = v;
		hip->tEst = t;
		vEstM(1) = v;
		tEstM(1) = t;
	}
	else if (id == 3 || id == 6)
	{
		knee->pEst = p;
		knee->vEst = v;
		knee->tEst = t;
		vEstM(2) = v;
		tEstM(2) = t;
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

void legController::changesMode(int legMode)
{
	mode = legMode;
	switch (legMode)
	{
	case FEET_MODE:
		abad->kp = 0;
		abad->kd = 0;
		hip->kp = 0;
		hip->kd = 0;
		knee->kp = 0;
		knee->kd = 0;
		kp = feetGain[id].kpLeg;
		kd = feetGain[id].kdLeg;
		break;
	case FORCE_MODE:
		abad->kp = 0;
		abad->kd = 0;
		hip->kp = 0;
		hip->kd = 0;
		knee->kp = 0;
		knee->kd = 0;
		kp = 0;
		kd = 0;
		break;
	case MOTOR_MODE:
		abad->kp = motorGain[id].kpAbad;
		abad->kd = motorGain[id].kdAbad;
		hip->kp = motorGain[id].kpHip;
		hip->kd = motorGain[id].kdHip;
		knee->kp = motorGain[id].kpKnee;
		knee->kd = motorGain[id].kdKnee;
		break;
	}
}

void legController::updateState() //fowrd kinematic
{

	static float c0 = cos(abad->pEst);
	static float s0 = sin(abad->pEst);
	static float c1 = cos(hip->pEst);
	static float s1 = sin(hip->pEst);

	static float c12 = cos(hip->pEst + knee->pEst);
	static float s12 = sin(hip->pEst + knee->pEst);
	pEst(0) = -c1 * l1 - c12 * l2 + baseOffset[0];
	static float L = -s1 * l1 - s12 * l2;
	pEst(1) = s0 * L + baseOffset[1];
	pEst(2) = c0 * L;
	jacobian << 0, s1 * l1 + s12 * l2, s12 * l2,
		-c0 * (s1 * l1 + s12 * l2 + s0 * baseOffset[0]), -s0 * (c1 * l1 + c12 * l2), -c0 * c12 * l2,
		s0 * (s1 * l1 + s12 * l2), -c0 * (c1 * l1 + c12 * l2), -c0 * c12 * l2;
	if (jacobian.determinant())
		inverseJacobian = jacobian.inverse();
	vEst = jacobian * vEstM;
	fEst = jacobian * tEstM;
}

void legController::control()
{
	switch (mode)
	{
	case FEET_MODE:
		fOut = kp * (pDes - pEst) + kd * (vDes - vEst) + fFF;
		tOut = inverseJacobian * fOut;
		abad->tFF = tOut(0);
		hip->tFF = tOut(1);
		knee->tFF = tOut(2);
		packAll();
		writeAll();
		break;
	case FORCE_MODE:
		fOut = fFF;
		tOut = inverseJacobian * fOut;
		abad->tFF = tOut(0);
		hip->tFF = tOut(1);
		knee->tFF = tOut(2);
		packAll();
		writeAll();
		break;
	}
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