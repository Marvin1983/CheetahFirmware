#ifndef CONFIG_H
#define CONFIG_H

///main///
#define SERIAL_BAUD 230400

///debug///

//#define DEBUG_LEG
//#define DEBUG_MOTOR
#define DEBUG_THREAD

/// motorController///
#define P_MIN -12.5f //between -4*pi and 4*pi
#define P_MAX 12.5f
#define V_MIN -45.0f // between -30 and + 30 rad/s
#define V_MAX 45.0f
#define T_MIN -18.0f //between 0 and 500 N-m/rad
#define T_MAX 18.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

///legController///

#define CAN_0 0
#define CAN_1 1

#define FEET_MODE 0  //feet mode for swiping
#define FORCE_MODE 1 //force mode for MPC
#define MOTOR_MODE 2 //control each motor for some sperical situation

#define FL_LEG_ID 0
#define FR_LEG_ID 1
#define BL_LEG_ID 2
#define BR_LEG_ID 3

struct motorPDGain_t
{
	float kpAbad;
	float kdAbad;
	float kpHip;
	float kdHip;
	float kpKnee;
	float kdKnee;
};
struct motorInitPos_t
{
	float abad;
	float hip;
	float knee;
};
struct motorCANID_t
{
	uint32_t abad;
	uint32_t hip;
	uint32_t knee;
};
struct feetPDGain_t
{
	float kpLeg;
	float kdLeg;
};
struct legLength_t
{
	float baseOffset0;
	float baseOffset1;
	float upperLength;
	float lowerLength;
};

extern struct motorCANID_t motorCANID[4];
extern struct motorPDGain_t motorGain[4];
extern struct motorInitPos_t motorInitPos[4];
extern struct feetPDGain_t feetGain[4];
extern struct legLength_t legLength[4];
extern int legCANPort[4];

///Thread///
#define F_LEG_THREAD 1000 //Hz
#define P_LEG_THREAD 5	//Priority
#endif