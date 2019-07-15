#ifndef CONFIG_H
#define CONFIG_H

///debug///

//#define DEBUG_LEG
#define DEBUG_MOTOR

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

#define FLLegID 0
#define FRLegID 1
#define BLLegID 2
#define BRLegID 3
#endif