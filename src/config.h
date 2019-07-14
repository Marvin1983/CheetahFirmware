#ifndef CONFIG_H
#define CONFIG_H

//------------------------------------------------------------------------------
// debug

//#define DEBUG_LEG
#define DEBUG_MOTOR

//------------------------------------------------------------------------------
/// motorController Limits ///
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

//legController
#define FRONT_LEG 0
#define BACK_LEG 1
#define RX_LEN 66
#define TX_LEN 66

//------------------------------------------------------------------------------
#endif