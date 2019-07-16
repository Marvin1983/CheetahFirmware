#include <Arduino.h>
#include "config.h"

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
int legCANPort[4] =
	//FL, FR, BL,BR
	{0, 0, 1, 1};

///Thread///

typedef enum priorityThread
{
	legThread = 5,
	gaitThread = 4,
	imuThread = 3,
	logThread = 2,
	ledThread = 0,

} priorityThread;