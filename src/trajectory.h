#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include "legController.h"

class trajectory
{
private:
	float time;
	float stepPeriod;
	float swipingDuty;
	bool isTouchDown;
	Vector3f end;
	Vector3f start;
	Vector3f delta;

	legController *leg;

public:
	trajectory(legController *leg);
	~trajectory();
	void upDateStartAndSetEnd(Vector3f end);
	void updateDes();
	void touchDown();
};

extern THD_WORKING_AREA(waTrajThread, 512);

extern THD_FUNCTION(trajThread, arg);

#endif
