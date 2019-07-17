#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <ChRt.h>
#include <Eigen.h>
#include "legController.h"
#include "config.h"

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

	legController *legCon;

public:
	trajectory(legController *leg);
	~trajectory();
	void upDateStartAndSetEnd(Vector3f end);
	void updateDes();
	void touchDown();
};

#endif
