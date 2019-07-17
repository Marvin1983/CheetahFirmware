#include "trajectory.h"
#include "legController.h"
#include "config.h"

#include <Arduino.h>
#include <Eigen.h>
#include <ChRt.h>

trajectory::trajectory(legController *leg)
{
	legCon = leg;
	stepPeriod = 1.0f / (float)F_TRAJ_THREAD;
	swipingDuty = 1.0f - STAND_DUTY;
}

trajectory::~trajectory()
{
}
void trajectory::upDateStartAndSetEnd(Vector3f endPos)
{
	start = legCon->pEst;
	end = endPos;
	delta = end - start;
	isTouchDown = false;
}
void trajectory::updateDes()
{
	if (isTouchDown) //Have touch the ground
		return;
	time += 1.0f / (float)F_TRAJ_THREAD;
	float a = (2.0f * PI) / (swipingDuty * stepPeriod);
	if (time <= stepPeriod * swipingDuty)
	{
		legCon->pDes(0) = delta(0) / (2.0f * PI) * (a * time - sin(a * time)) + start(0);
		legCon->pDes(1) = delta(1) / (2.0f * PI) * (a * time - sin(a * time)) + start(1);
		legCon->pDes(2) = FEET_HEIGHT / 2.0f * (1.0f - cos(a * time)) + start(2);
		legCon->vDes(0) = delta(0) / (swipingDuty * stepPeriod) * (1 - cos(a * time));
		legCon->vDes(1) = delta(1) / (swipingDuty * stepPeriod) * (1 - cos(a * time));
		legCon->vDes(2) = FEET_HEIGHT / 2.0f * (1.0f - cos(a * time));
	}
	else //Still no the ground
	{
		//not change x y postion
		legCon->vDes(0) = 0;
		legCon->vDes(1) = 0;
		//not change z speed
		legCon->pDes(2) += 1.0f / (float)F_TRAJ_THREAD * legCon->vDes(2);
	}
}
void trajectory::touchDown()
{
	legCon->vDes(0) = 0;
	legCon->vDes(1) = 0;
	legCon->vDes(2) = 0;
	isTouchDown = true;
}
