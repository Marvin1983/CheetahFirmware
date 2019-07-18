#include "trajectory.h"

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
	float sigma = (2.0f * PI) / (swipingDuty * stepPeriod);
	chMtxLock(&legDesDataMutex);
	if (time <= stepPeriod * swipingDuty)
	{
		legCon->pDes(0) = delta(0) / (2.0f * PI) * (sigma * time - sin(sigma * time)) + start(0);
		legCon->pDes(1) = delta(1) / (2.0f * PI) * (sigma * time - sin(sigma * time)) + start(1);
		legCon->pDes(2) = FEET_HEIGHT / 2.0f * (1.0f - cos(sigma * time)) + start(2);
		legCon->vDes(0) = delta(0) / (swipingDuty * stepPeriod) * (1 - cos(sigma * time));
		legCon->vDes(1) = delta(1) / (swipingDuty * stepPeriod) * (1 - cos(sigma * time));
		legCon->vDes(2) = FEET_HEIGHT / 2.0f * (1.0f - cos(sigma * time));
	}
	else //Still no the ground
	{
		//not change x y postion
		legCon->vDes(0) = 0;
		legCon->vDes(1) = 0;
		//not change z speed
		legCon->pDes(2) += 1.0f / (float)F_TRAJ_THREAD * legCon->vDes(2);
	}
	chMtxUnlock(&legDesDataMutex);
}
void trajectory::touchDown()
{
	legCon->vDes(0) = 0;
	legCon->vDes(1) = 0;
	legCon->vDes(2) = 0;
	isTouchDown = true;
}
trajectory trajFL(&controlFL);
trajectory trajFR(&controlFR);
trajectory trajBL(&controlBL);
trajectory trajBR(&controlBR);

THD_WORKING_AREA(waTrajThread, 512);

THD_FUNCTION(trajThread, arg)
{
	(void)arg;
	systime_t wakeTime = chVTGetSystemTimeX(); //
	while (true)
	{
		trajFL.updateDes();
		trajFR.updateDes();
		trajBL.updateDes();
		trajBR.updateDes();
#ifdef DEBUG_TRAJ
		//counter++;
#endif
		wakeTime += MS2ST((uint32_t)1000 / F_TRAJ_THREAD);
		chThdSleepUntil(wakeTime);
	}
}