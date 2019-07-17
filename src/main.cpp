#include <Arduino.h>
#include <config.h>
#include <ChRt.h>
#include <FlexCAN.h>
#include <Eigen.h>
#include <Eigen/Core>
#include <Eigen/LU>
//#include "legController.h"
#include "trajectory.h"
//------------------------------------------------------------------------------
// BlinkThread: Blink the built-in led at 1Hz so you know if the Teensy is on.

// 64 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waBlinkThread, 64);

static THD_FUNCTION(BlinkThread, arg)
{
	(void)arg;
	pinMode(LED_BUILTIN, OUTPUT);
	systime_t wakeTime = chVTGetSystemTimeX(); // T0
	while (true)
	{
		digitalWrite(LED_BUILTIN, HIGH);
		wakeTime += MS2ST(500);
		chThdSleepUntil(wakeTime);
		digitalWrite(LED_BUILTIN, LOW);
		wakeTime += MS2ST(500);
		chThdSleepUntil(wakeTime);
	}
}

//------------------------------------------------------------------------------
// chSetup thread. Begins the program threads.
// Continue setup() after chBegin().
void chSetup()
{
	// Create ALL the threads!!
	// This is the most important part of the setup

	// Blink thread: blinks the onboard LED
	chThdCreateStatic(waBlinkThread, sizeof(waBlinkThread), NORMALPRIO, BlinkThread, NULL);
	chThdCreateStatic(waLegThread, sizeof(waLegThread), NORMALPRIO + P_LEG_THREAD, legThread, NULL);
	chThdCreateStatic(waTrajThread, sizeof(waTrajThread), NORMALPRIO + P_TRAJ_THREAD, trajThread, NULL);
}

#ifdef DEBUG_THREAD
void printUnusedStack()
{
	Serial.print(F("Unused Stack: "));
	Serial.print(chUnusedThreadStack(waBlinkThread, sizeof(waBlinkThread)));
	Serial.print(' ');
	Serial.print(chUnusedThreadStack(waLegThread, sizeof(waLegThread)));
	Serial.print(' ');
	Serial.print(chUnusedMainStack());
	Serial.println();
}
#endif

void setup()
{

	Serial.begin(SERIAL_BAUD);
	// Wait for USB Serial.

	while (!Serial)
		;
	Serial.println("lalala");
	// Start ChibiOS.
	chBegin(chSetup);
	while (1)
		;
}

void loop()
{
	chThdSleepMilliseconds(500);
#ifdef DEBUG_THREAD
	printUnusedStack();
	Serial.println(counter);
#endif
}
