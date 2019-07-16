#include <Arduino.h>
#include <config.h>
#include <ChRt.h>
#include <FlexCAN.h>
#include <Eigen.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include "legController.h"

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
		wakeTime += MS2ST(1000);
		chThdSleepUntil(wakeTime);

		digitalWrite(LED_BUILTIN, HIGH);

		wakeTime += MS2ST(500);
		chThdSleepUntil(wakeTime);

		digitalWrite(LED_BUILTIN, LOW);
	}
}

//------------------------------------------------------------------------------
// chSetup thread. Begins the program threads.
// Continue setup() after chBegin().
void chSetup()
{
	// Checks to make sure you enabled cooperature scheduling
	if (CH_CFG_TIME_QUANTUM)
	{
		Serial.println("You must set CH_CFG_TIME_QUANTUM zero in");
		Serial.print("src/arm/chconf_arm.h");
		Serial.println(F(" to enable cooperative scheduling."));
		while (true)
			;
	}
	// Create ALL the threads!!
	// This is the most important part of the setup

	// Blink thread: blinks the onboard LED
	chThdCreateStatic(waBlinkThread, sizeof(waBlinkThread), NORMALPRIO,
					  BlinkThread, NULL);
}
void setup()
{

	Serial.begin(SERIAL_BAUD);
	// Wait for USB Serial.
	while (!Serial)
		;
	// Start ChibiOS.
	chBegin(chSetup);
}

void loop()
{
	chThdSleepMilliseconds(10000);
}