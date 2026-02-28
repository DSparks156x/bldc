#include "gtbms.h"
#include "ch.h"
#include "hal.h"
#include "comm_can.h"
#include "utils.h"
#include "mc_interface.h"

// Settings
#define GTBMS_THREAD_STACK_SIZE 1024

// Variables
static volatile bool gtbms_running = false;
static THD_WORKING_AREA(gtbms_thread_wa, GTBMS_THREAD_STACK_SIZE);
static THD_FUNCTION(gtbms_thread, arg);

void gtbms_init(void) {
	if (!gtbms_running) {
		chThdCreateStatic(gtbms_thread_wa, sizeof(gtbms_thread_wa),
				NORMALPRIO, gtbms_thread, NULL);
		gtbms_running = true;
	}
}

bool gtbms_is_running(void) {
	return gtbms_running;
}

static THD_FUNCTION(gtbms_thread, arg) {
	(void)arg;
	chRegSetThreadName("GT BMS Comms");

	for(;;) {
		// BMS Communication logic goes here
		chThdSleepMilliseconds(100);
	}
}
