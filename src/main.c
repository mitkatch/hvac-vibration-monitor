/*
 * HVAC Vibration Monitor — Main Entry Point
 *
 * Initialises hardware, Thread stack, and FSM, then runs the event loop.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "fsm.h"
#include "thread.h"
#include "sensor.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static int system_init(void)
{
	int err;

	LOG_INF("=== HVAC Vibration Monitor ===");

	/* Thread stack — must be initialised before FSM starts processing
	 * so EVT_THREAD_CONNECTED can arrive as soon as the network joins. */
	err = thread_init(event_post);
	if (err) {
		LOG_ERR("Thread init failed (%d)", err);
		return err;
	}

	/* Vibration sensor */
	err = sensor_init_vibration();
	if (err) {
		LOG_WRN("ADXL343 init failed (%d), continuing without", err);
	}

	/* Environmental sensor (optional) */
	bool env_present = false;
	err = sensor_init_environment();
	if (err == 0) {
		env_present = true;
		LOG_INF("Environmental sensor: present");
	} else {
		LOG_INF("Environmental sensor: not found");
	}

	/* FSM — must come last so callbacks are registered before EVT_INIT_COMPLETE */
	fsm_init(env_present);

	return 0;
}

int main(void)
{
	int err = system_init();

	fsm_event_t init_evt;
	if (err == 0) {
		init_evt.type = EVT_INIT_COMPLETE;
	} else {
		init_evt.type           = EVT_INIT_FAILED;
		init_evt.data.error_code = err;
	}
	event_post(&init_evt);

	/* Event loop — blocks on empty queue (low power) */
	fsm_event_t evt;
	struct k_msgq *q = fsm_get_event_queue();

	while (1) {
		k_msgq_get(q, &evt, K_FOREVER);
		fsm_handle_event(&evt);
	}

	return 0;
}
