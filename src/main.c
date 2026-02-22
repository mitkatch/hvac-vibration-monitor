/*
 * HVAC Vibration Monitor - Main Entry Point
 * 
 * Minimal main file - handles only system initialization and event loop.
 * All FSM logic has been moved to fsm.c
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "fsm.h"
#include "ble.h"
#include "sensor.h"
#include "led.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ──────────────────────────────────────────────
 * System Initialization
 * ────────────────────────────────────────────── */
static int system_init(void)
{
	int err;

	LOG_INF("=== HVAC Vibration Monitor ===");
	LOG_INF("FSM Architecture v1.0");

	/* Initialize LED (for pairing indication) */
	err = led_init();
	if (err) {
		LOG_WRN("LED init failed (err %d), continuing without LED", err);
		/* Non-fatal error - system can work without LED */
	}

	/* Initialize BLE (registers event_post as callback sink) */
	err = ble_init(event_post);
	if (err) {
		LOG_ERR("BLE init failed (err %d)", err);
		return err;
	}

	/* Initialize vibration sensor */
	err = sensor_init_vibration();
	if (err) {
		LOG_ERR("ADXL343 init failed (err %d)", err);
		return err;
	}

	/* Initialize environmental sensor (optional) */
	bool env_sensor_present = false;
	err = sensor_init_environment();
	if (err == 0) {
		env_sensor_present = true;
		LOG_INF("Environmental sensor: present");
	} else {
		LOG_INF("Environmental sensor: not found (continuing without)");
	}

	/* Initialize FSM with sensor configuration */
	fsm_init(env_sensor_present);

	return 0;
}

/* ──────────────────────────────────────────────
 * Main Entry Point
 *
 * Init hardware, then run the event loop forever.
 * The loop blocks on k_msgq_get() — sleeping when
 * there's nothing to process (low power).
 * ────────────────────────────────────────────── */
int main(void)
{
	int err = system_init();

	/* Post init result to the FSM */
	fsm_event_t init_evt;
	if (err == 0) {
		init_evt.type = EVT_INIT_COMPLETE;
	} else {
		init_evt.type = EVT_INIT_FAILED;
		init_evt.data.error_code = err;
	}
	event_post(&init_evt);

	/* ── Event loop (runs forever) ────────── */
	fsm_event_t evt;
	struct k_msgq *event_queue = fsm_get_event_queue();

	while (1) {
		/* Block until an event arrives */
		k_msgq_get(event_queue, &evt, K_FOREVER);

		/* Process event through FSM */
		fsm_handle_event(&evt);
	}

	return 0; /* never reached */
}
