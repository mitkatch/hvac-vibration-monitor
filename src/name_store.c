/*
 * name_store.c — Sensor name (hardcoded for v1)
 *
 * BLE NUS naming removed. Name is set at compile time via
 * CONFIG_HVAC_SENSOR_NAME. A CoAP command channel will replace
 * this in a future release.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "name_store.h"

LOG_MODULE_REGISTER(name_store, LOG_LEVEL_INF);

/* ──────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────── */

int name_store_init(void)
{
	LOG_INF("Sensor name: \"%s\"", CONFIG_HVAC_SENSOR_NAME);
	return 0;
}

const char *name_store_get(void)
{
	return CONFIG_HVAC_SENSOR_NAME;
}
