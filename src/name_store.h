/*
 * name_store.h — Sensor name (hardcoded for v1)
 *
 * Name is set at compile time via CONFIG_HVAC_SENSOR_NAME in prj.conf.
 * A CoAP command channel will replace this in a future release.
 */

#ifndef NAME_STORE_H
#define NAME_STORE_H

#define NAME_STORE_MAX_LEN  20

/**
 * @brief Initialize name store (logs the compiled-in name).
 * @return 0 always
 */
int name_store_init(void);

/**
 * @brief Get the sensor name.
 * @return Pointer to null-terminated name string (do not free)
 */
const char *name_store_get(void);

#endif /* NAME_STORE_H */
