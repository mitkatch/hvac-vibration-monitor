/*
 * name_store.c — Persistent sensor name storage
 *
 * Uses Zephyr's settings subsystem (same flash partition as BLE bonds)
 * to store a custom advertising name under key "hvac/name".
 *
 * On boot:
 *   1. settings_load() is called by ble.c (already in your code)
 *   2. The settings handler fires and populates active_name[]
 *   3. If no key is found, active_name stays as CONFIG_BT_DEVICE_NAME
 *
 * On name change:
 *   1. Validate (ASCII, length)
 *   2. Write to active_name[]
 *   3. Persist with settings_save_one()
 *   4. Caller (ble.c) restarts advertising with new name
 */

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <ctype.h>

#include "name_store.h"

LOG_MODULE_REGISTER(name_store, LOG_LEVEL_INF);

/* ──────────────────────────────────────────────
 * Internal State
 * ────────────────────────────────────────────── */

/* +1 for null terminator */
static char active_name[NAME_STORE_MAX_LEN + 1];
static bool has_custom_name = false;

/* ──────────────────────────────────────────────
 * Validation
 * ────────────────────────────────────────────── */

/**
 * Check that every character is printable ASCII (0x20–0x7E).
 */
static bool is_valid_ascii(const char *str, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (str[i] < 0x20 || str[i] > 0x7E) {
			return false;
		}
	}
	return true;
}

/* ──────────────────────────────────────────────
 * Zephyr Settings Handler
 *
 * Called during settings_load() for our subtree "hvac".
 * ────────────────────────────────────────────── */

static int name_settings_set(const char *name, size_t len,
			     settings_read_cb read_cb, void *cb_arg)
{
	if (strcmp(name, "name") != 0) {
		return -ENOENT;
	}

	/* Read the stored value */
	char buf[NAME_STORE_MAX_LEN + 1];
	ssize_t read_len = read_cb(cb_arg, buf, sizeof(buf) - 1);

	if (read_len <= 0) {
		/* No data or error — keep default */
		return 0;
	}

	buf[read_len] = '\0';

	/* Validate what we read from flash */
	if (read_len > NAME_STORE_MAX_LEN) {
		LOG_WRN("Stored name too long (%zd), ignoring", read_len);
		return 0;
	}

	if (!is_valid_ascii(buf, read_len)) {
		LOG_WRN("Stored name has invalid chars, ignoring");
		return 0;
	}

	memcpy(active_name, buf, read_len + 1);
	has_custom_name = true;
	LOG_INF("Loaded custom name from flash: \"%s\"", active_name);

	return 0;
}

/* Settings handler struct — handles "hvac/name" */
SETTINGS_STATIC_HANDLER_DEFINE(hvac, "hvac", NULL,
			       name_settings_set, NULL, NULL);

/* ──────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────── */

int name_store_init(void)
{
	/* Set default before settings_load() might overwrite it */
	strncpy(active_name, CONFIG_BT_DEVICE_NAME, NAME_STORE_MAX_LEN);
	active_name[NAME_STORE_MAX_LEN] = '\0';
	has_custom_name = false;

	LOG_INF("Name store initialized (default: \"%s\")", active_name);
	return 0;
}

const char *name_store_get(void)
{
	return active_name;
}

int name_store_set(const char *name)
{
	/* NULL or empty → revert to default */
	if (name == NULL || name[0] == '\0') {
		return name_store_clear();
	}

	size_t len = strlen(name);

	/* Truncate if too long */
	if (len > NAME_STORE_MAX_LEN) {
		LOG_WRN("Name truncated from %zu to %d chars", len, NAME_STORE_MAX_LEN);
		len = NAME_STORE_MAX_LEN;
	}

	/* Validate ASCII */
	if (!is_valid_ascii(name, len)) {
		LOG_ERR("Name rejected: contains non-ASCII characters");
		return -EINVAL;
	}

	/* Update active name */
	memcpy(active_name, name, len);
	active_name[len] = '\0';
	has_custom_name = true;

	/* Persist to flash */
	int err = settings_save_one("hvac/name", active_name, len);
	if (err) {
		LOG_ERR("Failed to save name to flash (err %d)", err);
		return err;
	}

	LOG_INF("Name set: \"%s\" (saved to flash)", active_name);
	return 0;
}

int name_store_clear(void)
{
	/* Revert to build default */
	strncpy(active_name, CONFIG_BT_DEVICE_NAME, NAME_STORE_MAX_LEN);
	active_name[NAME_STORE_MAX_LEN] = '\0';
	has_custom_name = false;

	/* Delete from flash */
	int err = settings_delete("hvac/name");
	if (err) {
		LOG_WRN("Failed to delete name from flash (err %d)", err);
		return err;
	}

	LOG_INF("Name cleared, reverted to default: \"%s\"", active_name);
	return 0;
}

bool name_store_is_custom(void)
{
	return has_custom_name;
}
