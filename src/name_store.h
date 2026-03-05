/*
 * name_store.h — Persistent sensor name storage
 *
 * Stores a custom BLE advertising name in flash via Zephyr settings.
 * Falls back to CONFIG_BT_DEVICE_NAME (build default) when no
 * custom name is stored or when the name is cleared.
 *
 * Constraints enforced on the board side (safety net):
 *   - Max length: NAME_STORE_MAX_LEN (20 chars)
 *   - ASCII printable only (0x20–0x7E)
 *   - Empty string → revert to build default
 */

#ifndef NAME_STORE_H
#define NAME_STORE_H

#include <stdbool.h>
#include <stdint.h>

/* Max custom name length (bytes, not including null terminator).
 * Scan response budget: 31 - 2 (length+type) = 29 usable.
 * We cap at 20 to leave margin.                                */
#define NAME_STORE_MAX_LEN  20

/**
 * @brief Initialize name store and load saved name from flash.
 *
 * Must be called after settings_subsys_init() and settings_load().
 * If no name is stored, the build default is used.
 *
 * @return 0 on success, negative errno on failure
 */
int name_store_init(void);

/**
 * @brief Get the current active name (custom or default).
 *
 * Always returns a valid null-terminated string.
 *
 * @return Pointer to the current name (do not free)
 */
const char *name_store_get(void);

/**
 * @brief Set a new custom name and persist to flash.
 *
 * Validates the name:
 *   - NULL or empty string → reverts to build default
 *   - Truncated to NAME_STORE_MAX_LEN if too long
 *   - Non-ASCII printable characters are rejected
 *
 * @param name  New name string (null-terminated)
 * @return 0 on success
 *         -EINVAL if name contains non-ASCII characters
 *         negative errno on flash write failure
 */
int name_store_set(const char *name);

/**
 * @brief Clear custom name and revert to build default.
 *
 * @return 0 on success, negative errno on failure
 */
int name_store_clear(void);

/**
 * @brief Check if a custom name is active (vs build default).
 *
 * @return true if using a custom name
 */
bool name_store_is_custom(void);

#endif /* NAME_STORE_H */
