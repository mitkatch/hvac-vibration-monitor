/*
 * led.h — LED Control for Pairing Indication
 *
 * Simple LED control for visual feedback during BLE pairing.
 * Supports blinking (during pairing wait) and solid on/off.
 */

#ifndef LED_H
#define LED_H

#include <stdint.h>

/**
 * @brief Initialize LED subsystem
 * 
 * Configures the LED GPIO pin for output.
 * Must be called before any other LED functions.
 * 
 * @return 0 on success, negative errno on failure
 */
int led_init(void);

/**
 * @brief Start LED blinking
 * 
 * Starts a periodic timer that toggles the LED on and off.
 * Use this to indicate pairing is in progress and user needs
 * to enter PIN code.
 * 
 * @param period_ms Blink period in milliseconds
 *                  Example: 500 = LED toggles every 500ms (1Hz blink)
 *                           250 = LED toggles every 250ms (2Hz blink)
 */
void led_start_blinking(uint32_t period_ms);

/**
 * @brief Stop LED blinking and turn LED off
 * 
 * Stops the blink timer and ensures LED is in off state.
 * Use this when pairing completes, fails, or times out.
 */
void led_stop_blinking(void);

/**
 * @brief Turn LED on (solid, no blinking)
 * 
 * Stops any active blinking and sets LED to solid on.
 */
void led_on(void);

/**
 * @brief Turn LED off
 * 
 * Stops any active blinking and sets LED to off.
 */
void led_off(void);

#endif /* LED_H */
