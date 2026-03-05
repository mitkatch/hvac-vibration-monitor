/*
 * led.c — LED Control Implementation
 *
 * Provides visual feedback during BLE pairing using LED blinking.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "led.h"

LOG_MODULE_REGISTER(led, LOG_LEVEL_INF);

/* ──────────────────────────────────────────────
 * LED Hardware Configuration
 *
 * Uses device tree alias 'led0' to find the LED.
 * For nRF52840 DK: LED1 (P0.13) is used by default.
 * ────────────────────────────────────────────── */
#define LED_NODE DT_ALIAS(led0)

#if !DT_NODE_EXISTS(LED_NODE)
#error "led0 alias not found in device tree. Please add it to your board overlay."
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

/* ──────────────────────────────────────────────
 * Blink Timer
 *
 * Periodic timer that toggles LED state.
 * ────────────────────────────────────────────── */
static void led_blink_timer_expiry(struct k_timer *timer);
K_TIMER_DEFINE(led_blink_timer, led_blink_timer_expiry, NULL);

static bool led_is_blinking = false;
static bool led_state = false;

static void led_blink_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	
	if (!led_is_blinking) {
		return;
	}
	
	/* Toggle LED state */
	led_state = !led_state;
	gpio_pin_set_dt(&led, led_state ? 1 : 0);
}

/* ──────────────────────────────────────────────
 * Public API Implementation
 * ────────────────────────────────────────────── */

int led_init(void)
{
	/* Check if LED GPIO port is ready */
	if (!device_is_ready(led.port)) {
		LOG_ERR("LED GPIO device not ready");
		return -ENODEV;
	}

	/* Configure LED pin as output, initially off */
	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure LED GPIO (err %d)", ret);
		return ret;
	}

	LOG_INF("LED initialized (port=%s pin=%d)", 
		led.port->name, led.pin);
	return 0;
}

void led_start_blinking(uint32_t period_ms)
{
	led_is_blinking = true;
	led_state = false;
	
	/* Start periodic blinking timer */
	k_timer_start(&led_blink_timer, K_MSEC(period_ms), K_MSEC(period_ms));
	
	LOG_DBG("LED blinking started (period=%u ms)", period_ms);
}

void led_stop_blinking(void)
{
	led_is_blinking = false;
	k_timer_stop(&led_blink_timer);
	
	/* Turn LED off when stopping blink */
	gpio_pin_set_dt(&led, 0);
	led_state = false;
	
	LOG_DBG("LED blinking stopped");
}

void led_on(void)
{
	/* Stop any active blinking first */
	if (led_is_blinking) {
		led_stop_blinking();
	}
	
	gpio_pin_set_dt(&led, 1);
	led_state = true;
	LOG_DBG("LED on");
}

void led_off(void)
{
	/* Stop any active blinking first */
	if (led_is_blinking) {
		led_stop_blinking();
	}
	
	gpio_pin_set_dt(&led, 0);
	led_state = false;
	LOG_DBG("LED off");
}
