/*
 * BLE Implementation for Vibration Data - NCS 3.2.1 Compatible
 */

#include "ble.h"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/printk.h>

/*
 * Custom UUIDs for Vibration Monitoring
 * Base: 12345678-1234-5678-1234-56789abcdef0
 * 
 * Note: UUIDs are in little-endian format (reversed bytes)
 * UUID: 12345678-1234-5678-1234-56789abcdef0
 * Bytes: f0 de bc 9a 78 56 34 12 - 78 56 - 34 12 - 78 56 34 12
 */

/* Service UUID: 12345678-1234-5678-1234-56789abcdef0 */
#define BT_UUID_VIBRATION_SERVICE \
	BT_UUID_DECLARE_128( \
		0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, \
		0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12)

/* Characteristic UUID for burst data: ...def1 */
#define BT_UUID_BURST_DATA \
	BT_UUID_DECLARE_128( \
		0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, \
		0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12)

/* Characteristic UUID for metadata: ...def2 */
#define BT_UUID_BURST_METADATA \
	BT_UUID_DECLARE_128( \
		0xf2, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, \
		0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12)

/* Connection handle */
static struct bt_conn *current_conn = NULL;

/* Metadata structure */
typedef struct {
	uint32_t timestamp;
	uint16_t sample_rate;
	uint16_t sample_count;
	uint8_t sensor_id;
	uint8_t reserved[3];
} __packed burst_metadata_t;

static burst_metadata_t burst_metadata;

/* GATT CCC changed callback */
static void burst_data_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
	printk("BLE: Burst data notifications %s\n", 
	       notif_enabled ? "enabled" : "disabled");
}

static ssize_t read_metadata(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
	const burst_metadata_t *meta = attr->user_data;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, meta, sizeof(*meta));
}

/* GATT Service definition */
BT_GATT_SERVICE_DEFINE(vibration_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_VIBRATION_SERVICE),
	
	BT_GATT_CHARACTERISTIC(BT_UUID_BURST_DATA,
	                      BT_GATT_CHRC_NOTIFY,
	                      BT_GATT_PERM_NONE,
	                      NULL, NULL, NULL),
	BT_GATT_CCC(burst_data_ccc_changed,
	            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	
	BT_GATT_CHARACTERISTIC(BT_UUID_BURST_METADATA,
	                      BT_GATT_CHRC_READ,
	                      BT_GATT_PERM_READ,
	                      read_metadata, NULL, &burst_metadata)
);

/* Device name for advertising */
#define DEVICE_NAME "HVAC-Vibe"  // Shorter name (max 8 chars safer for old BLE)
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* Advertising data - raw UUID bytes */
static uint8_t svc_uuid[16] = {
	0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_UUID128_ALL, svc_uuid, 16),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),  // Add name here too
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* Connection callbacks */
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	
	if (err) {
		printk("BLE: Connection failed (err 0x%02x)\n", err);
		return;
	}
	
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("BLE: Connected to %s\n", addr);
	
	current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;
	int retry_count = 0;
	const int max_retries = 3;
	
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("BLE: Disconnected from %s (reason 0x%02x)\n", addr, reason);
	
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	
	/* Wait a moment before restarting advertising */
	printk("BLE: Waiting before restarting advertising...\n");
	k_sleep(K_MSEC(500));
	
	/* Retry advertising if it fails */
	while (retry_count < max_retries) {
		err = ble_start_advertising();
		
		if (err == 0) {
			printk("BLE: Successfully restarted advertising\n");
			return;
		}
		
		retry_count++;
		printk("BLE: Advertising failed (attempt %d/%d), retrying...\n", 
		       retry_count, max_retries);
		k_sleep(K_MSEC(1000));
	}
	
	printk("BLE: ERROR - Failed to restart advertising after %d attempts!\n", 
	       max_retries);
	printk("BLE: Press RESET button to recover\n");
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/**
 * Initialize BLE
 */
int ble_init(void)
{
	int err;
	
	printk("\n=== BLE Initialization ===\n");
	
	err = bt_enable(NULL);
	if (err) {
		printk("ERROR: BLE enable failed (err %d)\n", err);
		return err;
	}
	
	printk("BLE stack enabled\n");
	
	err = ble_start_advertising();
	if (err) {
		printk("ERROR: Advertising start failed (err %d)\n", err);
		return err;
	}
	
	printk("=== BLE Ready ===\n\n");
	
	return 0;
}

/**
 * Start advertising
 */
/**
 * Start advertising
 */
/**
 * Start advertising
 */
int ble_start_advertising(void)
{
	int err;
	
	/* Try to stop advertising (ignore errors if not advertising) */
	err = bt_le_adv_stop();
	if (err && err != -EALREADY) {
		printk("BLE: Warning - stop advertising returned %d\n", err);
	}
	
	/* Small delay to let BLE stack settle */
	k_msleep(100);
	
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0,
		.secondary_max_skip = 0,
		.options = BT_LE_ADV_OPT_CONN,
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
		.peer = NULL,
	};

	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), 
	                     sd, ARRAY_SIZE(sd));
	if (err) {
		printk("BLE: ERROR - Advertising start failed (err %d)\n", err);
		return err;
	}

	printk("BLE: Advertising started (device name: %s)\n", DEVICE_NAME);

	return 0;
}

/**
 * Stop advertising
 */
int ble_stop_advertising(void)
{
	int err;
	
	err = bt_le_adv_stop();
	if (err) {
		printk("ERROR: Stop advertising failed (err %d)\n", err);
		return err;
	}
	
	printk("BLE: Advertising stopped\n");
	
	return 0;
}

/**
 * Check connection status
 */
bool ble_is_connected(void)
{
	return (current_conn != NULL);
}

/**
 * Transmit burst data
 */
int ble_transmit_burst(const accel_sample_t *buffer, uint16_t count)
{
	int err;
	
	if (!ble_is_connected()) {
		printk("BLE: Not connected, cannot transmit\n");
		return -ENOTCONN;
	}
	
	burst_metadata.timestamp = k_uptime_get_32();
	burst_metadata.sample_rate = SAMPLE_RATE_HZ;
	burst_metadata.sample_count = count;
	burst_metadata.sensor_id = 1;
	
	printk("BLE: Transmitting burst (%d samples, %d bytes)...\n",
	       count, count * sizeof(accel_sample_t));
	
	const uint16_t max_payload = 244;
	const uint16_t total_bytes = count * sizeof(accel_sample_t);
	const uint16_t num_packets = (total_bytes + max_payload - 1) / max_payload;
	
	printk("BLE: Splitting into %d packets\n", num_packets);
	
	uint16_t offset = 0;
	for (uint16_t i = 0; i < num_packets; i++) {
		uint16_t bytes_remaining = total_bytes - offset;
		uint16_t chunk_size = (bytes_remaining > max_payload) ? 
		                      max_payload : bytes_remaining;
		
		err = bt_gatt_notify(current_conn, 
		                   &vibration_svc.attrs[1],
		                   (uint8_t *)buffer + offset,
		                   chunk_size);
		
		if (err) {
			printk("BLE: Notify failed on packet %d (err %d)\n", i, err);
			return err;
		}
		
		offset += chunk_size;
		
		if ((i + 1) % 5 == 0 || i == num_packets - 1) {
			printk("  Sent %d/%d packets\n", i + 1, num_packets);
		}
		
		k_msleep(10);
	}
	
	printk("BLE: Burst transmitted successfully\n");
	
	return 0;
}

/**
 * Get status string
 */
const char *ble_get_status(void)
{
	if (current_conn) {
		return "Connected";
	} else {
		return "Advertising";
	}
}