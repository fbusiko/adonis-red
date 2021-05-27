/**
 ***************************************************************
 * @file apps/p1/src/main.c
 * @author Nicholas - s4537166
 * @date 01032021
 * @brief practical 1 main file
 ***************************************************************
 * EXTERNAL FUNCTIONS
 ***************************************************************
 * 
 ***************************************************************
*/
// #include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif

#define LEN 10

static uint8_t node_info[LEN] = {
	0x01, 
	0x03,
	0x00, 
	0x00, 
	0x00, 
	0x00, 
	0x00, 
	0x00,
	0x00, 
	0x00
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, node_info, LEN)
};

/* Set Scan Response data */
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf) 
{

	if (buf->data[18] == 0x01 && buf->data[19] == 0x02) {

		if (buf->data[20] == 0x01) {

			node_info[4] = rssi;
		}
		if (buf->data[20] == 0x02) {

			node_info[5] = rssi;
		}
		if (buf->data[20] == 0x03) {

			node_info[6] = rssi;
		}
		if (buf->data[20] == 0x04) {

			node_info[7] = rssi;
		}
	}
}

void main(void) {

	const struct device *dev;
	bool led_is_on = false;
	int ret;

	dev = device_get_binding(LED0);
	if (dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}
	
	struct bt_le_scan_param scan_param = {
		.type       = BT_HCI_LE_SCAN_PASSIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = 0x0010,
		.window     = 0x0010,
	};
	int err;

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	err = bt_le_scan_start(&scan_param, scan_cb);
	if (err) {
		printk("Starting scanning failed (err %d)\n", err);
		return;
	}

	while(1) {

		/* Start advertising */
		// && node_2_status && node_3_status && node_4_status

		err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad),
					NULL, 0);
		if (err) {
			printk("Advertising failed to start (err %d)\n", err);
			return;
		}

		k_sleep(K_MSEC(400));

		err = bt_le_adv_stop();
		if (err) {
			printk("Advertising failed to stop (err %d)\n", err);
			return;
		}
		
		k_sleep(K_MSEC(1000));

		gpio_pin_set(dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;
	}
}
