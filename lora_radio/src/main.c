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
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/addr.h>

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

struct bt_le_scan_param scan_param = {
	.type       = BT_HCI_LE_SCAN_PASSIVE,
	.options    = BT_LE_SCAN_OPT_NONE,
	.interval   = 0x0010,
	.window     = 0x0010,
};

#define MOBILE1 "020:cED:B8:6C:6A:0B (random)"

static void print_data(uint8_t *data, uint8_t len) {

	// printk("data 0x%02x", data[20]);
	// for (int i = 0; i < len; i++) {
	// 	printk("0x%02x-", data[i]);
		
	// }
	printk("#%03d-%03d-%03d-%03d-%03d-%03d-%03d-%03d-%03d-%03d-%03d-%03d\r\n",
		 data[19], data[20], data[21], data[22], data[23], data[24], data[25],
		 data[26], data[27], data[28], data[29], data[30]);
	
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	uint8_t buffer[BT_ADDR_LE_STR_LEN];

	// bt_addr_le_to_str(addr, buffer, BT_ADDR_LE_STR_LEN);
	// for (int i = 0; i < BT_ADDR_LE_STR_LEN; i++) {
	// 	printk("%c", buffer[i]);
		
	// }
	// for (int i = 0; i < buf->len; i++) {
	// 	printk("0x%02x-", buf->data[i]);
		
	// }
	// printk("Rssi\r\n");
	// printk("%d", strcmp(buffer, MOBILE1));
	// if (strcmp(buffer, MOBILE1) == 0) {

	// 	// for (int i = 0; i < buf->len; i++) {
	// 	// 	printk("0x%02x-", buf->data[i]);
		
	// 	// }
	// 	print_data(buf->data, buf->len);
	// 	// printk("Mobile1\r\n")
	// }
	if (buf->data[17] == 0x01 && buf->data[18] == 0x03 && buf->data[16] == 0xff) {
		// bt_le_whitelist_add(addr);
		
		print_data(buf->data, buf->len);
	}
}

void main(void) {

	const struct device *dev;
	bool led_is_on = true;
	int ret;

	dev = device_get_binding(LED0);
	if (dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}
		
	int err;

	printk("Starting Scanner/Advertiser Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = bt_le_scan_start(&scan_param, scan_cb);
	if (err) {
		printk("Starting scanning failed (err %d)\n", err);
		return;
	}

	const struct device *dev2 = device_get_binding(
			CONFIG_UART_CONSOLE_ON_DEV_NAME);
	uint32_t dtr = 0;

	if (usb_enable(NULL)) {
		return;
	}

	if (strlen(CONFIG_UART_CONSOLE_ON_DEV_NAME) !=
	    strlen("CDC_ACM_0") ||
	    strncmp(CONFIG_UART_CONSOLE_ON_DEV_NAME, "CDC_ACM_0",
		    strlen(CONFIG_UART_CONSOLE_ON_DEV_NAME))) {
		printk("Error: Console device name is not USB ACM\n");

		return;
	}

	while(1) {

		k_sleep(K_MSEC(1000));
		gpio_pin_set(dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;
	}
}
