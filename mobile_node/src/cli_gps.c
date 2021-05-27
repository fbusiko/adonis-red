/**
 ***************************************************************
 * @file myoslib/src/cli_i2c.c
 * @author Nicholas - s4537166
 * @date 15032021
 * @brief LED Light Bar peripheral driver
 ***************************************************************
 * EXTERNAL FUNCTIONS
 ***************************************************************
 * hal_ledbar_init () - intialise LED bar
 * hal_ledbar_set () - set LED bar value
 ***************************************************************
*/

#include <zephyr.h>
#include <shell/shell.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <stdlib.h>


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
/**
 * @brief Shell command that reads and writes any i2c register
 * 
 * This shell command will read and write to any i2c register using the HCI myoslib driver.
 * Only works with single bytes.
 */
static int cmd_i2creg(const struct shell *shell, size_t argc, char **argv) {

	const struct device *led;
	bool led_is_on = false;
	int ret;
	shell_print(shell, "test");

	led = device_get_binding(LED0);
	if (led == NULL) {
		return;
	}

	ret = gpio_pin_configure(led, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}

	while(1) {

		gpio_pin_set(led, PIN, (int)led_is_on);
		led_is_on = !led_is_on;

	}

	return 0;
}

SHELL_CMD_ARG_REGISTER(i2creg, NULL, "Read/Write Sensor Regestors", cmd_i2creg, 4, 1);