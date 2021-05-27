/**
***************************************************************
* @file     code_src_scu.c
* @author	Fletcher Busiko - 44788179
* @date		26032021
* @brief	B-L475E-IOT01A Main Code (which incldues SPI
*           and I2C)
*       REFERENCE: CSSE4011 Project.pdf
***************************************************************
*      EXTERNAL FUNCTIONS
***************************************************************
* main - main code
***************************************************************
*/

/*** Includes ************************************************/
#include <errno.h>
#include <drivers/spi.h>
#include <zephyr.h>
#include <shell/shell.h>
#include <version.h>
#include <logging/log.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <drivers/uart.h>
#include <usb/usb_device.h>
#include <ctype.h>
#include <logging/log_ctrl.h>
#include <power/reboot.h>
#include <shell/shell_uart.h>
#include <device.h>
#include <drivers/i2c.h>

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

/*** Macros **************************************************/
#define START_DELAY    500
#define WRITE_DELAY    50
#define GPS_ADDRESS    0x10
#define MAX_PACKET_SIZE 255

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

/*** Globals *************************************************/
static int i2c_gps_read(const struct device *i2c);
static int i2c_gps_init_pps(const struct device *i2c);
static int rfm_read(uint8_t addr);
char** string_split(char* a_str, const char a_delim);
uint8_t gpsData[255];
uint8_t head;
uint8_t tail;
const struct device *spi;
struct spi_config spi_cfg;
struct spi_cs_control spi_cs;
//uint8_t spiData[510];

void main(void) {

    const struct device *usb;
    const struct device *i2c;
    const struct device *rst;
    const struct device *dev;
    head = 0;
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

    /* Initialise SPI, USB and I2C */
    usb = device_get_binding(CONFIG_UART_CONSOLE_ON_DEV_NAME);
    if (usb == NULL) {
        return;
    }

    i2c = device_get_binding("I2C_0");
	if (!i2c) {

		printk("I2C: Device driver not found.\n");
		return;
	}

    spi = device_get_binding("SPI_1");
	if (!spi) {
		printk("Could not find SPI driver\n");
		return;
	}

    /* Initialise SPI gpio pins */
    spi_cs.gpio_dev = device_get_binding("GPIO_A");
    spi_cs.gpio_pin = 15;
    spi_cs.gpio_dt_flags = GPIO_ACTIVE_LOW;
    spi_cs.delay = 0;

    /* Configure SPI */
    spi_cfg.cs = &spi_cs;
	spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_SLAVE;
	spi_cfg.frequency = 2000000;
    spi_cfg.slave = 0;

    struct spi_buf tx_buf = {
        .buf = gpsData,
        .len = sizeof(gpsData)
    };

    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };

    /* Initialise the log and enable the USB */
    // log_init();
    if (usb_enable(NULL)) {
        return;
    }

    /* Sleep for some time to allow the USB client to connect before 
			outputting to the log */
    k_msleep(START_DELAY);

    /* Firstly initialise pps LED */
    /*if (i2c_gps_init_pps(i2c) != 0) {

        printk("Issue writing to the GPS module\n");
        return;
    }*/

    while (1) {

        if (i2c_read(i2c, gpsData, sizeof(gpsData), GPS_ADDRESS) != 0) {

            printk("issue reading from GPS\n");
        } else {
            k_msleep(2);
        }

        int head = 0;
        char tempData[255];
        for (int i = 0; i < 255; i++) {

                
            if (gpsData[i] != 0x0A) {
                    
                tempData[head++] = (char) gpsData[i];
            }
        }

        printk("%s\n", tempData);

        char *a, *b;
        int amountOfWords = 0;

        uint16_t longitude = 0;
        uint16_t latitude = 0;

        uint16_t lastWord = 0;

        for(a=strtok_r(tempData,",", &b) ; a!=NULL ; a=strtok_r(NULL,",", &b)) {

            //printk("%s ",a);
            if (strcmp(a, "N") == 0 || strcmp(a, "S") == 0) {
                //printk("Long\r\n");
            }

            if(strcmp(a, "E") == 0 || strcmp(a, "W") == 0) {
                //printk("Lat\r\n");
            }
            lastWord = (uint16_t) a;
        }
        //printk("\n");

        spi_write(spi, &spi_cfg, &tx);

        k_msleep(1500);

		gpio_pin_set(dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;
    }
}

/**
  * Reads to the GPS via I2C. Returns true if correctly read, otherwise false
  * Returns true for read, and false for write
  */
static int i2c_gps_read(const struct device* i2c) {

    /*uint8_t incoming;
    for (int i = 0; i < MAX_PACKET_SIZE; i++) {
        if (i2c_reg_read_byte(i2c, (GPS_ADDRESS << 1) | 0x01, GPS_ADDRESS, &incoming) == 0) {

            k_msleep(2);
            if (incoming != 0x0A) {

                // Record this byte
                gpsData[head] = incoming;
                head++;
                head %= MAX_PACKET_SIZE;
            }
        } else {

            return;
        }
    }*/

    uint8_t wr_addr[2];
	struct i2c_msg msgs[2];
    uint8_t gps_read_address = GPS_ADDRESS << 1 | 0x01;

	/* GPS address */
	wr_addr[0] = gps_read_address;
	wr_addr[1] = 0x00;

	/* Setup I2C messages */

	/* Send the address to write to */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Data to be written, and STOP after this. */
	msgs[1].buf = gpsData;
	msgs[1].len = MAX_PACKET_SIZE;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(i2c, &msgs[0], 2, GPS_ADDRESS);
}

static int i2c_gps_init_pps(const struct device *i2c) {

    char* ppsCommand = "$PMTK353,1,0,0,0,0*2A\r\n";

    uint8_t wr_addr[2];
	struct i2c_msg msgs[2];
    uint8_t gps_write_address = GPS_ADDRESS << 1;

	/* FRAM address */
	wr_addr[0] = gps_write_address;
	wr_addr[1] = 0x00;

	/* Setup I2C messages */

	/* Send the address to write to */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Data to be written, and STOP after this. */
	msgs[1].buf = (uint8_t *) ppsCommand;
	msgs[1].len = strlen(ppsCommand);
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(i2c, &msgs[0], 2, GPS_ADDRESS);
}

/**
  * Function used to sort an array by a delimiter
  */
char** string_split(char* a_str, const char a_delim) {

    char** result    = 0;
    size_t count     = 1;
    char* tmp        = a_str;
    char* last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;

    /* Count how many elements will be extracted. */
    for (int i = 0; i < strlen(a_str); i++) {

        if (a_delim == a_str[i]) {

            count++;
        }
    }

    result = malloc(sizeof(char*) * count);

    char* tempString = a_str;

    for (int i = 0; i < count; i++) {
        
        for (int j = 0; j < a_str; i++) {

            if (a_delim == a_str[j]) {

                strncpy(result[count], a_str, j);
                int len = strlen(a_str);
                memmove(a_str, a_str+j + 1, len - j);
                break;
            }
        }
    }

    return result;
}