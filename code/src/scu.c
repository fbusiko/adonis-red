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
#include <string.h>
#include <drivers/uart.h>
#include <usb/usb_device.h>
#include <ctype.h>
#include <logging/log_ctrl.h>
#include <power/reboot.h>
#include <shell/shell_uart.h>
#include <device.h>
#include <drivers/i2c.h>

/*** Macros **************************************************/
#define START_DELAY    500
#define WRITE_DELAY    50
#define GPS_ADDRESS    0x10
#define MAX_PACKET_SIZE 255

/*** Globals *************************************************/
static int i2c_gps_read(const struct device *i2c);
static int i2c_gps_init_pps(const struct device *i2c);
uint8_t gpsData[255];
uint8_t head;
uint8_t tail;

void main(void) {

    //const struct device *spi;
    const struct device *usb;
    const struct device *i2c;
	//struct spi_config spi_cfg = {0};
	//struct spi_cs_control spi_cs;
    head = 0;

    /* Initialise SPI, USB and I2C */
    usb = device_get_binding(CONFIG_UART_CONSOLE_ON_DEV_NAME);
    if (usb == NULL) {
        return;
    }

    i2c = device_get_binding("I2C_3");
	if (!i2c) {

		printk("I2C: Device driver not found.\n");
		return;
	}

    /* Initialise the log and enable the USB */
    log_init();
    if (usb_enable(NULL)) {
        return;
    }

    /* Sleep for some time to allow the USB client to connect before 
			outputting to the log */
    k_msleep(START_DELAY);

    /* Firstly initialise pps LED */
    if (i2c_gps_init_pps(i2c) != 0) {
        
        printk("Issue writing to the GPS module\n");
        return;
    }

    k_msleep(10);

    while (1) {

        i2c_gps_read(i2c);
        k_msleep(10);
            
        for (int i = 0; i < MAX_PACKET_SIZE; i++) {
            printk("%d", gpsData[i]);
        }
        printk("\n");
        k_msleep(1000);
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
    uint8_t gps_read_address = (GPS_ADDRESS << 1) | 1;

	/* GPS address */
	wr_addr[0] = 0x00;
	wr_addr[1] = (GPS_ADDRESS << 1) | 1;

	/* Setup I2C messages */

	/* Send the address to write to */
	msgs[0].buf = &gps_read_address;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Data to be written, and STOP after this. */
	msgs[1].buf = gpsData;
	msgs[1].len = sizeof(gpsData);
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(i2c, &msgs[0], 2, GPS_ADDRESS);
}

static int i2c_gps_init_pps(const struct device *i2c) {

    char* ppsCommand = "$PMTK285,2,100*3E\r\n";

    uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

    uint8_t data[strlen(ppsCommand)];
    for (int i = 0; i < strlen(ppsCommand); i++) {

        data[i] = data[i] * 10 - (ppsCommand[i] - 48);
    }

    uint8_t gps_write_address = GPS_ADDRESS << 1;

	/* FRAM address */
	wr_addr[0] = 0x00;
	wr_addr[1] = GPS_ADDRESS << 1;

	/* Setup I2C messages */

	/* Send the address to write to */
	msgs[0].buf = &gps_write_address;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Data to be written, and STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = sizeof(data);
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(i2c, &msgs[0], 2, GPS_ADDRESS);
}