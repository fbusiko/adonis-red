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

/* RFM Module Settings */
#define MODE_SLEEP              0x00
#define MODE_LORA               0x80
#define MODE_STDBY              0x01
#define MODE_TX                 0x83
#define TRANSMIT_DIRECTION_UP   0x00

/* RFM Registers */
#define REG_PA_CONFIG           0x09
#define REG_PREAMBLE_MSB        0x20
#define REG_PREAMBLE_LSB        0x21
#define REG_FRF_MSB             0x06
#define REG_FRF_MID             0x07
#define REG_FRF_LSB             0x08
#define REG_FEI_LSB             0x1E
#define REG_FEI_MSB             0x1D
#define REG_MODEM_CONFIG        0x26
#define REG_PAYLOAD_LENGTH      0x22
#define REG_FIFO_POINTER        0x0D
#define REG_FIFO_BASE_ADDR      0x80
#define REG_OPERATING_MODE      0x01
#define REG_VERSION             0x42
#define REG_PREAMBLE_DETECT     0x1F
#define REG_TIMER1_COEF         0x39
#define REG_NODE_ADDR           0x33
#define REG_IMAGE_CAL           0x3B
#define REG_RSSI_CONFIG         0x0E
#define REG_RSSI_COLLISION      0x0F
#define REG_DIO_MAPPING_1       0x40

#define RFM_RESET     DT_ALIAS(pa4)

#define RFM_RESET_GPIO_LABEL	DT_GPIO_LABEL(RFM_RESET, gpios)
#define RFM_RESET_GPIO_PIN	DT_GPIO_PIN(RFM_RESET, gpios)

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

/*** Globals *************************************************/
static int i2c_gps_read(const struct device *i2c);
static int i2c_gps_init_pps(const struct device *i2c);
static int rfm_read(uint8_t addr);
char** string_split(char* a_str, const char a_delim);
uint8_t gpsData[255];
uint8_t rfmBuffer[2];
uint8_t spiWriteAddress[1];
uint8_t head;
uint8_t tail;
const struct device *spi;
struct spi_config spi_cfg;
struct spi_cs_control spi_cs;

struct spi_buf tx_buf = {
    .buf = spiWriteAddress,
    .len = sizeof(spiWriteAddress)
};

struct spi_buf_set tx = {
    .buffers = &tx_buf,
    .count = 1
};

struct spi_buf rx_buf = {
    .buf = rfmBuffer,
    .len = sizeof(rfmBuffer)
};

struct spi_buf_set rx = {
    .buffers = &rx_buf,
    .count = 1
};

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

    i2c = device_get_binding("I2C_1");
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
	spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER;
	spi_cfg.frequency = 4000000;
    spi_cfg.slave = 0;

    /* Initialise the log and enable the USB */
    log_init();
    if (usb_enable(NULL)) {
        return;
    }

    /* Sleep for some time to allow the USB client to connect before 
			outputting to the log */
    k_msleep(START_DELAY);

    // int ret = 0;


    /* Initialise and configure GPIO pin for RFM reset */
    rst = device_get_binding(RFM_RESET_GPIO_LABEL);
    if (rst == NULL) {

        printk("Error: failed to find reset pin\n");
        return;
    }

    ret = gpio_pin_configure(rst, 4, (GPIO_OUTPUT_ACTIVE));
    k_msleep(100);
    if (ret != 0) {

        printk("Error: failed to configure reset pin\n");
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
	// err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	err = bt_le_scan_start(&scan_param, scan_cb);
	if (err) {
		printk("Starting scanning failed (err %d)\n", err);
		return;
	}

    /*printk("EEEEEEEEEEEEEEEEEEEEEEEEEEn\r\n");

    gpio_pin_set(rst, RFM_RESET_GPIO_PIN, 0);
    k_msleep(0.1); // 1 ms
    gpio_pin_set(rst, RFM_RESET_GPIO_PIN, 1);
    k_msleep(5);
    //gpio_pin_set(rst, RFM_RESET_GPIO_PIN, 0);*/

    /* Firstly initialise pps LED */
    /*if (i2c_gps_init_pps(i2c) != 0) {

        printk("Issue writing to the GPS module\n");
        return;
    }*/

    //k_msleep(10);
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

        char *a, *b;

        char* wordArray[255];
        int amountOfWords = 0;

        int longitude = 0;
        int latitude = 0;

        for(a=strtok_r(tempData,",", &b) ; a!=NULL ; a=strtok_r(NULL,",", &b) ) {

                //printf("%s ",a);
                wordArray[amountOfWords++] = a;
        }

        for (int i = 0; i < amountOfWords; i++) {

            // Focus on GPS data
            if (i == 0 && (wordArray[i] != "$GNGGA" || wordArray[i] != "$GPGGA" ||
                wordArray[i] != "$GPRMC")) {

                break;
            }

            if (wordArray[i] == "N" || wordArray[i] == "S") {
                
            }
        }
        printf("\n");
        k_msleep(1500);

        // node_info[2] = longitude;
        // node_info[4] = latitude;

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
		
		// k_sleep(K_MSEC(1000));

		gpio_pin_set(dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;
    }

        /*ret = rfm_read(REG_VERSION);
        //k_msleep(10);

        if (ret == 0) {

            printk("%d %d\n", rfmBuffer[0], rfmBuffer[1]);
        } else {

            printk("SPI not working\n");
            printk("%d %d\n", rfmBuffer[0], rfmBuffer[1]);
        }*/
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
  * Reads into an SPI, and returns what was written to the
  * rfmData buffer
  */
static int rfm_read(uint8_t addr) {

    /** Strip out top bit to set 0 value (read) */
    spiWriteAddress[0] = addr & 0x7F;
    spi_write(spi, &spi_cfg, &tx);
    
    int start = k_cycle_get_32();
    int finish;
    while (1) {

        finish = k_cycle_get_32();
        if (finish - start >= 16) {
            break;
        }
    }

    return spi_read(spi, &spi_cfg, &rx);
    //spi_transceive(spi, &spi_cfg, &tx, &rx);
    //return spi_transceive(spi, &spi_cfg, &tx, &rx);
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