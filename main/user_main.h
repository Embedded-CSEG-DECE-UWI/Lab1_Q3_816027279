/* I2C example
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 *
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C module by running two tasks on i2c
 * bus:
 *
 * - read external i2c sensor, here we use a ADS1115 sensor for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave
 * mode) on one ESP8266 chip.
 *
 * Pin assignment:
 *
 * - master:
 *    GPIO0 is assigned as the data signal of i2c master port
 *    GPIO2 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect sda/scl of sensor with GPIO0/GPIO2
 * - no need to add external pull-up resistors, driver will enable internal
 * pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.*/

#ifndef USER_MAIN_H
#define USER_MAIN_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"

#include "driver/i2c.h"

#define I2C_ADS1115_MASTER_SCL_IO         2 /* gpio number for I2C master clock */
#define I2C_ADS1115_MASTER_SDA_IO         0 /* gpio number for I2C master data */
#define I2C_ADS1115_MASTER_NUM            I2C_NUM_0 /* I2C port number for master dev */
#define I2C_ADS1115_MASTER_TX_BUF_DISABLE 0         /* Disable the I2C buffer */
#define I2C_ADS1115_MASTER_RX_BUF_DISABLE 0         /* Disable the I2C buffer */

#define ADS1115_SENSOR_ADDR 0x48 /* slave address for ADS1115 sensor */
#define WRITE_BIT           I2C_MASTER_WRITE /* I2C master write */
#define READ_BIT            I2C_MASTER_READ  /* I2C master read */
#define ACK_CHECK_EN        0x1 /* I2C master will check ack from slave */

/*
 * Register addresses for the ADS1115
 */
typedef enum
{
    ADS1115_CONVERSION_REG = 0x0,
    ADS1115_CONFIG_REG     = 0x1,
    ADS1115_LO_THRESH_REG  = 0x2,
    ADS1115_HI_THRESH_REG  = 0x3,
} ads1115_register_t;

/**
 * @brief convert a uint16_t containing a two's complement number to a int16_t
 *
 * @param num The two's complement numebr
 *
 * @return the int16_t representation of num
 */
static int16_t i2c_ads1115_u16_to_i16(uint16_t num);

/**
 * @brief Initialize the i2c interface
 */
static esp_err_t i2c_init();

/**
 * @brief set the config register of the ads1115
 * Data is written in the following order:
 *  - Start
 *  - ADS1115 I2C Address + Write bit
 *  - Config Register Address
 *  - OS | MUX2 | MUX1 | MUX0 | PGA2 | PGA1 | PGA0 | MODE
 *  - DR2 | DR1 | DR0 | COMP_MODE | COMP_POL | COMP_LAT | COMP_QUE1 | COMP_QUE0
 *  - Stop
 *
 * @param i2c_num I2C port number
 * @param data to write to the config register
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t i2c_ads1115_write_config(i2c_port_t i2c_num, uint16_t data);

/**
 * @brief select a register of the ads1115 for reading
 * Data is written in the following order:
 *  - Start
 *  - ADS1115 I2C Address + Read Bit
 *  - Register Address
 *  - Stop
 *
 * @param i2c_num I2C port number
 * @param Register to read from
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t i2c_ads1115_select_read_register(i2c_port_t         i2c_num,
                                                  ads1115_register_t reg);

/**
 * @brief read 16 bits from the ads1115. The register must be selected BEFORE
 * this command is called. Write commands may change the selected register, and
 * it must be reselected after all write commands. Data is written/read in the
 * following order:
 *  - Start
 *  - Write ADS1115 I2C Address + Read Bit
 *  - Read MSB of register
 *  - Read LSB of register
 *  - Stop
 *
 * @param i2c_num I2C port number
 * @param data that is read from the ads1115
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */

static esp_err_t i2c_ads1115_read_uint16_t(i2c_port_t i2c_num, uint16_t *data);

/**
* @brief Initialize the ADS1115 using I2C
* 
* @param i2c_num The i2c port for reading from and writing to
* 
* @return result of initializing the ads1115
*/
static esp_err_t i2c_ads1115_init(i2c_port_t i2c_num);

/**
* @brief Set up the ads1115 and read a single ended adc reading from A0 on the ADS1115
* 
* @param arg Context pointer passed by xTaskCreate (pvParameters)
*/
static void i2c_ads1115_task(void *arg);

// @brief Launch the i2c task then exit
void app_main(void);

#endif
