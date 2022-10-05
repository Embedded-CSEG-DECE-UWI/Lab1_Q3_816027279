/* I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

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

static const char *TAG = "main";

/**
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
 * - read the sensor data, if connected.
 */

#define I2C_ADS1115_MASTER_SCL_IO         2 /* gpio number for I2C master clock */
#define I2C_ADS1115_MASTER_SDA_IO         0 /* gpio number for I2C master data */
#define I2C_ADS1115_MASTER_NUM            I2C_NUM_0 /* I2C port number for master dev */
#define I2C_ADS1115_MASTER_TX_BUF_DISABLE 0         /* Disable the I2C buffer */
#define I2C_ADS1115_MASTER_RX_BUF_DISABLE 0         /* Disable the I2C buffer */

#define ADS1115_SENSOR_ADDR 0x48 /* slave address for ADS1115 sensor */
#define WRITE_BIT           I2C_MASTER_WRITE /* I2C master write */
#define READ_BIT            I2C_MASTER_READ  /* I2C master read */
#define ACK_CHECK_EN        0x1 /* I2C master will check ack from slave */

/**
 * Define the ads1115 register address:
 */
typedef enum
{
    ADS1115_CONVERSION_REG = 0x0,
    ADS1115_CONFIG_REG     = 0x1,
    ADS1115_LO_THRESH_REG  = 0x2,
    ADS1115_HI_THRESH_REG  = 0x3,
} ads1115_register_t;

/**
 * @brief Configuration bits for the ADS1115
 * OS:     Begin Conversion
 * MUX:    AINp = A0, AINn = GND
 * PGA:    FS = +-2.048V
 * MODE:   Continuous Conversion
 * DR:     128SPS
 * COMP_*: Disabled Comparator
 * Bits: 1_100_010_0_100_00011
 */
static const uint16_t config_bits = 0xC483;

/**
 * @brief Read buffer for reading uint16_t from ADS1115
 */
static uint8_t i2c_ads1115_read_buffer[2] = { 0, 0 };

static int16_t i2c_u16_to_i16(uint16_t num) {
    int16_t converted;
    if (num > 0x7FFF)
    {
        converted = (int16_t)num;
    } else {
        converted = (int16_t) (num - 0x8000);
        converted = -converted;
    }
    
    return converted;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t
i2c_init()
{
    int          i2c_master_port = I2C_ADS1115_MASTER_NUM;
    i2c_config_t conf;
    conf.mode          = I2C_MODE_MASTER;
    conf.sda_io_num    = I2C_ADS1115_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num    = I2C_ADS1115_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    // 300 ticks, Clock stretch is about 210us, you can make changes according
    // to the actual situation.
    // TODO: Check this
    conf.clk_stretch_tick = 300;
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

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
static esp_err_t
i2c_ads1115_write_config(i2c_port_t i2c_num, uint16_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(
        cmd, ADS1115_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1115_CONFIG_REG, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (data >> 8) & 0x00FF, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data & 0x00FF, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

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
static esp_err_t
i2c_ads1115_select_read_register(i2c_port_t i2c_num, ads1115_register_t reg)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(
        cmd, ADS1115_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

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

static esp_err_t
i2c_ads1115_read_uint16_t(i2c_port_t i2c_num, uint16_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    cmd                  = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(
        cmd, ADS1115_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, i2c_ads1115_read_buffer, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    *data = ((uint16_t)i2c_ads1115_read_buffer[1] << 8) | ((uint16_t)i2c_ads1115_read_buffer[0]);
    i2c_cmd_link_delete(cmd);

    return ret;
}

// TODO: Comment
static esp_err_t
i2c_ads1115_init(i2c_port_t i2c_num)
{
    vTaskDelay(100 / portTICK_RATE_MS);
    i2c_init();
    esp_err_t ret = i2c_ads1115_write_config(i2c_num, config_bits);
    return ret;
}

// TODO: Comment
static void
i2c_ads1115_task(void *arg)
{
    ESP_ERROR_CHECK(i2c_ads1115_init(I2C_ADS1115_MASTER_NUM));

    ESP_ERROR_CHECK(i2c_ads1115_select_read_register(I2C_ADS1115_MASTER_NUM,
                                                     ADS1115_CONVERSION_REG));

    esp_err_t ret;
    uint16_t sensor_reading;
    while (1)
    {
        memset(i2c_ads1115_read_buffer, 0, 2);
        ret = i2c_ads1115_read_uint16_t(I2C_ADS1115_MASTER_NUM, &sensor_reading);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Read did not complete successfully. Error: %s", esp_err_to_name(ret));
            continue;
        }
        
        ESP_LOGI(TAG, "Raw ADC Reading: %d", i2c_u16_to_i16(sensor_reading));
        
        vTaskDelay(100 / portTICK_RATE_MS);
    }

    i2c_driver_delete(I2C_ADS1115_MASTER_NUM);
}

// TODO: Comment
void
app_main(void)
{
    // start i2c task
    xTaskCreate(i2c_ads1115_task, "i2c_ads1115_task", 2048, NULL, 10, NULL);
}