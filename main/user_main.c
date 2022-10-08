#include "user_main.h"

static const char *TAG = "main";

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
static const uint16_t config_bits = 0b1100001010000011;

/**
 * @brief Read buffer for reading uint16_t from ADS1115
 */
static uint8_t i2c_ads1115_read_buffer[2] = { 0, 0 };

/**
* fixed point approximation of 3.3/65535 in a 2:30 fixed point
*/
static const uint32_t MAX_VOLTAGE = 54068;

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
    *data         = ((uint16_t)i2c_ads1115_read_buffer[1] << 8)
            | ((uint16_t)i2c_ads1115_read_buffer[0]);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t
i2c_ads1115_init(i2c_port_t i2c_num)
{
    vTaskDelay(100 / portTICK_RATE_MS);
    i2c_init();
    esp_err_t ret = i2c_ads1115_write_config(i2c_num, config_bits);
    return ret;
}

static void
i2c_ads1115_task(void *arg)
{
    ESP_ERROR_CHECK(i2c_ads1115_init(I2C_ADS1115_MASTER_NUM));

    ESP_ERROR_CHECK(i2c_ads1115_select_read_register(I2C_ADS1115_MASTER_NUM,
                                                     ADS1115_CONVERSION_REG));

    esp_err_t ret;
    uint16_t  sensor_reading;
    uint32_t voltage_reading;
    while (1)
    {
        memset(i2c_ads1115_read_buffer, 0, 2);
        ret = i2c_ads1115_read_uint16_t(I2C_ADS1115_MASTER_NUM,
                                        &sensor_reading);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG,
                     "Read did not complete successfully. Error: %s",
                     esp_err_to_name(ret));
            continue;
        }
        voltage_reading = ((uint32_t)sensor_reading) * MAX_VOLTAGE;
        ESP_LOGI(TAG, "ADC Reading: %u.%u V (RAW VALUE: %u)", (voltage_reading >> 30), (voltage_reading & 0xC0000000), sensor_reading);

        vTaskDelay(100 / portTICK_RATE_MS);
    }

    i2c_driver_delete(I2C_ADS1115_MASTER_NUM);
}

void
app_main(void)
{
    // start i2c task
    xTaskCreate(i2c_ads1115_task, "i2c_ads1115_task", 2048, NULL, 10, NULL);
}