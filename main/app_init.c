#include "app_init.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

#define I2C_SDA_GPIO        21
#define I2C_SCL_GPIO        22
#define I2C_FREQUENCY_HZ    100000

static const char *TAG = "APP_INIT";
static i2c_master_bus_handle_t i2c_bus;


void app_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source  = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));
    
    ESP_LOGI(TAG, "I2C bus initialized");
}

i2c_master_bus_handle_t app_get_i2c_bus(void)
{
    return i2c_bus;
}