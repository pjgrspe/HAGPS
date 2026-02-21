#include "i2c_gnss.h"
#include "driver/i2c.h"

void i2c_gnss_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

int i2c_gnss_read(uint8_t *data, size_t max_len) {
    return i2c_master_read_from_device(I2C_MASTER_NUM, ZEDF9P_ADDR, data, max_len, 100 / portTICK_PERIOD_MS);
}
