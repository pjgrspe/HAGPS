#ifndef I2C_GNSS_H
#define I2C_GNSS_H

#include <stdint.h>
#include <stddef.h>

#define ZEDF9P_ADDR 0x42
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000

void i2c_gnss_init(void);
int i2c_gnss_read(uint8_t *data, size_t max_len);

#endif // I2C_GNSS_H
