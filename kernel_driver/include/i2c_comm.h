#ifndef I2C_COMM_H
#define I2C_COMM_H

#include <linux/i2c.h>
#include <linux/types.h>

// Initialize I2C device
int i2c_init_device(struct i2c_adapter *adapter, u8 addr);

// Write data to I2C device
int i2c_write_data(u8 addr, u8 reg, u8 *data, size_t len);

// Read data from I2C device
int i2c_read_data(u8 addr, u8 reg, u8 *data, size_t len);

// Cleanup I2C resources
void i2c_cleanup(void);

#endif /* I2C_COMM_H */
