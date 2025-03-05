#ifndef _HEXAPOD_H_
#define _HEXAPOD_H_

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include "protocol.h"

/* Device structure */
struct hexapod_dev
{
    struct i2c_client *mpu6050; /* MPU6050 I2C client */
    struct mutex lock;          /* Device mutex */
    bool initialized;           /* Initialization flag */
};

/* Core functions */
int hexapod_get_imu_data(struct imu_data *data);
int hexapod_set_leg_position(u8 leg, struct leg_position *position);

#endif /* _HEXAPOD_H_ */
