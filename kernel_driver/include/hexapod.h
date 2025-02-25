#ifndef _HEXAPOD_KERNEL_H_
#define _HEXAPOD_KERNEL_H_

#include <linux/types.h>
#include "protocol.h"

/* Driver functions */
int hexapod_init_hardware(void);
void hexapod_cleanup_hardware(void);
int hexapod_set_servo_position(u8 leg, u8 joint, s32 angle);
int hexapod_read_imu(struct imu_data *data);

#endif /* _HEXAPOD_KERNEL_H_ */
