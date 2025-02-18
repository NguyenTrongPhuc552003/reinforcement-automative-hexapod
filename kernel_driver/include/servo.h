#ifndef SERVO_H
#define SERVO_H

#include <linux/types.h>

struct servo_data {
    u8 channel;
    bool is_initialized;
};

#define MAX_SERVOS 18  // Total number of servos in hexapod

// Initialize a servo
int servo_init(struct servo_data *servo);

// Set servo angle
int servo_set_angle(unsigned int servo_id, unsigned int angle);

// Cleanup all servos
void servo_cleanup(void);

// Add new function for specific initialization
int servo_init_channel(u8 i2c_addr, u8 channel, u8 min_angle, u8 max_angle);

#endif /* SERVO_H */ 