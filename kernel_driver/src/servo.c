#include <linux/module.h>
#include "../include/servo.h"
#include "../include/pca9685.h"

#define SERVO_FREQ 50  // 50Hz for standard servos
#define MIN_PULSE_WIDTH 600   // 0.6ms
#define MAX_PULSE_WIDTH 2400  // 2.4ms
#define DEFAULT_PULSE_WIDTH 1500  // 1.5ms (center position)
#define SERVO_I2C_ADDR 0x40
#define MAX_SERVOS 18

struct servo_device {
    u8 pca9685_addr;
    u8 channel;
    bool is_initialized;
    unsigned int min_angle;
    unsigned int max_angle;
    unsigned int current_angle;
};

static struct servo_device servos[MAX_SERVOS];
static int num_servos = 0;

int servo_init(struct servo_data *servo) {
    int ret;

    if (!servo)
        return -EINVAL;

    ret = pca9685_device_init(SERVO_I2C_ADDR);
    if (ret < 0)
        return ret;

    servo->is_initialized = true;
    return 0;
}

int servo_init_channel(u8 i2c_addr, u8 channel, u8 min_angle, u8 max_angle) {
    int ret;

    if (num_servos >= MAX_SERVOS)
        return -ENOMEM;

    ret = pca9685_device_init(i2c_addr);
    if (ret < 0)
        return ret;

    // Set initial position to min_angle
    ret = pca9685_set_pwm(i2c_addr, channel, 0, min_angle);
    if (ret < 0)
        return ret;

    servos[num_servos].channel = channel;
    servos[num_servos].is_initialized = true;
    num_servos++;

    return 0;
}

int servo_set_angle(unsigned int servo_id, unsigned int angle) {
    unsigned int pulse_width;
    u16 off_time;

    if (servo_id >= num_servos || !servos[servo_id].is_initialized)
        return -EINVAL;

    if (angle < servos[servo_id].min_angle || 
        angle > servos[servo_id].max_angle)
        return -EINVAL;

    // Convert angle to pulse width
    pulse_width = MIN_PULSE_WIDTH + 
                 (angle * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180);

    // Convert pulse width to PCA9685 off-time value
    // 4096 steps per cycle, 20ms cycle time (50Hz)
    off_time = pulse_width * 4096 / 20000;

    servos[servo_id].current_angle = angle;

    return pca9685_set_pwm(servos[servo_id].pca9685_addr,
                          servos[servo_id].channel,
                          0, off_time);
}

void servo_cleanup(void) {
    int i;

    // Set all initialized servos to their rest position
    for (i = 0; i < num_servos; i++) {
        if (servos[i].is_initialized) {
            pca9685_set_pwm(SERVO_I2C_ADDR, servos[i].channel, 0, 0);
            servos[i].is_initialized = false;
        }
    }
    num_servos = 0;
}

EXPORT_SYMBOL_GPL(servo_init);
EXPORT_SYMBOL_GPL(servo_init_channel);
EXPORT_SYMBOL_GPL(servo_cleanup);

MODULE_LICENSE("GPL"); 