#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "../include/pwm_control.h"
#include "../include/i2c_comm.h"
#include "../include/pca9685.h"

#define MAX_SERVOS 18
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_FREQ 50  // 50Hz for servos
#define SERVO_MIN_PULSE 150  // 0.6ms (150/4096 * 20ms)
#define SERVO_MAX_PULSE 600  // 2.4ms (600/4096 * 20ms)

struct servo_info {
    u8 i2c_addr;      // I2C address of PCA9685
    u8 channel;        // Channel on PCA9685 (0-15)
    u16 current_pulse; // Current pulse width
    bool is_active;
};

static struct servo_info servos[MAX_SERVOS];
static int num_servos = 0;

// PCA9685 initialization
int pca9685_init(u8 i2c_addr) {
    u8 prescale;
    int ret;

    prescale = 25000000 / (4096 * PCA9685_FREQ) - 1;

    // Call the renamed function
    ret = pca9685_device_init(i2c_addr);
    if (ret < 0)
        return ret;
    
    // Set frequency (50Hz)
    ret = i2c_write_data(i2c_addr, PCA9685_PRESCALE, &prescale, 1);
    if (ret < 0)
        return ret;
    
    // Normal mode
    ret = i2c_write_data(i2c_addr, PCA9685_MODE1, (u8[]){0x20}, 1);  // Auto increment enabled
    if (ret < 0)
        return ret;
    
    msleep(1);  // Wait for oscillator
    return 0;
}

int pwm_init_servo(u8 i2c_addr, u8 channel) {
    bool need_init;
    int i;
    int ret;

    need_init = true;

    // Check if servo already initialized
    for (i = 0; i < num_servos; i++) {
        if (servos[i].i2c_addr == i2c_addr && 
            servos[i].channel == channel) {
            need_init = false;
            break;
        }
    }

    if (need_init) {
        if (num_servos >= MAX_SERVOS)
            return -ENOMEM;

        ret = pca9685_init(i2c_addr);
        if (ret < 0)
            return ret;

        servos[num_servos].i2c_addr = i2c_addr;
        servos[num_servos].channel = channel;
        servos[num_servos].current_pulse = SERVO_MIN_PULSE;
        servos[num_servos].is_active = true;
        num_servos++;
    }

    return 0;
}

int pwm_set_servo_angle(u8 servo_id, u8 angle) {
    u16 pulse;
    int ret;

    if (servo_id >= num_servos || !servos[servo_id].is_active)
        return -EINVAL;

    // Calculate pulse width
    pulse = SERVO_MIN_PULSE + (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180);

    // Set PWM values using updated function signature
    ret = pca9685_set_pwm(servos[servo_id].i2c_addr, 
                         servos[servo_id].channel,
                         0,    // on_time
                         pulse // off_time
                         );
    return ret;
}

void pwm_cleanup(void) {
    int i;

    for (i = 0; i < num_servos; i++) {
        if (servos[i].is_active) {
            // Set PWM to 0
            pwm_set_servo_angle(i, 0);
            servos[i].is_active = false;
        }
    }
    num_servos = 0;
}
