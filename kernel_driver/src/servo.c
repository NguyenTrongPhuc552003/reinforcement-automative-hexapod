#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include "servo.h"
#include "pca9685.h"

/* Servo Configuration */
#define NUM_LEGS             6
#define JOINTS_PER_LEG       3
#define TOTAL_SERVOS         (NUM_LEGS * JOINTS_PER_LEG)

/* Channel mapping for servos */
static const u8 servo_channels[NUM_LEGS][JOINTS_PER_LEG] = {
    {0, 1, 2},    /* Leg 0: channels 0-2  (Group 0) */
    {3, 4, 5},    /* Leg 1: channels 3-5  (Group 1) */
    {6, 7, 8},    /* Leg 2: channels 6-8  (Group 2) */
    {9, 10, 11},  /* Leg 3: channels 9-11 (Group 2-3) */
    {12, 13, 14}, /* Leg 4: channels 12-14 (Group 3) */
    {15, 0, 1}    /* Leg 5: channels 15,0,1 (Group 3-0) */
};

/* Servo calibration values */
static const struct {
    s16 min_angle;
    s16 max_angle;
    s16 center_offset;
} servo_cal[TOTAL_SERVOS] = {
    /* Leg 0 */
    {-90, 90, 0}, {-90, 90, 0}, {-90, 90, 0},
    /* Leg 1 */
    {-90, 90, 0}, {-90, 90, 0}, {-90, 90, 0},
    /* Leg 2 */
    {-90, 90, 0}, {-90, 90, 0}, {-90, 90, 0},
    /* Leg 3 */
    {-90, 90, 0}, {-90, 90, 0}, {-90, 90, 0},
    /* Leg 4 */
    {-90, 90, 0}, {-90, 90, 0}, {-90, 90, 0},
    /* Leg 5 */
    {-90, 90, 0}, {-90, 90, 0}, {-90, 90, 0}
};

/* Initialize servo subsystem */
int servo_init(void)
{
    int ret;
    
    pr_info("Servo: Initializing servo subsystem\n");

    /* Initialize PCA9685 */
    ret = pca9685_init();
    if (ret < 0) {
        pr_err("Servo: Failed to initialize PCA9685 (ret=%d)\n", ret);
        return ret;
    }

    /* Give some time for PCA9685 to stabilize */
    msleep(100);

    /* Center all servos one by one */
    ret = leg_center_all();
    if (ret < 0) {
        pr_err("Servo: Failed to center servos (ret=%d)\n", ret);
        pca9685_cleanup();
        return ret;
    }

    pr_info("Servo: Initialization complete\n");
    return 0;
}

/* Get channel number for a servo */
static u8 get_servo_channel(u8 leg, u8 joint)
{
    if (leg >= NUM_LEGS || joint >= JOINTS_PER_LEG) {
        pr_err("Servo: Invalid leg %d or joint %d\n", leg, joint);
        return 0xFF;
    }
    return servo_channels[leg][joint];
}

/* Set servo angle with calibration */
int servo_set_angle(u8 channel, s16 angle)
{
    s16 cal_angle;
    u16 pulse_width;
    int ret;
    
    if (channel >= TOTAL_SERVOS) {
        pr_err("Servo: Invalid channel %d\n", channel);
        return -EINVAL;
    }
    
    /* Apply calibration */
    if (angle < servo_cal[channel].min_angle)
        angle = servo_cal[channel].min_angle;
    if (angle > servo_cal[channel].max_angle)
        angle = servo_cal[channel].max_angle;
    
    cal_angle = angle + servo_cal[channel].center_offset;
    
    /* Convert angle to pulse width (500-2500us) */
    pulse_width = PWM_CENTER_TIME + 
                 ((cal_angle * (PWM_MAX_TIME - PWM_MIN_TIME)) / 180);
    
    ret = pca9685_set_pwm_ms(channel, pulse_width);
    if (ret < 0) {
        pr_err("Servo: Failed to set channel %d to angle %d (ret=%d)\n",
               channel, angle, ret);
        return ret;
    }
    
    return 0;
}

/* Center a servo */
int servo_center(u8 channel)
{
    if (channel >= TOTAL_SERVOS) {
        pr_err("Servo: Invalid channel %d\n", channel);
        return -EINVAL;
    }

    return servo_set_angle(channel, 0);
}

/* Set leg position */
int leg_set_position(u8 leg_num, s16 hip_angle, s16 knee_angle, s16 ankle_angle)
{
    u8 hip_channel, knee_channel, ankle_channel;
    int ret;
    
    if (leg_num >= NUM_LEGS) {
        pr_err("Servo: Invalid leg number %d\n", leg_num);
        return -EINVAL;
    }
    
    hip_channel = get_servo_channel(leg_num, 0);
    knee_channel = get_servo_channel(leg_num, 1);
    ankle_channel = get_servo_channel(leg_num, 2);
    
    if (hip_channel == 0xFF || knee_channel == 0xFF || ankle_channel == 0xFF)
        return -EINVAL;
    
    ret = servo_set_angle(hip_channel, hip_angle);
    if (ret < 0)
        return ret;
        
    ret = servo_set_angle(knee_channel, knee_angle);
    if (ret < 0)
        return ret;
        
    ret = servo_set_angle(ankle_channel, ankle_angle);
    if (ret < 0)
        return ret;
        
    return 0;
}

/* Center all servos */
int leg_center_all(void)
{
    int leg, joint;
    u8 channel;
    int ret;
    
    for (leg = 0; leg < NUM_LEGS; leg++) {
        for (joint = 0; joint < JOINTS_PER_LEG; joint++) {
            channel = get_servo_channel(leg, joint);
            if (channel == 0xFF)
                continue;
                
            ret = servo_center(channel);
            if (ret < 0) {
                pr_err("Servo: Failed to center leg %d joint %d (ret=%d)\n",
                       leg, joint, ret);
                return ret;
            }
            msleep(50); /* Small delay between servos */
        }
    }
    
    return 0;
}

/* Cleanup servo subsystem */
void servo_cleanup(void)
{
    leg_center_all();
    msleep(500); /* Wait for servos to center */
    pca9685_cleanup();
}

EXPORT_SYMBOL_GPL(servo_init);
EXPORT_SYMBOL_GPL(servo_cleanup);
EXPORT_SYMBOL_GPL(servo_set_angle);
EXPORT_SYMBOL_GPL(servo_center);
EXPORT_SYMBOL_GPL(leg_set_position);
EXPORT_SYMBOL_GPL(leg_center_all);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Servo Driver for Hexapod Robot");
MODULE_VERSION("1.0");
