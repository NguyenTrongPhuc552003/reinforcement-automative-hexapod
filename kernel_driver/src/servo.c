#include <linux/module.h>
#include <linux/i2c.h>
#include "servo.h"
#include "pca9685.h"

#define SERVO_MIN_PULSE    150  // Min pulse length (out of 4096)
#define SERVO_MAX_PULSE    600  // Max pulse length (out of 4096)
#define SERVO_MIN_ANGLE    0    // Min angle in degrees
#define SERVO_MAX_ANGLE    180  // Max angle in degrees

static struct i2c_adapter *i2c_adapter;
static struct i2c_client *pca9685_clients[NUM_PCA9685];

/* Convert angle to PWM value */
static u16 angle_to_pwm(s16 angle) {
    if (angle < SERVO_MIN_ANGLE)
        angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;
        
    return SERVO_MIN_PULSE + ((angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / SERVO_MAX_ANGLE);
}

/* Initialize servo at given PCA9685 index and channel */
static int init_servo(int pca_idx, u8 channel) {
    int ret;
    
    // Set initial position to 90 degrees
    u16 pwm = angle_to_pwm(90);
    ret = pca9685_set_pwm(pca9685_clients[pca_idx], channel, 0, pwm);
    
    return ret;
}

/* Initialize all servos */
int servo_init(void) {
    int i, ret;
    u8 channel;
    
    // Get I2C adapter
    i2c_adapter = i2c_get_adapter(3);  // Use I2C bus 3
    if (!i2c_adapter)
        return -ENODEV;
        
    // Initialize each PCA9685
    for (i = 0; i < NUM_PCA9685; i++) {
        pca9685_clients[i] = (struct i2c_client *)pca9685_register(i2c_adapter, PCA9685_BASE_ADDR + i);
        if (IS_ERR(pca9685_clients[i])) {
            ret = PTR_ERR(pca9685_clients[i]);
            goto error;
        }
            
        // Initialize all channels
        for (channel = 0; channel < PCA9685_NUM_CHANNELS; channel++) {
            ret = init_servo(i, channel);
            if (ret < 0)
                goto error;
        }
    }
    
    return 0;
    
error:
    servo_cleanup();
    return ret;
}

/* Set servo angle */
int servo_set_angle(u8 leg_id, u8 joint_id, s16 angle) {
    u8 pca9685_idx, channel;
    u16 pwm;
    
    if (leg_id >= NUM_LEGS || joint_id >= JOINTS_PER_LEG)
        return -EINVAL;
        
    // Calculate PCA9685 index and channel
    pca9685_idx = (leg_id * JOINTS_PER_LEG + joint_id) / PCA9685_NUM_CHANNELS;
    channel = (leg_id * JOINTS_PER_LEG + joint_id) % PCA9685_NUM_CHANNELS;
    
    // Convert angle to PWM
    pwm = angle_to_pwm(angle);
    
    // Set PWM
    return pca9685_set_pwm(pca9685_clients[pca9685_idx], channel, 0, pwm);
}

/* Move an entire leg */
int servo_move_leg(u8 leg_id, s16 coxa_angle, s16 femur_angle, s16 tibia_angle) {
    int ret;
    
    if (leg_id >= NUM_LEGS)
        return -EINVAL;
        
    ret = servo_set_angle(leg_id, JOINT_COXA, coxa_angle);
    if (ret < 0)
        return ret;
        
    ret = servo_set_angle(leg_id, JOINT_FEMUR, femur_angle);
    if (ret < 0)
        return ret;
        
    ret = servo_set_angle(leg_id, JOINT_TIBIA, tibia_angle);
    if (ret < 0)
        return ret;
        
    return 0;
}

/* Cleanup */
void servo_cleanup(void) {
    int i;
    
    // Cleanup PCA9685 devices
    for (i = 0; i < NUM_PCA9685; i++) {
        if (pca9685_clients[i] && !IS_ERR(pca9685_clients[i])) {
            i2c_unregister_device(pca9685_clients[i]);
            pca9685_clients[i] = NULL;
        }
    }
    
    // Put I2C adapter
    if (i2c_adapter) {
        i2c_put_adapter(i2c_adapter);
        i2c_adapter = NULL;
    }
}

EXPORT_SYMBOL_GPL(servo_init);
EXPORT_SYMBOL_GPL(servo_cleanup);
EXPORT_SYMBOL_GPL(servo_set_angle);
EXPORT_SYMBOL_GPL(servo_move_leg);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phuc Nguyen");
MODULE_DESCRIPTION("Servo driver for hexapod robot");
