#include "hexapod_utils.h"

double cents_to_degrees(int32_t centidegrees)
{
    return (double)centidegrees / 100.0;
}

int32_t degrees_to_cents(double degrees)
{
    return (int32_t)(degrees * 100.0);
}

uint16_t angle_to_pwm(double angle)
{
    // Convert -90 to +90 degrees to PWM range
    return SERVO_CENTER_PWM + (int16_t)(angle * 1000.0 / 90.0);
}

double pwm_to_angle(uint16_t pwm)
{
    return (double)(pwm - SERVO_CENTER_PWM) * 90.0 / 1000.0;
}

double accel_to_g(int16_t raw)
{
    // ±2g range
    return (double)raw / 16384.0;
}

double gyro_to_dps(int16_t raw)
{
    // ±500 degrees/second range
    return (double)raw / 65.5;
}
