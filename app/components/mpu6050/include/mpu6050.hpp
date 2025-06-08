#ifndef MPU6050_HPP
#define MPU6050_HPP

#include "driver/i2c.h"

struct MpuData {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

class Mpu6050 {
public:
    Mpu6050(i2c_port_t i2c_port = I2C_NUM_0);
    void init();
    MpuData read();

private:
    i2c_port_t _port;
    static constexpr uint8_t MPU_ADDR = 0x68;

    void write_byte(uint8_t reg, uint8_t data);
    uint8_t read_byte(uint8_t reg);
    int16_t read_word(uint8_t reg);
};

#endif // MPU6050_HPP
