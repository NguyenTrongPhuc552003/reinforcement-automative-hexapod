#include "mpu6050.hpp"
#include "esp_log.h"

Mpu6050::Mpu6050(i2c_port_t i2c_port) : _port(i2c_port) {}

void Mpu6050::init() {
    write_byte(0x6B, 0);  // Wake up MPU6050
}

void Mpu6050::write_byte(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

int16_t Mpu6050::read_word(uint8_t reg) {
    uint8_t high = read_byte(reg);
    uint8_t low = read_byte(reg + 1);
    return (int16_t)((high << 8) | low);
}

uint8_t Mpu6050::read_byte(uint8_t reg) {
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return data;
}

MpuData Mpu6050::read() {
    MpuData data;
    data.accel_x = read_word(0x3B) / 16384.0f;
    data.accel_y = read_word(0x3D) / 16384.0f;
    data.accel_z = read_word(0x3F) / 16384.0f;
    data.gyro_x = read_word(0x43) / 131.0f;
    data.gyro_y = read_word(0x45) / 131.0f;
    data.gyro_z = read_word(0x47) / 131.0f;
    return data;
}
