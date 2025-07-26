#include "pca9685.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <cstdio>

// PCA9685 Registers
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06

PCA9685::PCA9685(uint8_t address) : addr(address), fd(-1) {}

PCA9685::~PCA9685()
{
    if (fd >= 0)
    {
        close(fd);
    }
}

bool PCA9685::init()
{
    char filename[20];
    sprintf(filename, "/dev/i2c-%d", I2C_BUS);

    fd = open(filename, O_RDWR);
    if (fd < 0)
    {
        printf("Failed to open i2c bus\n");
        return false;
    }

    if (ioctl(fd, I2C_SLAVE, addr) < 0)
    {
        printf("Failed to acquire bus access\n");
        close(fd);
        fd = -1;
        return false;
    }

    // Reset
    writeReg(PCA9685_MODE1, 0x00);
    usleep(10000);

    // Set frequency to 50Hz for servos
    setFreq(50.0f);

    return true;
}

void PCA9685::setFreq(float freq)
{
    float prescaleval = 25000000.0f;
    prescaleval /= 4096.0f;
    prescaleval /= freq;
    prescaleval -= 1.0f;

    uint8_t prescale = (uint8_t)(prescaleval + 0.5f);

    uint8_t oldmode = readReg(PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep

    writeReg(PCA9685_MODE1, newmode);
    writeReg(PCA9685_PRESCALE, prescale);
    writeReg(PCA9685_MODE1, oldmode);
    usleep(5000);
    writeReg(PCA9685_MODE1, oldmode | 0x80); // restart
}

void PCA9685::setPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;

    writeReg(reg, on & 0xFF);
    writeReg(reg + 1, on >> 8);
    writeReg(reg + 2, off & 0xFF);
    writeReg(reg + 3, off >> 8);
}

void PCA9685::setServoAngle(uint8_t channel, float angle)
{
    // Convert angle (-90 to 90) to pulse width (1000-2000us)
    // For 50Hz: 4096 ticks = 20ms, so 1ms = 204.8 ticks

    float pulse = 1500.0f + (angle * 500.0f / 90.0f); // 1000-2000us
    uint16_t ticks = (uint16_t)(pulse * 4096.0f / 20000.0f);

    setPWM(channel, 0, ticks);
}

bool PCA9685::writeReg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return write(fd, buf, 2) == 2;
}

uint8_t PCA9685::readReg(uint8_t reg)
{
    write(fd, &reg, 1);
    uint8_t value = 0;
    read(fd, &value, 1);
    return value;
}
