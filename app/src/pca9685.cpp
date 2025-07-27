#include "pca9685.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>

PCA9685::PCA9685() : initialized_(false), device_fd_(-1) {}

PCA9685::~PCA9685()
{
    cleanup();
}

bool PCA9685::init()
{
    if (initialized_)
        return true;

    // Open I2C device
    device_fd_ = open("/dev/i2c-2", O_RDWR);
    if (device_fd_ < 0)
    {
        std::cerr << "Failed to open I2C device /dev/i2c-2" << std::endl;
        return false;
    }

    // Initialize both PCA9685 devices
    uint8_t devices[] = {DEFAULT_ADDR_1, DEFAULT_ADDR_2};

    for (uint8_t addr : devices)
    {
        if (ioctl(device_fd_, I2C_SLAVE, addr) < 0)
        {
            std::cerr << "Failed to set I2C slave address: 0x" << std::hex << (int)addr << std::endl;
            close(device_fd_);
            device_fd_ = -1;
            return false;
        }

        // Reset device
        if (!writeRegister(addr, 0x00, 0x00))
        {
            std::cerr << "Failed to reset PCA9685 at 0x" << std::hex << (int)addr << std::endl;
            close(device_fd_);
            device_fd_ = -1;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Set PWM frequency to 50Hz for servos
        // Formula: prescale = round(osc_clock / (4096 * freq)) - 1
        // osc_clock = 25MHz, freq = 50Hz
        uint8_t prescale = static_cast<uint8_t>(std::round(25000000.0 / (4096.0 * 50.0)) - 1);

        // Enter sleep mode to set prescale
        if (!writeRegister(addr, 0x00, 0x10))
        {
            std::cerr << "Failed to enter sleep mode" << std::endl;
            return false;
        }

        // Set prescale
        if (!writeRegister(addr, 0xFE, prescale))
        {
            std::cerr << "Failed to set prescale" << std::endl;
            return false;
        }

        // Wake up and enable auto-increment
        if (!writeRegister(addr, 0x00, 0x20))
        {
            std::cerr << "Failed to wake up device" << std::endl;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // Restart
        if (!writeRegister(addr, 0x00, 0xA0))
        {
            std::cerr << "Failed to restart device" << std::endl;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    initialized_ = true;
    std::cout << "PCA9685 initialized successfully" << std::endl;
    return true;
}

void PCA9685::cleanup()
{
    if (device_fd_ >= 0)
    {
        close(device_fd_);
        device_fd_ = -1;
    }
    initialized_ = false;
}

bool PCA9685::writeRegister(uint8_t device_addr, uint8_t reg, uint8_t value)
{
    if (device_fd_ < 0)
        return false;

    if (ioctl(device_fd_, I2C_SLAVE, device_addr) < 0)
    {
        return false;
    }

    uint8_t buffer[2] = {reg, value};
    return write(device_fd_, buffer, 2) == 2;
}

bool PCA9685::setPWM(uint8_t device_addr, uint8_t channel, uint16_t on, uint16_t off)
{
    if (device_fd_ < 0 || channel >= CHANNELS_PER_DEVICE)
    {
        return false;
    }

    if (ioctl(device_fd_, I2C_SLAVE, device_addr) < 0)
    {
        return false;
    }

    uint8_t reg_base = 0x06 + 4 * channel;
    uint8_t data[4] = {
        static_cast<uint8_t>(on & 0xFF),
        static_cast<uint8_t>(on >> 8),
        static_cast<uint8_t>(off & 0xFF),
        static_cast<uint8_t>(off >> 8)};

    uint8_t buffer[5] = {reg_base, data[0], data[1], data[2], data[3]};
    return write(device_fd_, buffer, 5) == 5;
}

uint16_t PCA9685::microsecondsToTicks(uint16_t microseconds)
{
    // 50Hz = 20ms period
    // 4096 ticks per period
    // 1 tick = 20000us / 4096 = 4.88us
    return static_cast<uint16_t>(microseconds / 4.88);
}

bool PCA9685::setServoMicroseconds(uint8_t channel, uint16_t microseconds)
{
    if (!initialized_ || channel >= TOTAL_SERVOS)
    {
        return false;
    }

    // Clamp values
    if (microseconds < SERVO_MIN_PULSE)
        microseconds = SERVO_MIN_PULSE;
    if (microseconds > SERVO_MAX_PULSE)
        microseconds = SERVO_MAX_PULSE;

    // Map logical servo index to physical device and channel
    uint8_t device_addr;
    uint8_t device_channel;

    if (channel < 9)
    {
        // Servos 0-8: PCA9685 #1 (0x40)
        // FR: 0,1,2 -> channels 0,1,2
        // FL: 3,4,5 -> channels 3,4,5
        // MR: 6,7,8 -> channels 6,7,8
        device_addr = DEFAULT_ADDR_1;
        device_channel = channel;
    }
    else
    {
        // Servos 9-17: PCA9685 #2 (0x41)
        // ML: 9,10,11 -> channels 0,1,2
        // BR: 12,13,14 -> channels 3,4,5
        // BL: 15,16,17 -> channels 6,7,8
        device_addr = DEFAULT_ADDR_2;
        device_channel = channel - 9;
    }

    uint16_t ticks = microsecondsToTicks(microseconds);

    return setPWM(device_addr, device_channel, 0, ticks);
}

bool PCA9685::setAllServosHome()
{
    if (!initialized_)
    {
        std::cerr << "PCA9685 not initialized" << std::endl;
        return false;
    }

    std::cout << "Setting all " << (int)TOTAL_SERVOS << " servos to HOME position ("
              << SERVO_HOME_PULSE << "us)..." << std::endl;

    bool success = true;
    for (uint8_t i = 0; i < TOTAL_SERVOS; i++)
    {
        if (!setServoMicroseconds(i, SERVO_HOME_PULSE))
        {
            std::cerr << "Failed to set servo " << (int)i << " to HOME position" << std::endl;
            success = false;
        }
        else
        {
            std::cout << "Servo " << (int)i << " -> HOME" << std::endl;
        }

        // Small delay between servo commands
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (success)
    {
        std::cout << "All servos set to HOME position successfully!" << std::endl;
    }

    return success;
}
