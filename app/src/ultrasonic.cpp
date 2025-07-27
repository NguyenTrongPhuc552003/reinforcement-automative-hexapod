#include "ultrasonic.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include <chrono>
#include <thread>
#include <iostream>
#include <time.h>

Ultrasonic::Ultrasonic() : trigger_pin_(TRIG_PIN), echo_pin_(ECHO_PIN), initialized_(false)
{
    // Default constructor using predefined pins
}

bool Ultrasonic::init()
{
    // Export GPIO pins
    if (!exportGPIO(trigger_pin_) || !exportGPIO(echo_pin_))
    {
        std::cerr << "Failed to export GPIO pins" << std::endl;
        return false;
    }

    // Set pin directions
    if (!setGPIODirection(trigger_pin_, "out") || !setGPIODirection(echo_pin_, "in"))
    {
        std::cerr << "Failed to set GPIO directions" << std::endl;
        return false;
    }

    // Set trigger pin to low initially
    setGPIOValue(trigger_pin_, 0);

    initialized_ = true;
    std::cout << "HC-SR04 ultrasonic sensor initialized" << std::endl;
    return true;
}

double Ultrasonic::getDistance()
{
    if (!initialized_)
        return -1.0;

    // Send trigger pulse - exactly like working C code
    setGPIOValue(trigger_pin_, 0);
    delayMicroseconds(2); // Initial 2us delay
    setGPIOValue(trigger_pin_, 1);
    delayMicroseconds(10); // 10us trigger pulse
    setGPIOValue(trigger_pin_, 0);

    // Use nanosecond precision timing like C code
    struct timespec start, end;

    // Wait for echo pin to go HIGH (start of pulse)
    auto timeout_start = std::chrono::high_resolution_clock::now();
    while (getGPIOValue(echo_pin_) == 0)
    {
        if (std::chrono::high_resolution_clock::now() - timeout_start > std::chrono::milliseconds(30))
        {
            return -1.0; // Timeout waiting for echo start
        }
    }

    // Record pulse start time
    clock_gettime(CLOCK_MONOTONIC, &start);

    // Wait for echo pin to go LOW (end of pulse)
    auto timeout_end = std::chrono::high_resolution_clock::now();
    while (getGPIOValue(echo_pin_) == 1)
    {
        if (std::chrono::high_resolution_clock::now() - timeout_end > std::chrono::milliseconds(30))
        {
            return -1.0; // Timeout waiting for echo end
        }
    }

    // Record pulse end time
    clock_gettime(CLOCK_MONOTONIC, &end);

    // Calculate pulse duration in microseconds
    long start_ns = start.tv_sec * 1000000000L + start.tv_nsec;
    long end_ns = end.tv_sec * 1000000000L + end.tv_nsec;
    long duration_us = (end_ns - start_ns) / 1000;

    // Calculate distance using exact same formula as C code
    double distance_cm = (duration_us * 0.034) / 2.0;

    return distance_cm;
}

void Ultrasonic::cleanup()
{
    if (initialized_)
    {
        // Unexport GPIO pins
        unexportGPIO(trigger_pin_);
        unexportGPIO(echo_pin_);
        initialized_ = false;
    }
}

bool Ultrasonic::exportGPIO(uint8_t pin)
{
    std::ofstream export_file("/sys/class/gpio/export");
    if (!export_file.is_open())
        return false;

    export_file << static_cast<int>(pin);
    export_file.close();

    // Small delay to allow system to set up the GPIO
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}

bool Ultrasonic::unexportGPIO(uint8_t pin)
{
    std::ofstream unexport_file("/sys/class/gpio/unexport");
    if (!unexport_file.is_open())
        return false;

    unexport_file << static_cast<int>(pin);
    unexport_file.close();
    return true;
}

bool Ultrasonic::setGPIODirection(uint8_t pin, const std::string &direction)
{
    std::string direction_path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    std::ofstream direction_file(direction_path);
    if (!direction_file.is_open())
        return false;

    direction_file << direction;
    direction_file.close();
    return true;
}

bool Ultrasonic::setGPIOValue(uint8_t pin, int value)
{
    std::string value_path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ofstream value_file(value_path);
    if (!value_file.is_open())
        return false;

    value_file << value;
    value_file.close();
    return true;
}

int Ultrasonic::getGPIOValue(uint8_t pin)
{
    std::string value_path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ifstream value_file(value_path);
    if (!value_file.is_open())
        return -1;

    int value;
    value_file >> value;
    value_file.close();
    return value;
}

void Ultrasonic::delayMicroseconds(int us)
{
    struct timespec ts;
    ts.tv_sec = us / 1000000;
    ts.tv_nsec = (us % 1000000) * 1000;
    nanosleep(&ts, NULL);
}
