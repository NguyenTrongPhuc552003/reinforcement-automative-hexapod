#include "ultrasonic.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <sys/time.h>

Ultrasonic::Ultrasonic() {}

Ultrasonic::~Ultrasonic() {}

bool Ultrasonic::init()
{
    // Export GPIO pins
    if (!exportGPIO(TRIG_PIN) || !exportGPIO(ECHO_PIN))
    {
        return false;
    }

    // Set directions
    if (!setDirection(TRIG_PIN, "out") || !setDirection(ECHO_PIN, "in"))
    {
        return false;
    }

    // Initialize TRIG to LOW
    writeGPIO(TRIG_PIN, 0);
    usleep(100000); // 100ms

    return true;
}

float Ultrasonic::getDistance()
{
    // Send trigger pulse
    writeGPIO(TRIG_PIN, 1);
    usleep(10); // 10us pulse
    writeGPIO(TRIG_PIN, 0);

    // Wait for echo start
    struct timeval start, end;
    int timeout = 10000; // 10ms timeout

    while (readGPIO(ECHO_PIN) == 0 && timeout-- > 0)
    {
        usleep(1);
    }

    if (timeout <= 0)
        return -1; // Timeout

    gettimeofday(&start, nullptr);

    // Wait for echo end
    timeout = 30000; // 30ms timeout
    while (readGPIO(ECHO_PIN) == 1 && timeout-- > 0)
    {
        usleep(1);
    }

    if (timeout <= 0)
        return -1; // Timeout

    gettimeofday(&end, nullptr);

    // Calculate distance
    long duration = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec);
    float distance = duration * 0.034f / 2.0f; // Speed of sound = 34000 cm/s

    return distance;
}

bool Ultrasonic::exportGPIO(int pin)
{
    char command[50];
    sprintf(command, "echo %d > /sys/class/gpio/export", pin);
    return system(command) == 0;
}

bool Ultrasonic::setDirection(int pin, const char *direction)
{
    char filename[50];
    sprintf(filename, "/sys/class/gpio/gpio%d/direction", pin);

    int fd = open(filename, O_WRONLY);
    if (fd < 0)
        return false;

    bool result = write(fd, direction, strlen(direction)) > 0;
    close(fd);
    return result;
}

bool Ultrasonic::writeGPIO(int pin, int value)
{
    char filename[50];
    sprintf(filename, "/sys/class/gpio/gpio%d/value", pin);

    int fd = open(filename, O_WRONLY);
    if (fd < 0)
        return false;

    char val = value ? '1' : '0';
    bool result = write(fd, &val, 1) == 1;
    close(fd);
    return result;
}

int Ultrasonic::readGPIO(int pin)
{
    char filename[50];
    sprintf(filename, "/sys/class/gpio/gpio%d/value", pin);

    int fd = open(filename, O_RDONLY);
    if (fd < 0)
        return -1;

    char val;
    read(fd, &val, 1);
    close(fd);

    return val == '1' ? 1 : 0;
}
