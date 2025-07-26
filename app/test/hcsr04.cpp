#include "ultrasonic.hpp"
#include <cstdio>
#include <unistd.h>

int main()
{
    printf("Testing HC-SR04 Ultrasonic Sensor...\n");

    Ultrasonic sensor;
    if (!sensor.init())
    {
        printf("Failed to initialize sensor\n");
        return -1;
    }

    for (int i = 0; i < 10; i++)
    {
        float distance = sensor.getDistance();
        printf("Distance: %.1f cm\n", distance);
        sleep(1);
    }

    return 0;
}
