#include "pca9685.hpp"
#include <cstdio>
#include <unistd.h>

int main()
{
    printf("Testing Servo Movement...\n");

    PCA9685 pca(PCA9685::ADDR_1);
    if (!pca.init())
    {
        printf("Failed to initialize PCA9685\n");
        return -1;
    }

    printf("Moving servo on channel 0...\n");
    for (int angle = -90; angle <= 90; angle += 30)
    {
        printf("Angle: %d\n", angle);
        pca.setServoAngle(0, angle);
        sleep(1);
    }

    // Center servo
    pca.setServoAngle(0, 0);
    printf("Test complete\n");

    return 0;
}
