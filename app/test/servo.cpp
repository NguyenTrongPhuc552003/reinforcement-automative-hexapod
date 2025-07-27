#include "pca9685.hpp"
#include <iostream>

int main()
{
    std::cout << "Servo Test" << std::endl;

    PCA9685 controller;

    if (!controller.init())
    {
        std::cerr << "Failed to initialize PCA9685" << std::endl;
        return -1;
    }

    std::cout << "Setting all servos to home position..." << std::endl;
    controller.setAllServosHome();

    std::cout << "Servo test completed" << std::endl;
    return 0;
}
