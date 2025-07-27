#include "hexapod.hpp"
#include <iostream>

int main()
{
    std::cout << "Hexapod Movement Test" << std::endl;

    Hexapod robot;

    if (!robot.init())
    {
        std::cerr << "Failed to initialize hexapod" << std::endl;
        return -1;
    }

    std::cout << "Setting servos to home position..." << std::endl;
    robot.homePosition();

    std::cout << "Movement test completed" << std::endl;
    return 0;
}
