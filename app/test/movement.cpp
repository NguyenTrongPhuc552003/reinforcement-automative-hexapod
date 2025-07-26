#include "hexapod.hpp"
#include <cstdio>
#include <unistd.h>

int main()
{
    printf("Testing Hexapod Movement...\n");

    Hexapod hexapod;
    if (!hexapod.init())
    {
        printf("Failed to initialize hexapod\n");
        return -1;
    }

    printf("Hardware initialized. Press Ctrl+C to stop autonomous mode.\n");
    sleep(2);

    // Start autonomous walking
    hexapod.run();

    return 0;
}
