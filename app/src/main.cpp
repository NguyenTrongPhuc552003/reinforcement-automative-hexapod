#include "application.hpp"

int main(void)
{
    // Get singleton instance and initialize
    Application &app = Application::getInstance();

    // Initialize with error checking
    if (!app.init())
    {
        return 1; // Error code, app handles internal reporting
    }

    // Run and return appropriate exit code (0 for success, 1 for errors)
    auto result = app.run();
    return (result == Application::Result::SUCCESS ||
            result == Application::Result::TERMINATED_BY_USER)
               ? 0
               : 1;
}
