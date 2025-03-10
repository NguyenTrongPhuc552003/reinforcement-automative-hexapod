#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <memory>
#include "gait.hpp"

// Forward declaration for implementation
class ControllerImpl;

// Controller states
enum class ControllerState
{
    IDLE,
    WALKING,
    ROTATING,
    TILTING
};

// Controller class for user input handling
class Controller
{
public:
    // Constructor with hexapod reference
    explicit Controller(Hexapod &hexapod);
    ~Controller();

    // Non-copyable
    Controller(const Controller &) = delete;
    Controller &operator=(const Controller &) = delete;

    // Move constructor and assignment
    Controller(Controller &&) noexcept;
    Controller &operator=(Controller &&) noexcept;

    // Initialize controller
    bool init();

    // Process keyboard input
    bool processKey(char key);

    // Update hexapod state
    bool update();

    // Get current state
    ControllerState getState() const;

    // Set movement parameters
    void setSpeed(double speed);
    void setDirection(double direction);
    void setHeight(double height);
    void setTilt(double tiltX, double tiltY);
    void setGaitType(GaitType type);

private:
    std::unique_ptr<ControllerImpl> pImpl;
};

#endif /* CONTROLLER_HPP */
