#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include "controller.hpp"

class ControllerImpl
{
public:
    ControllerImpl(Hexapod &hexapod) : hexapod(hexapod),
                                       speed(0.5),
                                       direction(0.0),
                                       height(0.0),
                                       tiltX(0.0),
                                       tiltY(0.0),
                                       gaitType(GaitType::TRIPOD),
                                       state(ControllerState::IDLE)
    {
        clock_gettime(CLOCK_MONOTONIC, &lastUpdate);
    }

    ~ControllerImpl()
    {
        // Restore terminal settings if needed
        if (terminalConfigured)
        {
            tcsetattr(STDIN_FILENO, TCSANOW, &origTermios);
        }

        // Make sure hexapod is centered before exit
        hexapod.centerAll();
    }

    bool init()
    {
        // Set up terminal for immediate key input
        terminalConfigured = true;
        if (tcgetattr(STDIN_FILENO, &origTermios) < 0)
        {
            terminalConfigured = false;
        }
        else
        {
            struct termios newTermios = origTermios;
            newTermios.c_lflag &= ~(ICANON | ECHO);
            if (tcsetattr(STDIN_FILENO, TCSANOW, &newTermios) < 0)
            {
                terminalConfigured = false;
            }
        }

        // Initialize gait controller
        GaitParameters params;
        params.type = gaitType;
        params.stepHeight = 30.0;
        params.stepLength = 60.0;
        params.cycleTime = 1.0;
        params.dutyFactor = 0.5;

        if (!gait.init(hexapod, params))
        {
            fprintf(stderr, "Failed to initialize gait controller\n");
            return false;
        }

        return true;
    }

    bool processKey(char key)
    {
        switch (key)
        {
        case 'w': // Forward
            direction = 0.0;
            state = ControllerState::WALKING;
            break;
        case 's': // Backward
            direction = 180.0;
            state = ControllerState::WALKING;
            break;
        case 'a': // Rotate left
            direction = -90.0;
            state = ControllerState::ROTATING;
            break;
        case 'd': // Rotate right
            direction = 90.0;
            state = ControllerState::ROTATING;
            break;
        case 'i': // Raise body
            height += 10.0;
            break;
        case 'k': // Lower body
            height -= 10.0;
            break;
        case 'j': // Tilt left
            tiltY = -15.0;
            state = ControllerState::TILTING;
            break;
        case 'l': // Tilt right
            tiltY = 15.0;
            state = ControllerState::TILTING;
            break;
        case '1': // Tripod gait
            setGaitType(GaitType::TRIPOD);
            break;
        case '2': // Wave gait
            setGaitType(GaitType::WAVE);
            break;
        case '3': // Ripple gait
            setGaitType(GaitType::RIPPLE);
            break;
        case '+': // Increase speed
            speed = std::min(1.0, speed + 0.1);
            printf("Speed: %.1f\n", speed);
            break;
        case '-': // Decrease speed
            speed = std::max(0.1, speed - 0.1);
            printf("Speed: %.1f\n", speed);
            break;
        case ' ': // Stop
            state = ControllerState::IDLE;
            return hexapod.centerAll();
        default:
            return true;
        }
        return true;
    }

    bool update()
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        double timeDelta = (now.tv_sec - lastUpdate.tv_sec) +
                           (now.tv_nsec - lastUpdate.tv_nsec) / 1.0e9;

        // Skip update if time delta is too small (prevents excessive CPU usage)
        if (timeDelta < 0.005)
        { // Only update at ~200Hz max
            return true;
        }

        bool result = true;

        // Cache the current state to avoid repeated comparisons
        ControllerState currentState = state;

        switch (currentState)
        {
        case ControllerState::WALKING:
        case ControllerState::ROTATING:
            result = gait.update(timeDelta, direction, speed);
            break;

        case ControllerState::TILTING:
            // Implement tilt control using inverse kinematics
            result = applyTilt();
            break;

        case ControllerState::IDLE:
            // Do nothing in idle state
            break;
        }

        lastUpdate = now;
        return result;
    }

    // Accessors and mutators
    ControllerState getState() const { return state; }
    void setSpeed(double newSpeed) { speed = std::max(0.0, std::min(1.0, newSpeed)); }
    void setDirection(double newDirection) { direction = newDirection; }
    void setHeight(double newHeight) { height = newHeight; }
    void setTilt(double newTiltX, double newTiltY)
    {
        tiltX = newTiltX;
        tiltY = newTiltY;
    }

    void setGaitType(GaitType type)
    {
        if (type == gaitType)
            return;

        gaitType = type;

        // Update gait controller with new type
        GaitParameters params = gait.getParameters();
        params.type = gaitType;

        // Adjust parameters based on gait type
        switch (gaitType)
        {
        case GaitType::TRIPOD:
            params.dutyFactor = 0.5;
            params.stepHeight = 30.0;
            break;
        case GaitType::WAVE:
            params.dutyFactor = 0.75;
            params.stepHeight = 40.0;
            break;
        case GaitType::RIPPLE:
            params.dutyFactor = 0.65;
            params.stepHeight = 35.0;
            break;
        }

        gait.setParameters(params);

        const char *gaitName = "";
        switch (gaitType)
        {
        case GaitType::TRIPOD:
            gaitName = "Tripod";
            break;
        case GaitType::WAVE:
            gaitName = "Wave";
            break;
        case GaitType::RIPPLE:
            gaitName = "Ripple";
            break;
        }
        printf("Switched to %s gait\n", gaitName);
    }

private:
    // Apply tilting to the body
    bool applyTilt()
    {
        // TODO: Implement proper body orientation control
        // This is a simplified version - for a complete implementation
        // you would adjust the height of legs based on tilt angles

        // Reset tilt gradually when not actively tilting
        if (state != ControllerState::TILTING)
        {
            tiltX *= 0.9; // Reduce tilt by 10% each update
            tiltY *= 0.9;

            // If tilt is very small, consider it zero
            if (std::abs(tiltX) < 0.1 && std::abs(tiltY) < 0.1)
            {
                tiltX = 0;
                tiltY = 0;
                return hexapod.centerAll();
            }
        }

        return true;
    }

    // Member variables
    Hexapod &hexapod;
    Gait gait;
    struct termios origTermios;
    bool terminalConfigured = false;
    struct timespec lastUpdate;

    double speed;     // 0.0 to 1.0
    double direction; // Degrees, 0 = forward
    double height;    // Body height adjustment in mm
    double tiltX;     // Forward/backward tilt in degrees
    double tiltY;     // Left/right tilt in degrees
    GaitType gaitType;
    ControllerState state;
};

// Controller class implementation
Controller::Controller(Hexapod &hexapod) : pImpl(std::make_unique<ControllerImpl>(hexapod))
{
}

Controller::~Controller() = default;

Controller::Controller(Controller &&other) noexcept = default;

Controller &Controller::operator=(Controller &&other) noexcept = default;

bool Controller::init()
{
    return pImpl->init();
}

bool Controller::processKey(char key)
{
    return pImpl->processKey(key);
}

bool Controller::update()
{
    return pImpl->update();
}

ControllerState Controller::getState() const
{
    return pImpl->getState();
}

void Controller::setSpeed(double speed)
{
    pImpl->setSpeed(speed);
}

void Controller::setDirection(double direction)
{
    pImpl->setDirection(direction);
}

void Controller::setHeight(double height)
{
    pImpl->setHeight(height);
}

void Controller::setTilt(double tiltX, double tiltY)
{
    pImpl->setTilt(tiltX, tiltY);
}

void Controller::setGaitType(GaitType type)
{
    pImpl->setGaitType(type);
}
