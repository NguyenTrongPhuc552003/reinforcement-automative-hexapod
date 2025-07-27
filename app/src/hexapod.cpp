#include "hexapod.hpp"
#include <iostream>
#include <thread>
#include <chrono>

Hexapod::Hexapod() : initialized_(false), moving_(false), current_state_(STANDING), step_phase_(0)
{
    initializeServoMapping();
    last_step_time_ = std::chrono::steady_clock::now();
}

Hexapod::~Hexapod()
{
    cleanup();
}

void Hexapod::initializeServoMapping()
{
    // Map leg indices to servo channels based on the channel mapping
    // PCA9685 #1 (0x40): FR(0,1,2), FL(3,4,5), MR(6,7,8)
    // PCA9685 #2 (0x41): ML(0,1,2), BR(3,4,5), BL(6,7,8)

    leg_servo_mapping_[0] = {FR_COXA, FR_FEMUR, FR_TIBIA}; // Front Right
    leg_servo_mapping_[1] = {FL_COXA, FL_FEMUR, FL_TIBIA}; // Front Left
    leg_servo_mapping_[2] = {MR_COXA, MR_FEMUR, MR_TIBIA}; // Middle Right
    leg_servo_mapping_[3] = {ML_COXA, ML_FEMUR, ML_TIBIA}; // Middle Left
    leg_servo_mapping_[4] = {BR_COXA, BR_FEMUR, BR_TIBIA}; // Back Right
    leg_servo_mapping_[5] = {BL_COXA, BL_FEMUR, BL_TIBIA}; // Back Left
}

bool Hexapod::init()
{
    if (initialized_)
        return true;

    if (!pwm_controller_.init())
    {
        std::cerr << "Failed to initialize PWM controller" << std::endl;
        return false;
    }

    if (!ultrasonic_.init())
    {
        std::cerr << "Failed to initialize ultrasonic sensor" << std::endl;
        return false;
    }

    initialized_ = true;
    return true;
}

void Hexapod::cleanup()
{
    if (initialized_)
    {
        stopMovement();
        pwm_controller_.cleanup();
        ultrasonic_.cleanup();
        initialized_ = false;
    }
}

bool Hexapod::homePosition()
{
    if (!initialized_)
    {
        std::cerr << "Hexapod not initialized" << std::endl;
        return false;
    }

    return pwm_controller_.setAllServosHome();
}

bool Hexapod::startAutonomousMovement()
{
    if (!initialized_)
    {
        std::cerr << "Hexapod not initialized" << std::endl;
        return false;
    }

    moving_ = true;
    current_state_ = WALKING_FORWARD;
    step_phase_ = 0;
    last_step_time_ = std::chrono::steady_clock::now();

    std::cout << "Autonomous movement started!" << std::endl;
    return true;
}

void Hexapod::stopMovement()
{
    moving_ = false;
    current_state_ = STANDING;
    setAllLegsPosition(standing_position_);
    std::cout << "Movement stopped" << std::endl;
}

bool Hexapod::moveForward()
{
    if (!initialized_)
        return false;

    current_state_ = WALKING_FORWARD;
    moving_ = true;
    return true;
}

bool Hexapod::turnLeft()
{
    if (!initialized_)
        return false;

    current_state_ = TURNING_LEFT;
    moving_ = true;
    return true;
}

bool Hexapod::turnRight()
{
    if (!initialized_)
        return false;

    current_state_ = TURNING_RIGHT;
    moving_ = true;
    return true;
}

bool Hexapod::stepBack()
{
    if (!initialized_)
        return false;

    current_state_ = BACKING_UP;
    moving_ = true;
    return true;
}

bool Hexapod::crawlSideways()
{
    if (!initialized_)
        return false;

    current_state_ = CRAWLING_SIDEWAYS;
    moving_ = true;
    return true;
}

void Hexapod::update(double /*time_step*/)
{
    if (!initialized_ || !moving_)
        return;

    // Check for obstacles first (except when already in pause state)
    if (current_state_ != PAUSED_FOR_OBSTACLE && isObstacleDetected())
    {
        handleObstacleAvoidance();
        return;
    }

    // If we're in pause state, handle the obstacle avoidance logic
    if (current_state_ == PAUSED_FOR_OBSTACLE)
    {
        handleObstacleAvoidance();
        return;
    }

    // Check if it's time for next step (500ms interval)
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_step_time_);

    if (elapsed.count() >= 500) // 500ms per step
    {
        performTripodStep();
        last_step_time_ = current_time;
    }
}

bool Hexapod::setServoAngles(uint8_t leg_index, uint16_t coxa_us, uint16_t femur_us, uint16_t tibia_us)
{
    if (!initialized_ || leg_index >= 6)
        return false;

    const auto &servo_map = leg_servo_mapping_[leg_index];

    // Set servo positions
    return pwm_controller_.setServoMicroseconds(servo_map.coxa_channel, coxa_us) &&
           pwm_controller_.setServoMicroseconds(servo_map.femur_channel, femur_us) &&
           pwm_controller_.setServoMicroseconds(servo_map.tibia_channel, tibia_us);
}

void Hexapod::performTripodStep()
{
    switch (current_state_)
    {
    case WALKING_FORWARD:
        if (step_phase_ == 0)
        {
            // Lift group A, move group B forward
            setTripodGroup(tripod_group_a_, lift_position_);
            setTripodGroup(tripod_group_b_, forward_position_);
        }
        else
        {
            // Put group A down forward, lift group B
            setTripodGroup(tripod_group_a_, forward_position_);
            setTripodGroup(tripod_group_b_, lift_position_);
        }
        break;

    case TURNING_LEFT:
        if (step_phase_ == 0)
        {
            // Group A: turn left (coxa less than 1500)
            setTripodGroup(tripod_group_a_, {1400, 1500, 1500});
            setTripodGroup(tripod_group_b_, lift_position_);
        }
        else
        {
            setTripodGroup(tripod_group_a_, lift_position_);
            setTripodGroup(tripod_group_b_, {1400, 1500, 1500});
        }
        break;

    case TURNING_RIGHT:
        if (step_phase_ == 0)
        {
            // Group A: turn right (coxa more than 1500)
            setTripodGroup(tripod_group_a_, {1600, 1500, 1500});
            setTripodGroup(tripod_group_b_, lift_position_);
        }
        else
        {
            setTripodGroup(tripod_group_a_, lift_position_);
            setTripodGroup(tripod_group_b_, {1600, 1500, 1500});
        }
        break;

    case BACKING_UP:
        if (step_phase_ == 0)
        {
            // Move backward (opposite of forward)
            setTripodGroup(tripod_group_a_, lift_position_);
            setTripodGroup(tripod_group_b_, back_position_);
        }
        else
        {
            setTripodGroup(tripod_group_a_, back_position_);
            setTripodGroup(tripod_group_b_, lift_position_);
        }
        break;

    case CRAWLING_SIDEWAYS:
        if (step_phase_ == 0)
        {
            // Sideways crawling: lift group A, move group B sideways to the right
            setTripodGroup(tripod_group_a_, lift_position_);
            // For sideways movement, adjust femur outward (1600) and tibia down (1700) for stability
            setTripodGroup(tripod_group_b_, {1500, 1600, 1700});
        }
        else
        {
            // Put group A down in sideways position, lift group B
            setTripodGroup(tripod_group_a_, {1500, 1600, 1700});
            setTripodGroup(tripod_group_b_, lift_position_);
        }
        break;

    case PAUSED_FOR_OBSTACLE:
        // During pause, keep all legs in standing position
        setAllLegsPosition(standing_position_);
        return; // Don't change step_phase during pause

    default:
        setAllLegsPosition(standing_position_);
        break;
    }

    // Alternate phase
    step_phase_ = 1 - step_phase_;
}

void Hexapod::setTripodGroup(const uint8_t group[], const ServoPositions &position)
{
    for (int i = 0; i < 3; i++)
    {
        setServoAngles(group[i], position.coxa, position.femur, position.tibia);
    }
}

void Hexapod::setAllLegsPosition(const ServoPositions &position)
{
    for (uint8_t leg = 0; leg < 6; leg++)
    {
        setServoAngles(leg, position.coxa, position.femur, position.tibia);
    }
}

bool Hexapod::isObstacleDetected()
{
    double distance = ultrasonic_.getDistance();

    static int reading_count = 0;
    reading_count++;

    // Print every 20 readings to avoid spam
    if (reading_count % 20 == 0)
    {
        std::cout << "Ultrasonic reading #" << reading_count << ": ";
        if (distance >= 0)
        {
            std::cout << distance << " cm";
            if (distance < obstacle_threshold_cm_)
                std::cout << "OBSTACLE DETECTED!";
            else
                std::cout << "Clear path";
        }
        else
        {
            std::cout << "ERROR reading sensor";
        }
        std::cout << std::endl;
    }

    return (distance > 0 && distance < obstacle_threshold_cm_);
}

void Hexapod::handleObstacleAvoidance()
{
    double current_distance = ultrasonic_.getDistance();

    std::cout << "OBSTACLE AVOIDANCE TRIGGERED!" << std::endl;
    std::cout << "   Current distance: " << current_distance << " cm" << std::endl;
    std::cout << "   Threshold: " << obstacle_threshold_cm_ << " cm" << std::endl;

    // Check if we're already in pause state
    if (current_state_ != PAUSED_FOR_OBSTACLE)
    {
        // First time detecting obstacle - start pause
        std::cout << "   Action: Stopping for 1 second before crawling sideways..." << std::endl;
        current_state_ = PAUSED_FOR_OBSTACLE;
        obstacle_pause_start_ = std::chrono::steady_clock::now();

        // Stop all movement - set to standing position
        setAllLegsPosition(standing_position_);
        return;
    }

    // We're in pause state - check if 1 second has passed
    auto current_time = std::chrono::steady_clock::now();
    auto pause_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - obstacle_pause_start_);

    if (pause_duration.count() < 1000) // Still pausing
    {
        std::cout << "   Pausing... (" << pause_duration.count() << "ms / 1000ms)" << std::endl;
        return;
    }

    // Pause complete - now crawl sideways to avoid obstacle
    std::cout << "   Pause complete! Now crawling sideways to avoid obstacle..." << std::endl;
    current_state_ = CRAWLING_SIDEWAYS;

    // Perform sideways crawling movement for a short duration
    static auto crawl_start = current_time;
    auto crawl_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - crawl_start);

    if (crawl_duration.count() < 1000) // Crawl sideways for 1 second
    {
        // Perform crawling step
        performTripodStep();
        return;
    }

    // Reset crawl timer and check distance after sideways movement
    crawl_start = current_time;
    double new_distance = ultrasonic_.getDistance();
    std::cout << "   Distance after sideways crawl: " << new_distance << " cm" << std::endl;

    if (new_distance > safe_distance_cm_)
    {
        std::cout << "   Path clear! Resuming forward movement" << std::endl;
        current_state_ = WALKING_FORWARD;
    }
    else
    {
        std::cout << "   Still detecting obstacle, pausing again before next crawl..." << std::endl;
        // Go back to pause state for another cycle
        current_state_ = PAUSED_FOR_OBSTACLE;
        obstacle_pause_start_ = current_time;
    }
}
