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

    // Check if it's time for next step (600ms interval for smoother, more natural movement)
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_step_time_);

    if (elapsed.count() >= 600) // 600ms per step for smooth, natural movement
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
            std::cout << "Forward walking phase 0: Smooth tripod gait - Group A advancing" << std::endl;
            // Phase 1: Lift Group A smoothly
            setTripodGroup(tripod_group_a_, lift_position_);
            std::this_thread::sleep_for(std::chrono::milliseconds(180)); // Reduced for smoother flow

            // Phase 2: Move Group B forward while Group A pushes backward for forward momentum
            setTripodGroup(tripod_group_a_, back_position_);
            setTripodGroup(tripod_group_b_, forward_position_);
            std::this_thread::sleep_for(std::chrono::milliseconds(120)); // Quick stabilization

            // Phase 3: Plant Group A for stable landing
            setTripodGroup(tripod_group_a_, standing_position_);
        }
        else
        {
            std::cout << "Forward walking phase 1: Smooth tripod gait - Group B advancing" << std::endl;
            // Phase 1: Lift Group B smoothly
            setTripodGroup(tripod_group_b_, lift_position_);
            std::this_thread::sleep_for(std::chrono::milliseconds(180)); // Consistent timing

            // Phase 2: Move Group A forward while Group B pushes backward for forward momentum
            setTripodGroup(tripod_group_b_, back_position_);
            setTripodGroup(tripod_group_a_, forward_position_);
            std::this_thread::sleep_for(std::chrono::milliseconds(120)); // Quick stabilization

            // Phase 3: Plant Group B for stable landing
            setTripodGroup(tripod_group_b_, standing_position_);
        }
        break;

    case TURNING_LEFT:
        if (step_phase_ == 0)
        {
            std::cout << "Turning left phase 0: Smooth left turn" << std::endl;
            setTripodGroup(tripod_group_a_, lift_position_);
            std::this_thread::sleep_for(std::chrono::milliseconds(160));
            setTripodGroup(tripod_group_a_, turn_left_pos);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            setTripodGroup(tripod_group_a_, standing_position_);
        }
        else
        {
            std::cout << "Turning left phase 1: Completing left turn" << std::endl;
            setTripodGroup(tripod_group_b_, lift_position_);
            std::this_thread::sleep_for(std::chrono::milliseconds(160));
            setTripodGroup(tripod_group_b_, turn_left_pos);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            setTripodGroup(tripod_group_b_, standing_position_);
        }
        break;

    case TURNING_RIGHT:
        if (step_phase_ == 0)
        {
            std::cout << "Turning right phase 0: Smooth right turn" << std::endl;
            setTripodGroup(tripod_group_a_, lift_position_);
            std::this_thread::sleep_for(std::chrono::milliseconds(160));
            setTripodGroup(tripod_group_a_, turn_right_pos);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            setTripodGroup(tripod_group_a_, standing_position_);
        }
        else
        {
            std::cout << "Turning right phase 1: Completing right turn" << std::endl;
            setTripodGroup(tripod_group_b_, lift_position_);
            std::this_thread::sleep_for(std::chrono::milliseconds(160));
            setTripodGroup(tripod_group_b_, turn_right_pos);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            setTripodGroup(tripod_group_b_, standing_position_);
        }
        break;

    case BACKING_UP:
        if (step_phase_ == 0)
        {
            std::cout << "Backing up phase 0: Smooth backward movement" << std::endl;
            setTripodGroup(tripod_group_a_, lift_position_);
            std::this_thread::sleep_for(std::chrono::milliseconds(180));
            setTripodGroup(tripod_group_a_, forward_position_); // FIXED: Move forward to push robot backward
            setTripodGroup(tripod_group_b_, back_position_);    // FIXED: Group B prepares for next step
            std::this_thread::sleep_for(std::chrono::milliseconds(120));
            setTripodGroup(tripod_group_a_, standing_position_);
        }
        else
        {
            std::cout << "Backing up phase 1: Continuing backward movement" << std::endl;
            setTripodGroup(tripod_group_b_, lift_position_);
            std::this_thread::sleep_for(std::chrono::milliseconds(180));
            setTripodGroup(tripod_group_b_, forward_position_); // FIXED: Move forward to push robot backward
            setTripodGroup(tripod_group_a_, back_position_);    // FIXED: Group A prepares for next step
            std::this_thread::sleep_for(std::chrono::milliseconds(120));
            setTripodGroup(tripod_group_b_, standing_position_);
        }
        break;

    case CRAWLING_SIDEWAYS:
        if (step_phase_ == 0)
        {
            std::cout << "Sideways crawling phase 0: Enhanced obstacle avoidance movement" << std::endl;
            // High lift for obstacle clearance
            setTripodGroup(tripod_group_a_, sideways_lift);
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Extra time for clearance
            // Extended sideways reach for effective avoidance
            setTripodGroup(tripod_group_a_, sideways_reach);
            std::this_thread::sleep_for(std::chrono::milliseconds(150)); // Stabilization
            // Plant down in new position
            setTripodGroup(tripod_group_a_, standing_position_);
        }
        else
        {
            std::cout << "Sideways crawling phase 1: Continuing enhanced sideways movement" << std::endl;
            // High lift for obstacle clearance
            setTripodGroup(tripod_group_b_, sideways_lift);
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Extra time for clearance
            // Extended sideways reach for effective avoidance
            setTripodGroup(tripod_group_b_, sideways_reach);
            std::this_thread::sleep_for(std::chrono::milliseconds(150)); // Stabilization
            // Plant down in new position
            setTripodGroup(tripod_group_b_, standing_position_);
        }
        break;

    case PAUSED_FOR_OBSTACLE:
        // During pause, maintain stable standing position with slight body elevation for readiness
        std::cout << "Maintaining elevated stable position during obstacle detection" << std::endl;
        setAllLegsPosition(standing_position_);
        return; // Don't change step_phase during pause

    default:
        std::cout << "Default state: returning to stable standing position" << std::endl;
        setAllLegsPosition(standing_position_);
        break;
    }

    // Alternate phase for continuous, smooth movement
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
    if (current_state_ != PAUSED_FOR_OBSTACLE &&
        current_state_ != BACKING_UP &&
        current_state_ != TURNING_LEFT)
    {
        std::cout << "Obstacle detected! Pausing..." << std::endl;
        current_state_ = PAUSED_FOR_OBSTACLE;
        obstacle_pause_start_ = std::chrono::steady_clock::now();
        setAllLegsPosition(standing_position_);
        return;
    }

    auto now = std::chrono::steady_clock::now();

    // After pausing for 0.5 second, start backing up
    if (current_state_ == PAUSED_FOR_OBSTACLE)
    {
        auto pause_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - obstacle_pause_start_).count();
        if (pause_ms >= 500) // down from 1000ms to 500ms
        {
            std::cout << "Pause done. Start backing up..." << std::endl;
            current_state_ = BACKING_UP;
            back_start_ = now;
        }
        return;
    }

    // Back up in 0.8 seconds
    if (current_state_ == BACKING_UP)
    {
        auto back_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - back_start_).count();
        if (back_ms >= 800) // down from 2000ms to 800ms
        {
            std::cout << "Backing done. Start turning left 60deg..." << std::endl;
            current_state_ = TURNING_LEFT;
            turn_start_ = now;
        }
        return;
    }

    // Turn left in 1.2 seconds
    if (current_state_ == TURNING_LEFT)
    {
        auto turn_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - turn_start_).count();
        if (turn_ms >= 1200) // down from 2000ms to 1200ms
        {
            std::cout << "Turning done. Resume forward walking." << std::endl;
            current_state_ = WALKING_FORWARD;
        }
        return;
    }
}
