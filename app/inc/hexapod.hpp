#ifndef HEXAPOD_HPP
#define HEXAPOD_HPP

#include "pca9685.hpp"
#include "ultrasonic.hpp"
#include <array>
#include <chrono>

class Hexapod
{
public:
    static constexpr uint8_t NUM_LEGS = 6;
    static constexpr uint8_t SERVOS_PER_LEG = 3;

    // Servo channel mapping (view from top, hexapod facing forward)
    // PCA9685 #1 (0x40): FR(0,1,2), FL(3,4,5), MR(6,7,8), unused(9-15)
    // PCA9685 #2 (0x41): ML(0,1,2), BR(3,4,5), BL(6,7,8), unused(9-15)
    enum ServoChannels
    {
        // Front Right leg (servo indices 0-2 -> PCA9685 #1 channels 0-2)
        FR_COXA = 0,
        FR_FEMUR = 1,
        FR_TIBIA = 2,
        // Front Left leg (servo indices 3-5 -> PCA9685 #1 channels 3-5)
        FL_COXA = 3,
        FL_FEMUR = 4,
        FL_TIBIA = 5,
        // Middle Right leg (servo indices 6-8 -> PCA9685 #1 channels 6-8)
        MR_COXA = 6,
        MR_FEMUR = 7,
        MR_TIBIA = 8,
        // Middle Left leg (servo indices 9-11 -> PCA9685 #2 channels 0-2)
        ML_COXA = 9,
        ML_FEMUR = 10,
        ML_TIBIA = 11,
        // Back Right leg (servo indices 12-14 -> PCA9685 #2 channels 3-5)
        BR_COXA = 12,
        BR_FEMUR = 13,
        BR_TIBIA = 14,
        // Back Left leg (servo indices 15-17 -> PCA9685 #2 channels 6-8)
        BL_COXA = 15,
        BL_FEMUR = 16,
        BL_TIBIA = 17
    };

    Hexapod();
    ~Hexapod();

    bool init();
    void cleanup();
    bool homePosition();

    // Movement functions
    bool startAutonomousMovement();
    void stopMovement();
    bool moveForward();
    bool turnLeft();
    bool turnRight();
    bool stepBack();
    bool crawlSideways(); // New sideways crawling movement

    // Update hexapod state (call this in main loop)
    void update(double time_step);

    // Manual servo control functions
    bool setServoAngles(uint8_t leg_index, uint16_t coxa_us, uint16_t femur_us, uint16_t tibia_us);

private:
    PCA9685 pwm_controller_;
    Ultrasonic ultrasonic_;
    bool initialized_;
    bool moving_;

    // Movement state
    enum MovementState
    {
        STANDING,
        WALKING_FORWARD,
        TURNING_LEFT,
        TURNING_RIGHT,
        BACKING_UP,
        CRAWLING_SIDEWAYS,
        PAUSED_FOR_OBSTACLE
    };

    MovementState current_state_;
    std::chrono::steady_clock::time_point last_step_time_;
    std::chrono::steady_clock::time_point obstacle_pause_start_;
    int step_phase_; // 0 or 1 for tripod gait phases

    // Predefined servo positions (in microseconds)
    struct ServoPositions
    {
        uint16_t coxa;
        uint16_t femur;
        uint16_t tibia;
    };

    // Servo positions for natural, stable walking - FIXED servo direction for forward movement
    static constexpr ServoPositions standing_position_ = {1500, 1550, 1450}; // Stable stance - slight body lift
    static constexpr ServoPositions lift_position_ = {1500, 1350, 1200};     // Higher leg lift for clearance
    static constexpr ServoPositions forward_position_ = {1750, 1580, 1480};  // Forward step - FIXED direction (higher value = forward)
    static constexpr ServoPositions back_position_ = {1250, 1580, 1480};     // Back step - FIXED direction (lower value = backward)

    // Enhanced sideways positions for effective obstacle avoidance
    static constexpr ServoPositions sideways_lift = {1500, 1300, 1150};  // High lift for obstacle clearance
    static constexpr ServoPositions sideways_reach = {1680, 1400, 1300}; // Extended sideways reach for avoidance

    // Smoother turning positions
    static constexpr ServoPositions turn_left_pos = {1400, 1550, 1450};  // Gradual left turn
    static constexpr ServoPositions turn_right_pos = {1600, 1550, 1450}; // Gradual right turn

    // Tripod groups for alternating gait
    static constexpr uint8_t tripod_group_a_[3] = {1, 2, 5}; // FL, MR, BL
    static constexpr uint8_t tripod_group_b_[3] = {0, 3, 4}; // FR, ML, BR

    // Distance thresholds for obstacle avoidance
    static constexpr double obstacle_threshold_cm_ = 20.0;
    static constexpr double safe_distance_cm_ = 30.0;

    // Map leg index to servo channels
    struct LegServos
    {
        uint8_t coxa_channel;
        uint8_t femur_channel;
        uint8_t tibia_channel;
    };

    std::array<LegServos, 6> leg_servo_mapping_;

    // Initialize servo mapping
    void initializeServoMapping();

    // Manual movement functions
    void performTripodStep();
    void setTripodGroup(const uint8_t group[], const ServoPositions &position);
    void setAllLegsPosition(const ServoPositions &position);

    // Obstacle avoidance logic
    bool isObstacleDetected();
    void handleObstacleAvoidance();
};

#endif // HEXAPOD_HPP
