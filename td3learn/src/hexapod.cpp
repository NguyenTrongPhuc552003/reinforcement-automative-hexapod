#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <thread>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "td3learn/hexapod.hpp"
#include "td3learn/utils.hpp"

// Include robot-specific driver interface with correct path
extern "C"
{
#include "hexapod.h" // Kernel driver interface from driver/inc
}

namespace td3learn
{

    // HexapodEnvironment Implementation class
    class HexapodEnvironmentImpl
    {
    public:
        HexapodEnvironmentImpl(const HexapodEnvironmentConfig &config)
            : config(config),
              device_fd(-1),
              simulation_mode(config.type == "simulation"),
              last_action(config.action_dim),
              step_counter(0)
        {

            // Initialize state and action dimensions based on robot configuration
            state_dim = 0;

            // IMU data: 3 accel + 3 gyro = 6 values
            if (config.use_imu)
            {
                state_dim += 6;
            }

            // Leg positions: 6 legs x 3 joints = 18 values
            if (config.use_leg_positions)
            {
                state_dim += NUM_LEGS * 3;
            }

            // Action dimension - one value per joint
            action_dim = NUM_LEGS * 3; // 6 legs x 3 joints

            utils::Logger::info("Created HexapodEnvironment with state_dim=" +
                                std::to_string(state_dim) + ", action_dim=" +
                                std::to_string(action_dim));
        }

        ~HexapodEnvironmentImpl()
        {
            if (device_fd >= 0)
            {
                close(device_fd);
                utils::Logger::info("Closed hexapod device");
            }
        }

        Result init()
        {
            if (!simulation_mode)
            {
                // Open device file
                device_fd = open("/dev/hexapod", O_RDWR);
                if (device_fd < 0)
                {
                    utils::Logger::error("Failed to open hexapod device: " + std::string(strerror(errno)));
                    return Result::ERROR_HARDWARE;
                }

                // Center all legs at startup for safety
                if (ioctl(device_fd, HEXAPOD_IOCTL_CENTER_ALL) < 0)
                {
                    utils::Logger::error("Failed to center legs: " + std::string(strerror(errno)));
                    close(device_fd);
                    device_fd = -1;
                    return Result::ERROR_HARDWARE;
                }

                utils::Logger::info("Connected to hexapod hardware");
            }
            else
            {
                utils::Logger::info("Using hexapod simulation mode");
                initSimulation();
            }

            // Initialize state vector
            current_state.resize(state_dim);
            std::fill(current_state.begin(), current_state.end(), 0.0f);

            // Set action limits
            action_low.resize(action_dim);
            action_high.resize(action_dim);
            std::fill(action_low.begin(), action_low.end(), -1.0f); // Normalized to [-1, 1]
            std::fill(action_high.begin(), action_high.end(), 1.0f);

            last_action.resize(action_dim);
            std::fill(last_action.begin(), last_action.end(), 0.0f);

            step_counter = 0;
            return Result::SUCCESS;
        }

        State reset()
        {
            step_counter = 0;

            if (!simulation_mode && device_fd >= 0)
            {
                // Center all legs
                if (ioctl(device_fd, HEXAPOD_IOCTL_CENTER_ALL) < 0)
                {
                    utils::Logger::warning("Failed to center legs on reset: " + std::string(strerror(errno)));
                }
            }

            // Re-initialize state vector
            std::fill(current_state.begin(), current_state.end(), 0.0f);
            std::fill(last_action.begin(), last_action.end(), 0.0f);

            // Update state with current sensor data
            updateStateVector();

            return current_state;
        }

        Result step(const Action &action, State &next_state, Scalar &reward, bool &done, std::string &info)
        {
            if (action.size() != action_dim)
            {
                return Result::ERROR_INVALID_ARGUMENT;
            }

            // Apply actions to robot
            Result result = applyAction(action);
            if (result != Result::SUCCESS)
            {
                return result;
            }

            // Wait for step duration
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(config.step_time * 1000)));

            // Update state with new sensor readings
            updateStateVector();
            next_state = current_state;

            // Calculate reward
            reward = calculateReward(action);

            // Increment step counter and check if done
            step_counter++;
            done = (step_counter >= config.max_episode_steps);

            // Store action for next reward calculation
            last_action = action;

            info = ""; // No additional info for now
            return Result::SUCCESS;
        }

        int getStateDimension() const
        {
            return state_dim;
        }

        int getActionDimension() const
        {
            return action_dim;
        }

        void getActionLimits(State &low, State &high) const
        {
            low = action_low;
            high = action_high;
        }

        Result setSimulationMode(bool simulation)
        {
            if (simulation_mode == simulation)
            {
                return Result::SUCCESS; // No change needed
            }

            if (simulation)
            {
                // Switching from hardware to simulation
                if (device_fd >= 0)
                {
                    close(device_fd);
                    device_fd = -1;
                }
                initSimulation();
            }
            else
            {
                // Switching from simulation to hardware
                device_fd = open("/dev/hexapod", O_RDWR);
                if (device_fd < 0)
                {
                    utils::Logger::error("Failed to open hexapod device: " + std::string(strerror(errno)));
                    return Result::ERROR_HARDWARE;
                }
            }

            simulation_mode = simulation;
            return Result::SUCCESS;
        }

        bool isSimulation() const
        {
            return simulation_mode;
        }

    private:
        // Configuration
        HexapodEnvironmentConfig config;

        // Device file descriptor for hardware control
        int device_fd;

        // Simulation flag
        bool simulation_mode;

        // State and action information
        int state_dim;
        int action_dim;
        State current_state;
        Action action_low;
        Action action_high;
        Action last_action;

        // Simulation state variables
        struct
        {
            struct hexapod_leg_position legs[NUM_LEGS];
            struct hexapod_imu_data imu;
        } sim_state;

        // Episode tracking
        int step_counter;

        // Initialize simulation state
        void initSimulation()
        {
            // Set default leg positions
            for (int i = A; i < NUM_LEGS; i++)
            {
                sim_state.legs[i].hip = 0;
                sim_state.legs[i].knee = 0;
                sim_state.legs[i].ankle = 0;
            }

            // Set default IMU values
            sim_state.imu.accel_x = 0;
            sim_state.imu.accel_y = 0;
            sim_state.imu.accel_z = 16384; // 1g in the z direction
            sim_state.imu.gyro_x = 0;
            sim_state.imu.gyro_y = 0;
            sim_state.imu.gyro_z = 0;
        }

        // Update the state vector from sensor data
        void updateStateVector()
        {
            int idx = 0;

            if (config.use_imu)
            {
                // Get IMU data
                struct hexapod_imu_data imu_data;

                if (!simulation_mode && device_fd >= 0)
                {
                    if (ioctl(device_fd, HEXAPOD_IOCTL_GET_IMU, &imu_data) < 0)
                    {
                        utils::Logger::warning("Failed to read IMU data: " + std::string(strerror(errno)));
                        // Use previous data
                        imu_data = sim_state.imu;
                    }
                }
                else
                {
                    imu_data = sim_state.imu;
                }

                // Normalize IMU data into state vector
                // Acceleration: ±2g range -> ±1.0 value
                current_state[idx++] = imu_data.accel_x / 16384.0f;
                current_state[idx++] = imu_data.accel_y / 16384.0f;
                current_state[idx++] = imu_data.accel_z / 16384.0f;

                // Gyroscope: ±250°/s range -> ±1.0 value
                current_state[idx++] = imu_data.gyro_x / 131.0f;
                current_state[idx++] = imu_data.gyro_y / 131.0f;
                current_state[idx++] = imu_data.gyro_z / 131.0f;

                // Store data in simulation state
                sim_state.imu = imu_data;
            }

            if (config.use_leg_positions)
            {
                // Get leg positions
                for (int leg = 0; leg < NUM_LEGS; leg++)
                {
                    struct hexapod_leg_position leg_pos;

                    if (!simulation_mode && device_fd >= 0)
                    {
                        // In real hardware, we're accessing the cached positions inside the driver
                        // This would need to match with a driver-side implementation to get current leg positions
                        leg_pos = sim_state.legs[leg];
                    }
                    else
                    {
                        leg_pos = sim_state.legs[leg];
                    }

                    // Normalize joint angles to [-1, 1] range
                    // Assuming 90 degrees is the max angle in each direction
                    current_state[idx++] = leg_pos.hip / 90.0f;
                    current_state[idx++] = leg_pos.knee / 90.0f;
                    current_state[idx++] = leg_pos.ankle / 90.0f;
                }
            }
        }

        // Apply an action to the robot
        Result applyAction(const Action &action)
        {
            // Check that action is the right size
            if (action.size() != action_dim)
            {
                return Result::ERROR_INVALID_ARGUMENT;
            }

            // Apply actions to each leg
            for (int leg = 0; leg < NUM_LEGS; leg++)
            {
                // Get joint angles from action vector
                // Convert normalized [-1, 1] to servo angles [-90, 90]
                struct hexapod_leg_position leg_pos;

                int base_idx = leg * 3;
                leg_pos.hip = static_cast<int16_t>(action[base_idx] * 90.0f);
                leg_pos.knee = static_cast<int16_t>(action[base_idx + 1] * 90.0f);
                leg_pos.ankle = static_cast<int16_t>(action[base_idx + 2] * 90.0f);

                // Apply to hardware or simulation
                if (!simulation_mode && device_fd >= 0)
                {
                    struct hexapod_leg_cmd cmd;
                    cmd.leg_num = leg;
                    cmd.position = leg_pos;

                    if (ioctl(device_fd, HEXAPOD_IOCTL_SET_LEG, &cmd) < 0)
                    {
                        utils::Logger::warning("Failed to set leg " + std::to_string(leg) +
                                               " position: " + std::string(strerror(errno)));
                    }
                }

                // Save to simulation state
                sim_state.legs[leg] = leg_pos;
            }

            return Result::SUCCESS;
        }

        // Calculate reward based on current state and action
        Scalar calculateReward(const Action &action)
        {
            Scalar reward = 0.0f;

            // Forward velocity reward component - would come from IMU or external measurement
            // This is a simplification - in a real system, we'd need actual velocity feedback
            Scalar velocity_reward = 0.0f;
            if (config.use_imu && simulation_mode)
            {
                // In simulation, we can derive a simple velocity reward
                // Assume the robot is moving in the direction of its body orientation
                velocity_reward = 1.0f; // Placeholder
            }

            // Energy efficiency reward component - penalize large/wasteful movements
            Scalar energy_penalty = 0.0f;
            for (size_t i = 0; i < action.size(); i++)
            {
                // Quadratic penalty for action magnitudes
                energy_penalty += action[i] * action[i];
            }
            energy_penalty /= action.size();

            // Stability reward component - based on IMU data
            Scalar stability_reward = 0.0f;
            if (config.use_imu)
            {
                // Reward level orientation (accel_z close to 1g, others close to 0)
                // and minimal rotational velocity
                if (!simulation_mode && device_fd >= 0)
                {
                    // We'd use real IMU data here
                    stability_reward = 1.0f; // Placeholder
                }
                else
                {
                    stability_reward = 1.0f; // Placeholder for simulation
                }
            }

            // Smoothness reward - penalize sudden changes in action
            Scalar smoothness_reward = 0.0f;
            if (!last_action.empty())
            {
                Scalar action_diff = 0.0f;
                for (size_t i = 0; i < action.size(); i++)
                {
                    Scalar diff = action[i] - last_action[i];
                    action_diff += diff * diff;
                }
                action_diff /= action.size();
                smoothness_reward = std::exp(-10.0f * action_diff); // High for small changes
            }

            // Combine rewards with weights
            reward = config.velocity_weight * velocity_reward -
                     config.energy_weight * energy_penalty +
                     config.stability_weight * stability_reward +
                     config.smoothness_weight * smoothness_reward;

            // Scale the final reward
            return reward * config.reward_scale;
        }
    };

    // HexapodEnvironment implementation methods
    HexapodEnvironment::HexapodEnvironment(const HexapodEnvironmentConfig &config)
        : Environment(config), impl(std::make_unique<HexapodEnvironmentImpl>(config)) {}

    HexapodEnvironment::~HexapodEnvironment() = default;

    Result HexapodEnvironment::init()
    {
        return impl->init();
    }

    State HexapodEnvironment::reset()
    {
        return impl->reset();
    }

    Result HexapodEnvironment::step(const Action &action, State &next_state,
                                    Scalar &reward, bool &done, std::string &info)
    {
        return impl->step(action, next_state, reward, done, info);
    }

    int HexapodEnvironment::getStateDimension() const
    {
        return impl->getStateDimension();
    }

    int HexapodEnvironment::getActionDimension() const
    {
        return impl->getActionDimension();
    }

    void HexapodEnvironment::getActionLimits(State &low, State &high) const
    {
        impl->getActionLimits(low, high);
    }

    Result HexapodEnvironment::setSimulationMode(bool simulation)
    {
        return impl->setSimulationMode(simulation);
    }

    bool HexapodEnvironment::isSimulation() const
    {
        return impl->isSimulation();
    }

} // namespace td3learn
