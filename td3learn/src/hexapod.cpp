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

// Include the user space hexapod interface
#include "hexapod.hpp"

namespace td3learn
{

    // HexapodEnvironment Implementation class
    class HexapodEnvironmentImpl
    {
    public:
        HexapodEnvironmentImpl(const HexapodEnvironmentConfig &config)
            : config(config),
              simulation_mode(config.type == "simulation"),
              last_action(config.action_dim),
              step_counter(0)
        {
            // Create hexapod instance
            hexapod = std::make_unique<hexapod::Hexapod>();

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
                state_dim += hexapod::Config::NUM_LEGS * 3;
            }

            // Action dimension - one value per joint
            action_dim = hexapod::Config::NUM_LEGS * 3; // 6 legs x 3 joints

            utils::Logger::info("Created HexapodEnvironment with state_dim=" +
                                std::to_string(state_dim) + ", action_dim=" +
                                std::to_string(action_dim));
        }

        ~HexapodEnvironmentImpl()
        {
            // Clean up hexapod resources if initialized
            if (hexapod && initialized)
            {
                hexapod->cleanup();
            }
        }

        Result init()
        {
            try
            {
                // Initialize hexapod
                if (!hexapod->init())
                {
                    utils::Logger::error("Failed to initialize hexapod: " +
                                         hexapod->getLastErrorMessage());
                    return Result::ERROR_HARDWARE;
                }

                utils::Logger::info(simulation_mode ? "Connected to hexapod simulation" : "Connected to hexapod hardware");

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
                initialized = true;
                return Result::SUCCESS;
            }
            catch (const std::exception &e)
            {
                utils::Logger::error("Exception initializing hexapod: " + std::string(e.what()));
                return Result::ERROR_INITIALIZATION;
            }
        }

        State reset()
        {
            step_counter = 0;

            // Center all legs
            if (!hexapod->centerAll())
            {
                utils::Logger::warning("Failed to center legs on reset: " +
                                       hexapod->getLastErrorMessage());
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

            // Would need to reinitialize hexapod with different mode
            // This is a simplification - in real implementation, we would
            // recreate the hexapod with the new mode
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
        bool initialized = false;

        // Hexapod interface
        std::unique_ptr<hexapod::Hexapod> hexapod;

        // Simulation flag
        bool simulation_mode;

        // State and action information
        int state_dim;
        int action_dim;
        State current_state;
        Action action_low;
        Action action_high;
        Action last_action;

        // Episode tracking
        int step_counter;

        // Update the state vector from sensor data
        void updateStateVector()
        {
            int idx = 0;

            if (config.use_imu)
            {
                // Get IMU data
                hexapod::ImuData imu_data;

                if (!hexapod->getImuData(imu_data))
                {
                    utils::Logger::warning("Failed to read IMU data: " +
                                           hexapod->getLastErrorMessage());
                    // Use zeros for IMU data
                    imu_data = hexapod::ImuData();
                }

                // Normalize IMU data into state vector
                // Acceleration: ±2g range -> ±1.0 value
                current_state[idx++] = imu_data.getAccelX() / 2.0f;
                current_state[idx++] = imu_data.getAccelY() / 2.0f;
                current_state[idx++] = imu_data.getAccelZ() / 2.0f;

                // Gyroscope: ±250°/s range -> ±1.0 value
                current_state[idx++] = imu_data.getGyroX() / 250.0f;
                current_state[idx++] = imu_data.getGyroY() / 250.0f;
                current_state[idx++] = imu_data.getGyroZ() / 250.0f;
            }

            if (config.use_leg_positions)
            {
                // Get leg positions
                for (int leg = 0; leg < hexapod::Config::NUM_LEGS; leg++)
                {
                    hexapod::LegPosition leg_pos;

                    if (!hexapod->getLegPosition(leg, leg_pos))
                    {
                        utils::Logger::warning("Failed to get position for leg " +
                                               std::to_string(leg) + ": " +
                                               hexapod->getLastErrorMessage());
                        // Use zeros for leg position
                        leg_pos = hexapod::LegPosition();
                    }

                    // Normalize joint angles to [-1, 1] range
                    // Assuming 90 degrees is the max angle in each direction
                    current_state[idx++] = leg_pos.getHip() / 90.0f;
                    current_state[idx++] = leg_pos.getKnee() / 90.0f;
                    current_state[idx++] = leg_pos.getAnkle() / 90.0f;
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
            for (int leg = 0; leg < hexapod::Config::NUM_LEGS; leg++)
            {
                // Get joint angles from action vector
                // Convert normalized [-1, 1] to servo angles [-90, 90]
                hexapod::LegPosition leg_pos;
                leg_pos.leg_num = leg;

                int base_idx = leg * 3;
                leg_pos.setHip(static_cast<int16_t>(action[base_idx] * 90.0f));
                leg_pos.setKnee(static_cast<int16_t>(action[base_idx + 1] * 90.0f));
                leg_pos.setAnkle(static_cast<int16_t>(action[base_idx + 2] * 90.0f));

                // Apply to hardware
                if (!hexapod->setLegPosition(leg, leg_pos))
                {
                    utils::Logger::warning("Failed to set leg " + std::to_string(leg) +
                                           " position: " + hexapod->getLastErrorMessage());
                }
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
            if (config.use_imu)
            {
                // In a real system, we would calculate velocity from IMU data
                // This is a placeholder
                velocity_reward = 1.0f;
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
                // In a real system, would calculate based on actual IMU readings
                stability_reward = 1.0f; // Placeholder
            }

            // Smoothness reward - penalize sudden changes in action
            Scalar smoothness_reward = 0.0f;
            if (last_action.size() > 0) // Replace last_action.empty() with size check
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
