#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <iomanip>
#include <vector>

#include "td3learn/config.hpp"
#include "td3learn/agent.hpp"
#include "td3learn/environment.hpp"
#include "td3learn/trainer.hpp"
#include "td3learn/utils.hpp"

using namespace td3learn;
using namespace td3learn::utils;

// Simple progress bar function
void printProgressBar(int current, int total, int width = 50)
{
    float progress = static_cast<float>(current) / static_cast<float>(total);
    int pos = static_cast<int>(width * progress);

    std::cout << "[";
    for (int i = 0; i < width; ++i)
    {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << std::setw(3) << static_cast<int>(progress * 100.0) << "%\r";
    std::cout.flush();
}

int main(int argc, char **argv)
{
    // Initialize logger
    Logger::init("td3_simple", Logger::Level::INFO);

    try
    {
        // Create a default configuration
        Config config = Config::createDefault();

        // Set up simulation environment
        config.environment.type = "simulation";
        config.environment.max_episode_steps = 500;

        // Create a hexapod-specific environment config
        HexapodEnvironmentConfig hexapod_config;
        hexapod_config.type = "simulation";
        hexapod_config.use_imu = true;
        hexapod_config.use_leg_positions = true;
        hexapod_config.step_time = 0.05;

        // Set the environment config in the main config
        config.environment = hexapod_config;

        // Configure TD3 parameters
        config.td3.state_dim = 24;  // 6 legs x 3 joints + IMU (6 values)
        config.td3.action_dim = 18; // 6 legs x 3 joints
        config.td3.gamma = 0.99;
        config.td3.tau = 0.005;
        config.td3.batch_size = 128;
        config.td3.exploration_noise = true;
        config.td3.exploration_noise_std = 0.1;

        // Configure training parameters
        config.training.max_epochs = 10;
        config.training.steps_per_epoch = 1000;
        config.training.evaluate_every = 2;
        config.training.save_path = "models/td3_simple";

        // Create file logger
        Logger::init("td3_simple", Logger::Level::INFO, true, "td3_simple.log");
        Logger::info("Starting TD3 simple example");

        // Create agent
        auto agent = std::make_shared<TD3Agent>(config.td3);
        if (agent->init() != Result::SUCCESS)
        {
            Logger::error("Failed to initialize agent");
            return 1;
        }

        // Create environment
        auto env = Environment::create(config.environment.type, config.environment);
        if (!env)
        {
            Logger::error("Failed to create environment");
            return 1;
        }

        if (env->init() != Result::SUCCESS)
        {
            Logger::error("Failed to initialize environment");
            return 1;
        }

        // Create trainer
        Trainer trainer(agent, env, config.training);
        if (trainer.init() != Result::SUCCESS)
        {
            Logger::error("Failed to initialize trainer");
            return 1;
        }

        // Setup progress callback
        trainer.setCallback([](int epoch, int step, const std::vector<Scalar> &metrics)
                            {
            static int last_step = -1;
            
            if (step != last_step) {
                std::cout << "Epoch " << (epoch+1) << ", Step " << step;
                if (metrics.size() >= 3) {
                    std::cout << " | Critic: " << std::fixed << std::setprecision(4) << metrics[0]
                              << ", Actor: " << metrics[1]
                              << ", Reward: " << metrics[2];
                }
                std::cout << std::endl;
                last_step = step;
                printProgressBar(step, 1000);
            } });

        // Run training
        Logger::info("Starting training...");
        Result result = trainer.train(config.training.max_epochs);

        if (result == Result::SUCCESS)
        {
            Logger::info("Training completed successfully");

            // Export model
            Logger::info("Exporting model...");
            result = trainer.exportModel(config.training.save_path);

            if (result == Result::SUCCESS)
            {
                Logger::info("Model exported to " + config.training.save_path);
            }
            else
            {
                Logger::error("Failed to export model");
            }
        }
        else
        {
            Logger::error("Training failed: " + resultToString(result));
        }

        return 0;
    }
    catch (const std::exception &e)
    {
        Logger::error("Exception: " + std::string(e.what()));
        return 1;
    }
}
