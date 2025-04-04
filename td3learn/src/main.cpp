#include <iostream>
#include <cstdlib>
#include <string>
#include <memory>

#include "td3learn/config.hpp"
#include "td3learn/agent.hpp"
#include "td3learn/environment.hpp"
#include "td3learn/trainer.hpp"
#include "td3learn/utils.hpp"

using namespace td3learn;
using namespace td3learn::utils;

// Parse command line arguments
bool parseArgs(int argc, char **argv, std::string &config_path, std::string &mode)
{
    mode = "train";
    config_path = "config.yaml";

    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc)
        {
            config_path = argv[++i];
        }
        else if (arg == "--train")
        {
            mode = "train";
        }
        else if (arg == "--eval")
        {
            mode = "eval";
        }
        else if (arg == "--deploy")
        {
            mode = "deploy";
        }
        else if (arg == "--help")
        {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --config <file>  Config file (default: config.yaml)\n"
                      << "  --train          Train model (default)\n"
                      << "  --eval           Evaluate model\n"
                      << "  --deploy         Deploy model for inference\n"
                      << "  --help           Show this help message\n";
            return false;
        }
        else
        {
            std::cout << "Unknown argument: " << arg << "\n";
            std::cout << "Use --help for usage information\n";
            return false;
        }
    }
    return true;
}

int main(int argc, char **argv)
{
    // Parse command line arguments
    std::string config_path, mode;
    if (!parseArgs(argc, argv, config_path, mode))
    {
        return 1;
    }

    // Initialize logger
    Logger::init("td3learn", Logger::Level::INFO);
    Logger::info("TD3Learn starting in " + mode + " mode");

    try
    {
        // Load configuration
        Config config;
        Result result = config.loadFromFile(config_path);
        if (result != Result::SUCCESS)
        {
            Logger::info("Using default configuration");
            config = Config::createDefault();
        }
        else
        {
            Logger::info("Loaded configuration from " + config_path);
        }

        if (mode == "train")
        {
            // Create agent and environment
            auto agent = std::make_shared<TD3Agent>(config.td3);
            auto env_unique = Environment::create(config.environment.type, config.environment);

            if (!env_unique)
            {
                Logger::error("Failed to create environment");
                return 1;
            }

            // Convert unique_ptr to shared_ptr for the trainer
            std::shared_ptr<Environment> env = std::move(env_unique);

            // Initialize agent
            if (agent->init() != Result::SUCCESS)
            {
                Logger::error("Failed to initialize agent");
                return 1;
            }

            // Create and run trainer
            Trainer trainer(agent, env, config.training);
            if (trainer.init() != Result::SUCCESS)
            {
                Logger::error("Failed to initialize trainer");
                return 1;
            }

            // Set callback to log progress
            trainer.setCallback([](int epoch, int step, const std::vector<Scalar> &metrics)
                                {
                if (metrics.size() >= 3) {
                    Logger::info("Epoch " + std::to_string(epoch) + 
                                 ", Step " + std::to_string(step) + 
                                 ", Critic Loss: " + std::to_string(metrics[0]) +
                                 ", Actor Loss: " + std::to_string(metrics[1]) +
                                 ", Avg Reward: " + std::to_string(metrics[2]));
                } });

            // Start training
            Logger::info("Starting training for " + std::to_string(config.training.max_epochs) + " epochs");
            result = trainer.train(config.training.max_epochs);

            if (result == Result::SUCCESS)
            {
                // Export model
                Logger::info("Training completed, exporting model");
                result = trainer.exportModel(config.training.save_path);

                if (result == Result::SUCCESS)
                {
                    Logger::info("Model exported to " + config.training.save_path);

                    // Export TIDL model if requested
                    if (config.tidl.model_path.empty() == false)
                    {
                        result = trainer.exportTIDLModel(config.tidl.model_path, config.tidl);
                        if (result == Result::SUCCESS)
                        {
                            Logger::info("TIDL model exported to " + config.tidl.model_path);
                        }
                        else
                        {
                            Logger::error("Failed to export TIDL model");
                        }
                    }
                }
                else
                {
                    Logger::error("Failed to export model");
                }
            }
            else
            {
                Logger::error("Training failed with result: " + resultToString(result));
            }
        }
        else if (mode == "eval")
        {
            // Evaluation mode implementation
            Logger::info("Evaluation mode not fully implemented yet");
            // TODO: Implement evaluation mode
        }
        else if (mode == "deploy")
        {
            // Deployment mode implementation
            Logger::info("Deployment mode not fully implemented yet");
            // TODO: Implement deployment mode
        }
    }
    catch (const std::exception &e)
    {
        Logger::error("Exception: " + std::string(e.what()));
        return 1;
    }

    Logger::info("TD3Learn completed successfully");
    return 0;
}
