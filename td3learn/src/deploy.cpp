#include <iostream>
#include <cstdlib>
#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

#include "td3learn/config.hpp"
#include "td3learn/agent.hpp"
#include "td3learn/environment.hpp"
#include "td3learn/hexapod.hpp"
#include "td3learn/tidl.hpp"
#include "td3learn/utils.hpp"

using namespace td3learn;
using namespace td3learn::utils;

// Command line options
struct Options
{
    std::string model_path;
    std::string config_path = "configs/default.yaml";
    std::string output_path = "deploy";
    bool use_hardware = false;
    bool convert_to_tidl = false;
    bool verbose = false;
    Logger::Level log_level = Logger::Level::INFO;
    bool help = false;
};

// Parse command line arguments
Options parseArgs(int argc, char **argv)
{
    Options opts;

    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "--model" && i + 1 < argc)
        {
            opts.model_path = argv[++i];
        }
        else if (arg == "--config" && i + 1 < argc)
        {
            opts.config_path = argv[++i];
        }
        else if (arg == "--output" && i + 1 < argc)
        {
            opts.output_path = argv[++i];
        }
        else if (arg == "--hardware")
        {
            opts.use_hardware = true;
        }
        else if (arg == "--tidl")
        {
            opts.convert_to_tidl = true;
        }
        else if (arg == "--verbose")
        {
            opts.verbose = true;
        }
        else if (arg == "--loglevel" && i + 1 < argc)
        {
            std::string level = argv[++i];
            if (level == "debug")
            {
                opts.log_level = Logger::Level::DEBUG;
            }
            else if (level == "info")
            {
                opts.log_level = Logger::Level::INFO;
            }
            else if (level == "warning")
            {
                opts.log_level = Logger::Level::WARNING;
            }
            else if (level == "error")
            {
                opts.log_level = Logger::Level::ERROR;
            }
        }
        else if (arg == "--help")
        {
            opts.help = true;
        }
    }

    return opts;
}

// Print usage information
void printHelp(const char *program)
{
    std::cout << "Usage: " << program << " [options]\n"
              << "Options:\n"
              << "  --model PATH       Source model path\n"
              << "  --config FILE      Config file (default: configs/default.yaml)\n"
              << "  --output DIR       Output directory (default: deploy)\n"
              << "  --hardware         Target real hardware (default: simulation)\n"
              << "  --tidl             Convert to TIDL format for acceleration\n"
              << "  --verbose          Show detailed output\n"
              << "  --loglevel LEVEL   Set log level (debug, info, warning, error)\n"
              << "  --help             Show this help message\n";
}

int main(int argc, char **argv)
{
    // Parse command line options
    Options opts = parseArgs(argc, argv);

    if (opts.help || opts.model_path.empty())
    {
        printHelp(argv[0]);
        return opts.help ? 0 : 1;
    }

    // Initialize logger
    Logger::init("td3learn_deploy", opts.log_level, true, "td3learn_deploy.log");
    Logger::info("TD3Learn deployment tool starting");

    try
    {
        // Check if model exists
        if (!std::filesystem::exists(opts.model_path))
        {
            Logger::error("Model not found: " + opts.model_path);
            return 1;
        }

        // Load configuration
        Config config;
        Result result = config.loadFromFile(opts.config_path);
        if (result != Result::SUCCESS)
        {
            Logger::info("Using default configuration");
            config = Config::createDefault();
        }
        else
        {
            Logger::info("Loaded configuration from " + opts.config_path);
        }

        // Create output directory
        if (!std::filesystem::exists(opts.output_path))
        {
            if (!std::filesystem::create_directories(opts.output_path))
            {
                Logger::error("Failed to create output directory: " + opts.output_path);
                return 1;
            }
        }

        // Create agent for deployment
        auto agent = std::make_shared<TD3Agent>(config.td3);

        // Initialize agent
        if (agent->init() != Result::SUCCESS)
        {
            Logger::error("Failed to initialize agent");
            return 1;
        }

        // Load trained model
        Logger::info("Loading model from: " + opts.model_path);
        if (agent->loadModels(opts.model_path) != Result::SUCCESS)
        {
            Logger::error("Failed to load models");
            return 1;
        }

        // Save model to output directory
        std::string output_model_path = opts.output_path + "/model";
        Logger::info("Saving deployment model to: " + output_model_path);
        if (agent->saveModels(output_model_path) != Result::SUCCESS)
        {
            Logger::error("Failed to save deployment model");
            return 1;
        }

        // Convert to TIDL model if requested
        if (opts.convert_to_tidl)
        {
            Logger::info("Converting model to TIDL format");
            std::string tidl_path = opts.output_path + "/tidl_model";

            if (agent->exportTIDLModel(tidl_path, config.tidl) != Result::SUCCESS)
            {
                Logger::warning("TIDL model conversion failed - using CPU model");
            }
            else
            {
                Logger::info("TIDL model saved to: " + tidl_path);
            }
        }

        // Create environment configuration file for deployment
        std::string env_config_path = opts.output_path + "/environment.yaml";
        std::ofstream env_file(env_config_path);
        if (env_file.is_open())
        {
            env_file << "# Hexapod environment configuration\n"
                     << "[env]\n"
                     << "type = " << (opts.use_hardware ? "real" : "simulation") << "\n"
                     << "max_episode_steps = 0  # No step limit for deployment\n"
                     << "\n"
                     << "[hexapod]\n"
                     << "use_imu = " << (opts.use_hardware ? "true" : "false") << "\n"
                     << "use_leg_positions = true\n"
                     << "step_time = 0.05\n";
            env_file.close();
            Logger::info("Environment configuration saved to: " + env_config_path);
        }

        // Create deployment script
        std::string run_script_path = opts.output_path + "/run.sh";
        std::ofstream script_file(run_script_path);
        if (script_file.is_open())
        {
            script_file << "#!/bin/bash\n"
                        << "# TD3Learn deployment runtime\n\n"
                        << "MODEL_DIR=\"$(dirname \"$0\")\"\n"
                        << "BUILD_DIR=\"$MODEL_DIR/../build\"\n\n"
                        << "if [ ! -f \"$BUILD_DIR/td3learn_run\" ]; then\n"
                        << "    echo \"Error: Runtime executable not found\"\n"
                        << "    exit 1\n"
                        << "fi\n\n"
                        << "# Run the model\n"
                        << "\"$BUILD_DIR/td3learn_run\" \\\n"
                        << "    --model \"$MODEL_DIR/model\" \\\n"
                        << "    --config \"$MODEL_DIR/environment.yaml\" \\\n";

            if (opts.convert_to_tidl)
            {
                script_file << "    --tidl \"$MODEL_DIR/tidl_model\" \\\n";
            }

            if (opts.use_hardware)
            {
                script_file << "    --hardware\n";
            }
            else
            {
                script_file << "    --simulation\n";
            }

            script_file.close();

            // Make the script executable
            std::filesystem::permissions(
                run_script_path,
                std::filesystem::perms::owner_exec |
                    std::filesystem::perms::group_exec |
                    std::filesystem::status(run_script_path).permissions());

            Logger::info("Runtime script saved to: " + run_script_path);
        }

        Logger::info("Deployment completed successfully");
    }
    catch (const std::exception &e)
    {
        Logger::error("Error during deployment: " + std::string(e.what()));
        return 1;
    }

    return 0;
}
