#include <iostream>
#include <cstdlib>
#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

#include "td3learn/config.hpp"
#include "td3learn/agent.hpp"
#include "td3learn/environment.hpp"
#include "td3learn/hexapod.hpp"
#include "td3learn/tidl.hpp"
#include "td3learn/utils.hpp"

using namespace td3learn;
using namespace td3learn::utils;

// Global flag for running state (modified by signal handler)
static std::atomic<bool> g_running = true;

// Signal handler to catch Ctrl+C
void signalHandler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM)
    {
        std::cout << "\nShutting down...\n";
        g_running = false;
    }
}

// Command line options
struct Options
{
    std::string model_path;
    std::string config_path = "configs/default.yaml";
    std::string tidl_path;
    bool use_hardware = false;
    bool use_tidl = false;
    bool use_simulation = true;
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
        else if (arg == "--tidl" && i + 1 < argc)
        {
            opts.tidl_path = argv[++i];
            opts.use_tidl = true;
        }
        else if (arg == "--hardware")
        {
            opts.use_hardware = true;
            opts.use_simulation = false;
        }
        else if (arg == "--simulation")
        {
            opts.use_simulation = true;
            opts.use_hardware = false;
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
              << "  --model PATH       Model path\n"
              << "  --config FILE      Config file (default: configs/default.yaml)\n"
              << "  --tidl PATH        Use TIDL model from PATH\n"
              << "  --hardware         Use real hardware\n"
              << "  --simulation       Use simulation mode (default)\n"
              << "  --loglevel LEVEL   Set log level (debug, info, warning, error)\n"
              << "  --help             Show this help message\n";
}

int main(int argc, char **argv)
{
    // Install signal handler
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Parse command line options
    Options opts = parseArgs(argc, argv);

    if (opts.help || opts.model_path.empty())
    {
        printHelp(argv[0]);
        return opts.help ? 0 : 1;
    }

    // Initialize logger
    Logger::init("td3learn_run", opts.log_level);
    Logger::info("TD3Learn runtime starting");

    try
    {
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

        // Override environment type based on CLI args
        if (opts.use_hardware)
        {
            config.environment.type = "real";
            Logger::info("Using real hardware");
        }
        else
        {
            config.environment.type = "simulation";
            Logger::info("Using simulation mode");
        }

        // Override device based on TIDL availability
        if (opts.use_tidl)
        {
            config.td3.device = "tidl";
            config.tidl.model_path = opts.tidl_path;
            Logger::info("Using TIDL acceleration");
        }

        // Create agent
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
            Logger::error("Failed to load model");
            return 1;
        }

        // Create environment
        auto env = Environment::create(config.environment.type, config.environment);
        if (!env)
        {
            Logger::error("Failed to create environment");
            return 1;
        }

        // Initialize environment
        if (env->init() != Result::SUCCESS)
        {
            Logger::error("Failed to initialize environment");
            return 1;
        }

        // Reset environment to start state
        State state = env->reset();

        // Main control loop
        Logger::info("Starting control loop");
        int step = 0;
        while (g_running)
        {
            // Get action from policy (no exploration noise in deployment)
            Action action = agent->selectAction(state, false);

            // Apply action to environment
            State next_state;
            Scalar reward;
            bool done;
            std::string info;

            Result step_result = env->step(action, next_state, reward, done, info);
            if (step_result != Result::SUCCESS)
            {
                Logger::error("Environment step failed: " + resultToString(step_result));
                break;
            }

            // Update state
            state = next_state;

            // Log periodically
            if (++step % 100 == 0)
            {
                Logger::info("Running: completed " + std::to_string(step) + " steps");
            }

            // Sleep to keep loop rate reasonable for debugging
            if (opts.log_level == Logger::Level::DEBUG)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }

        Logger::info("Control loop terminated after " + std::to_string(step) + " steps");
    }
    catch (const std::exception &e)
    {
        Logger::error("Exception: " + std::string(e.what()));
        return 1;
    }

    Logger::info("TD3Learn runtime completed");
    return 0;
}
