#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include "td3learn/config.hpp"
#include "td3learn/utils.hpp"

namespace td3learn
{

    // Simple YAML-like parser helpers
    namespace
    {
        // Trim whitespace from start/end of string
        std::string trim(const std::string &str)
        {
            size_t first = str.find_first_not_of(" \t\n\r");
            if (first == std::string::npos)
                return "";
            size_t last = str.find_last_not_of(" \t\n\r");
            return str.substr(first, last - first + 1);
        }

        // Parse a scalar value (string, number, bool)
        template <typename T>
        bool parseScalar(const std::string &value, T &result)
        {
            std::istringstream iss(value);
            iss >> result;
            return !iss.fail();
        }

        // Specialization for string
        template <>
        bool parseScalar<std::string>(const std::string &value, std::string &result)
        {
            result = value;
            return true;
        }

        // Specialization for boolean
        template <>
        bool parseScalar<bool>(const std::string &value, bool &result)
        {
            std::string lower = value;
            for (auto &c : lower)
                c = std::tolower(c);

            if (lower == "true" || lower == "yes" || lower == "1")
            {
                result = true;
                return true;
            }
            else if (lower == "false" || lower == "no" || lower == "0")
            {
                result = false;
                return true;
            }
            return false;
        }

        // Parse a vector of values [x, y, z]
        template <typename T>
        bool parseVector(const std::string &value, std::vector<T> &result)
        {
            result.clear();
            std::string trimmed = trim(value);

            // Check for array syntax [a, b, c]
            if (trimmed.empty() || trimmed[0] != '[' || trimmed[trimmed.size() - 1] != ']')
            {
                return false;
            }

            // Remove brackets
            trimmed = trimmed.substr(1, trimmed.size() - 2);

            // Split by commas
            std::istringstream iss(trimmed);
            std::string element;
            while (std::getline(iss, element, ','))
            {
                element = trim(element);
                T value;
                if (parseScalar(element, value))
                {
                    result.push_back(value);
                }
                else
                {
                    return false;
                }
            }

            return true;
        }
    }

    // Configuration implementation
    Result Config::loadFromFile(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file)
        {
            utils::Logger::error("Failed to open config file: " + filename);
            return Result::ERROR_INVALID_ARGUMENT;
        }

        std::map<std::string, std::string> values;
        std::string line;
        std::string current_section;

        while (std::getline(file, line))
        {
            // Skip empty lines and comments
            std::string trimmed = trim(line);
            if (trimmed.empty() || trimmed[0] == '#')
            {
                continue;
            }

            // Check for section header [section]
            if (trimmed[0] == '[' && trimmed[trimmed.size() - 1] == ']')
            {
                current_section = trimmed.substr(1, trimmed.size() - 2);
                continue;
            }

            // Parse key-value pairs
            size_t delimiter = trimmed.find('=');
            if (delimiter != std::string::npos)
            {
                std::string key = trim(trimmed.substr(0, delimiter));
                std::string value = trim(trimmed.substr(delimiter + 1));

                // Add section prefix if we're in a section
                if (!current_section.empty())
                {
                    key = current_section + "." + key;
                }

                values[key] = value;
            }
        }

        // Apply values to configuration
        try
        {
            // TD3 config
            if (values.count("td3.state_dim"))
                parseScalar(values["td3.state_dim"], td3.state_dim);
            if (values.count("td3.action_dim"))
                parseScalar(values["td3.action_dim"], td3.action_dim);
            if (values.count("td3.gamma"))
                parseScalar(values["td3.gamma"], td3.gamma);
            if (values.count("td3.tau"))
                parseScalar(values["td3.tau"], td3.tau);
            if (values.count("td3.policy_noise"))
                parseScalar(values["td3.policy_noise"], td3.policy_noise);
            if (values.count("td3.noise_clip"))
                parseScalar(values["td3.noise_clip"], td3.noise_clip);
            if (values.count("td3.policy_freq"))
                parseScalar(values["td3.policy_freq"], td3.policy_freq);
            if (values.count("td3.batch_size"))
                parseScalar(values["td3.batch_size"], td3.batch_size);
            if (values.count("td3.buffer_size"))
                parseScalar(values["td3.buffer_size"], td3.buffer_size);
            if (values.count("td3.device"))
                parseScalar(values["td3.device"], td3.device);
            if (values.count("td3.exploration_noise"))
                parseScalar(values["td3.exploration_noise"], td3.exploration_noise);
            if (values.count("td3.exploration_noise_std"))
                parseScalar(values["td3.exploration_noise_std"], td3.exploration_noise_std);

            // Actor network config
            if (values.count("td3.actor.hidden_layers"))
                parseVector(values["td3.actor.hidden_layers"], td3.actor.hidden_layers);
            if (values.count("td3.actor.activation"))
                parseScalar(values["td3.actor.activation"], td3.actor.activation);
            if (values.count("td3.actor.learning_rate"))
                parseScalar(values["td3.actor.learning_rate"], td3.actor.learning_rate);
            if (values.count("td3.actor.weight_decay"))
                parseScalar(values["td3.actor.weight_decay"], td3.actor.weight_decay);
            if (values.count("td3.actor.batch_norm"))
                parseScalar(values["td3.actor.batch_norm"], td3.actor.batch_norm);

            // Critic network config
            if (values.count("td3.critic.hidden_layers"))
                parseVector(values["td3.critic.hidden_layers"], td3.critic.hidden_layers);
            if (values.count("td3.critic.activation"))
                parseScalar(values["td3.critic.activation"], td3.critic.activation);
            if (values.count("td3.critic.learning_rate"))
                parseScalar(values["td3.critic.learning_rate"], td3.critic.learning_rate);
            if (values.count("td3.critic.weight_decay"))
                parseScalar(values["td3.critic.weight_decay"], td3.critic.weight_decay);
            if (values.count("td3.critic.batch_norm"))
                parseScalar(values["td3.critic.batch_norm"], td3.critic.batch_norm);

            // Environment config
            if (values.count("env.type"))
                parseScalar(values["env.type"], environment.type);
            if (values.count("env.max_episode_steps"))
                parseScalar(values["env.max_episode_steps"], environment.max_episode_steps);
            if (values.count("env.reward_function"))
                parseScalar(values["env.reward_function"], environment.reward_function);
            if (values.count("env.reward_scale"))
                parseScalar(values["env.reward_scale"], environment.reward_scale);

            // Hexapod environment config
            const auto *hexapod_config = dynamic_cast<const HexapodEnvironmentConfig *>(&environment);
            if (hexapod_config)
            {
                // Create a new hexapod config
                HexapodEnvironmentConfig new_hex_config = *hexapod_config;

                if (values.count("hexapod.velocity_weight"))
                    parseScalar(values["hexapod.velocity_weight"], new_hex_config.velocity_weight);
                if (values.count("hexapod.energy_weight"))
                    parseScalar(values["hexapod.energy_weight"], new_hex_config.energy_weight);
                if (values.count("hexapod.stability_weight"))
                    parseScalar(values["hexapod.stability_weight"], new_hex_config.stability_weight);
                if (values.count("hexapod.smoothness_weight"))
                    parseScalar(values["hexapod.smoothness_weight"], new_hex_config.smoothness_weight);
                if (values.count("hexapod.use_imu"))
                    parseScalar(values["hexapod.use_imu"], new_hex_config.use_imu);
                if (values.count("hexapod.use_leg_positions"))
                    parseScalar(values["hexapod.use_leg_positions"], new_hex_config.use_leg_positions);
                if (values.count("hexapod.step_time"))
                    parseScalar(values["hexapod.step_time"], new_hex_config.step_time);

                // Replace the environment with our updated hexapod config
                environment = new_hex_config;
            }
        }
        catch (const std::exception &e)
        {
            utils::Logger::error("Error parsing config: " + std::string(e.what()));
            return Result::ERROR_INVALID_ARGUMENT;
        }

        utils::Logger::info("Configuration loaded successfully from " + filename);
        return Result::SUCCESS;
    }

    Result Config::saveToFile(const std::string &filename) const
    {
        std::ofstream file(filename);
        if (!file)
        {
            utils::Logger::error("Failed to open config file for writing: " + filename);
            return Result::ERROR_INVALID_ARGUMENT;
        }

        // Helper function to write a section header
        auto writeSection = [&](const std::string &name)
        {
            file << "\n[" << name << "]\n";
        };

        // Write TD3 section
        writeSection("td3");
        file << "state_dim = " << td3.state_dim << "\n";
        file << "action_dim = " << td3.action_dim << "\n";
        file << "gamma = " << td3.gamma << "\n";
        file << "tau = " << td3.tau << "\n";
        file << "policy_noise = " << td3.policy_noise << "\n";
        file << "noise_clip = " << td3.noise_clip << "\n";
        file << "policy_freq = " << td3.policy_freq << "\n";
        file << "batch_size = " << td3.batch_size << "\n";
        file << "buffer_size = " << td3.buffer_size << "\n";
        file << "device = " << td3.device << "\n";
        file << "exploration_noise = " << (td3.exploration_noise ? "true" : "false") << "\n";
        file << "exploration_noise_std = " << td3.exploration_noise_std << "\n";

        // Write actor config
        writeSection("td3.actor");
        file << "hidden_layers = [";
        for (size_t i = 0; i < td3.actor.hidden_layers.size(); i++)
        {
            file << td3.actor.hidden_layers[i];
            if (i < td3.actor.hidden_layers.size() - 1)
                file << ", ";
        }
        file << "]\n";
        file << "activation = " << td3.actor.activation << "\n";
        file << "learning_rate = " << td3.actor.learning_rate << "\n";
        file << "weight_decay = " << td3.actor.weight_decay << "\n";
        file << "batch_norm = " << (td3.actor.batch_norm ? "true" : "false") << "\n";

        // Write critic config
        writeSection("td3.critic");
        file << "hidden_layers = [";
        for (size_t i = 0; i < td3.critic.hidden_layers.size(); i++)
        {
            file << td3.critic.hidden_layers[i];
            if (i < td3.critic.hidden_layers.size() - 1)
                file << ", ";
        }
        file << "]\n";
        file << "activation = " << td3.critic.activation << "\n";
        file << "learning_rate = " << td3.critic.learning_rate << "\n";
        file << "weight_decay = " << td3.critic.weight_decay << "\n";
        file << "batch_norm = " << (td3.critic.batch_norm ? "true" : "false") << "\n";

        // Write environment config
        writeSection("env");
        file << "type = " << environment.type << "\n";
        file << "max_episode_steps = " << environment.max_episode_steps << "\n";
        file << "reward_function = " << environment.reward_function << "\n";
        file << "reward_scale = " << environment.reward_scale << "\n";

        // Write hexapod specific config
        try
        {
            const auto &hexapod_config = dynamic_cast<const HexapodEnvironmentConfig &>(environment);
            writeSection("hexapod");
            file << "velocity_weight = " << hexapod_config.velocity_weight << "\n";
            file << "energy_weight = " << hexapod_config.energy_weight << "\n";
            file << "stability_weight = " << hexapod_config.stability_weight << "\n";
            file << "smoothness_weight = " << hexapod_config.smoothness_weight << "\n";
            file << "use_imu = " << (hexapod_config.use_imu ? "true" : "false") << "\n";
            file << "use_leg_positions = " << (hexapod_config.use_leg_positions ? "true" : "false") << "\n";
            file << "step_time = " << hexapod_config.step_time << "\n";
        }
        catch (...)
        {
            // Not a hexapod environment, skip
        }

        // Write training config
        writeSection("training");
        file << "max_epochs = " << training.max_epochs << "\n";
        file << "steps_per_epoch = " << training.steps_per_epoch << "\n";
        file << "evaluate_every = " << training.evaluate_every << "\n";
        file << "evaluation_episodes = " << training.evaluation_episodes << "\n";
        file << "save_path = " << training.save_path << "\n";
        file << "log_path = " << training.log_path << "\n";
        file << "render = " << (training.render ? "true" : "false") << "\n";

        // Write TIDL config
        writeSection("tidl");
        file << "model_path = " << tidl.model_path << "\n";
        file << "num_eve_cores = " << tidl.num_eve_cores << "\n";
        file << "num_dsp_cores = " << tidl.num_dsp_cores << "\n";
        file << "quantize = " << (tidl.quantize ? "true" : "false") << "\n";
        file << "quantization_bits = " << tidl.quantization_bits << "\n";
        file << "calibration_file = " << tidl.calibration_file << "\n";

        // Write OpenCL config
        writeSection("opencl");
        file << "platform_index = " << opencl.platform_index << "\n";
        file << "device_index = " << opencl.device_index << "\n";
        file << "use_local_mem = " << (opencl.use_local_mem ? "true" : "false") << "\n";
        file << "profiling = " << (opencl.profiling ? "true" : "false") << "\n";

        utils::Logger::info("Configuration saved successfully to " + filename);
        return Result::SUCCESS;
    }

    Config Config::createDefault()
    {
        Config config;

        // Create hexapod-specific environment config
        config.environment = HexapodEnvironmentConfig();

        // Set default values for TD3
        config.td3.state_dim = 24;  // 6 legs x (hip, knee, ankle) + IMU (3 accel, 3 gyro)
        config.td3.action_dim = 18; // 6 legs x 3 joints

        utils::Logger::info("Created default configuration");
        return config;
    }

} // namespace td3learn
