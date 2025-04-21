#include <random>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iostream>
#include "td3learn/models.hpp"
#include "td3learn/utils.hpp"

// Add conditionally included headers
#ifdef ENABLE_TIDL
#include "tidl/tidl_api.h"
#endif

#ifdef ENABLE_OPENCL
#include <CL/cl.hpp>
#endif

namespace td3learn
{

    // Simple fully connected neural network implementation
    class NeuralNetwork
    {
    public:
        struct Layer
        {
            std::vector<std::vector<Scalar>> weights;
            std::vector<Scalar> biases;
            std::string activation;
            bool use_batch_norm;

            // Apply layer to input
            std::vector<Scalar> forward(const std::vector<Scalar> &input) const
            {
                if (input.size() != weights[0].size())
                {
                    throw std::runtime_error("Input size mismatch");
                }

                // Compute layer output (y = activation(wx + b))
                std::vector<Scalar> output(weights.size(), 0.0f);
                for (size_t i = 0; i < weights.size(); i++)
                {
                    Scalar sum = biases[i];
                    for (size_t j = 0; j < weights[i].size(); j++)
                    {
                        sum += weights[i][j] * input[j];
                    }
                    output[i] = applyActivation(sum, activation);
                }

                return output;
            }

            // Apply activation function
            static Scalar applyActivation(Scalar x, const std::string &activation_type)
            {
                if (activation_type == "relu")
                {
                    return std::max(0.0f, x);
                }
                else if (activation_type == "tanh")
                {
                    return std::tanh(x);
                }
                else if (activation_type == "sigmoid")
                {
                    return 1.0f / (1.0f + std::exp(-x));
                }
                else
                {
                    // Linear activation (default)
                    return x;
                }
            }
        };

        // Create neural network with specified layer sizes
        void createNetwork(
            int input_dim,
            const std::vector<int> &hidden_sizes,
            int output_dim,
            const std::string &hidden_activation,
            const std::string &output_activation,
            bool batch_norm)
        {
            if (hidden_sizes.empty())
            {
                throw std::invalid_argument("At least one hidden layer is required");
            }

            layers.clear();

            // Input -> First Hidden
            Layer input_layer;
            input_layer.weights.resize(hidden_sizes[0]);
            for (auto &w : input_layer.weights)
            {
                w.resize(input_dim);
            }
            input_layer.biases.resize(hidden_sizes[0]);
            input_layer.activation = hidden_activation;
            input_layer.use_batch_norm = batch_norm;
            layers.push_back(input_layer);

            // Hidden -> Hidden
            for (size_t i = 1; i < hidden_sizes.size(); i++)
            {
                Layer hidden_layer;
                hidden_layer.weights.resize(hidden_sizes[i]);
                for (auto &w : hidden_layer.weights)
                {
                    w.resize(hidden_sizes[i - 1]);
                }
                hidden_layer.biases.resize(hidden_sizes[i]);
                hidden_layer.activation = hidden_activation;
                hidden_layer.use_batch_norm = batch_norm;
                layers.push_back(hidden_layer);
            }

            // Hidden -> Output
            Layer output_layer;
            output_layer.weights.resize(output_dim);
            for (auto &w : output_layer.weights)
            {
                w.resize(hidden_sizes.back());
            }
            output_layer.biases.resize(output_dim);
            output_layer.activation = output_activation;
            output_layer.use_batch_norm = false; // No batch norm in output
            layers.push_back(output_layer);

            // Initialize with Xavier/Glorot
            initializeWeights();
        }

        // Forward pass through the network
        std::vector<Scalar> forward(const std::vector<Scalar> &input) const
        {
            std::vector<Scalar> current = input;
            for (const auto &layer : layers)
            {
                current = layer.forward(current);
            }
            return current;
        }

        // Save network to file
        Result save(const std::string &filename) const
        {
            try
            {
                std::ofstream file(filename, std::ios::binary);
                if (!file.is_open())
                {
                    utils::Logger::error("Failed to open file for writing: " + filename);
                    return Result::ERROR_INVALID_ARGUMENT;
                }

                // Write number of layers
                size_t num_layers = layers.size();
                file.write(reinterpret_cast<const char *>(&num_layers), sizeof(num_layers));

                // Write each layer
                for (const auto &layer : layers)
                {
                    // Write dimensions
                    size_t output_size = layer.weights.size();
                    size_t input_size = layer.weights.empty() ? 0 : layer.weights[0].size();
                    file.write(reinterpret_cast<const char *>(&output_size), sizeof(output_size));
                    file.write(reinterpret_cast<const char *>(&input_size), sizeof(input_size));

                    // Write activation function
                    size_t name_size = layer.activation.size();
                    file.write(reinterpret_cast<const char *>(&name_size), sizeof(name_size));
                    file.write(layer.activation.data(), static_cast<std::streamsize>(name_size));

                    // Write batch norm flag
                    file.write(reinterpret_cast<const char *>(&layer.use_batch_norm), sizeof(layer.use_batch_norm));

                    // Write weights
                    for (const auto &row : layer.weights)
                    {
                        file.write(reinterpret_cast<const char *>(row.data()),
                                   static_cast<std::streamsize>(row.size() * sizeof(Scalar)));
                    }

                    // Write biases
                    file.write(reinterpret_cast<const char *>(layer.biases.data()),
                               static_cast<std::streamsize>(layer.biases.size() * sizeof(Scalar)));
                }

                return Result::SUCCESS;
            }
            catch (const std::exception &e)
            {
                utils::Logger::error("Error saving network: " + std::string(e.what()));
                return Result::ERROR_EXECUTION;
            }
        }

        // Load network from file
        Result load(const std::string &filename)
        {
            try
            {
                std::ifstream file(filename, std::ios::binary);
                if (!file.is_open())
                {
                    utils::Logger::error("Failed to open file for reading: " + filename);
                    return Result::ERROR_INVALID_ARGUMENT;
                }

                // Read number of layers
                size_t num_layers;
                file.read(reinterpret_cast<char *>(&num_layers), sizeof(num_layers));
                layers.resize(num_layers);

                // Read each layer
                for (auto &layer : layers)
                {
                    // Read dimensions
                    size_t output_size, input_size;
                    file.read(reinterpret_cast<char *>(&output_size), sizeof(output_size));
                    file.read(reinterpret_cast<char *>(&input_size), sizeof(input_size));

                    // Read activation function
                    size_t name_size;
                    file.read(reinterpret_cast<char *>(&name_size), sizeof(name_size));
                    layer.activation.resize(name_size);
                    file.read(&layer.activation[0], static_cast<std::streamsize>(name_size));

                    // Read batch norm flag
                    file.read(reinterpret_cast<char *>(&layer.use_batch_norm), sizeof(layer.use_batch_norm));

                    // Resize weights and biases
                    layer.weights.resize(output_size);
                    for (auto &row : layer.weights)
                    {
                        row.resize(input_size);
                        file.read(reinterpret_cast<char *>(row.data()),
                                  static_cast<std::streamsize>(row.size() * sizeof(Scalar)));
                    }

                    layer.biases.resize(output_size);
                    file.read(reinterpret_cast<char *>(layer.biases.data()),
                              static_cast<std::streamsize>(layer.biases.size() * sizeof(Scalar)));
                }

                return Result::SUCCESS;
            }
            catch (const std::exception &e)
            {
                utils::Logger::error("Error loading network: " + std::string(e.what()));
                return Result::ERROR_EXECUTION;
            }
        }

        // Initialize weights with Xavier/Glorot initialization
        void initializeWeights()
        {
            std::random_device rd;
            std::mt19937 gen(rd());

            for (size_t i = 0; i < layers.size(); i++)
            {
                auto &layer = layers[i];
                const size_t fan_in = layer.weights[0].size();
                const size_t fan_out = layer.weights.size();
                const Scalar scale = std::sqrt(6.0f / static_cast<float>(fan_in + fan_out));

                std::uniform_real_distribution<Scalar> dist(-scale, scale);

                // Initialize weights
                for (auto &row : layer.weights)
                {
                    for (auto &w : row)
                    {
                        w = dist(gen);
                    }
                }

                // Initialize biases to zero
                std::fill(layer.biases.begin(), layer.biases.end(), 0.0f);
            }
        }

        // Update weights from another network with interpolation factor
        void updateFrom(const NeuralNetwork &other, Scalar tau)
        {
            if (layers.size() != other.layers.size())
            {
                throw std::runtime_error("Network structures don't match");
            }

            for (size_t i = 0; i < layers.size(); i++)
            {
                auto &layer = layers[i];
                const auto &other_layer = other.layers[i];

                // Interpolate weights: target = (1-tau)*target + tau*source
                for (size_t j = 0; j < layer.weights.size(); j++)
                {
                    for (size_t k = 0; k < layer.weights[j].size(); k++)
                    {
                        layer.weights[j][k] = (1 - tau) * layer.weights[j][k] + tau * other_layer.weights[j][k];
                    }
                }

                // Interpolate biases
                for (size_t j = 0; j < layer.biases.size(); j++)
                {
                    layer.biases[j] = (1 - tau) * layer.biases[j] + tau * other_layer.biases[j];
                }
            }
        }

        // Get a copy of the network
        NeuralNetwork clone() const
        {
            NeuralNetwork copy;
            copy.layers = this->layers;
            return copy;
        }

    private:
        std::vector<Layer> layers;
    };

    // Base class for models
    class ModelBase : public Model
    {
    public:
        ModelBase() = default;
        virtual ~ModelBase() = default;

        // Base implementations
        std::vector<Scalar> forward(const std::vector<Scalar> &input) override
        {
            return network.forward(input);
        }

        Result save(const std::string &path) override
        {
            return network.save(path);
        }

        Result load(const std::string &path) override
        {
            return network.load(path);
        }

        void updateFrom(const Model &other, Scalar tau) override
        {
            const ModelBase *other_base = dynamic_cast<const ModelBase *>(&other);
            if (!other_base)
            {
                throw std::runtime_error("Invalid model type in updateFrom");
            }
            network.updateFrom(other_base->network, tau);
        }

    protected:
        NeuralNetwork network;
    };

    // Actor model implementation
    class ActorImpl : public Actor
    {
    public:
        ActorImpl(int state_dim, int action_dim, const NetworkConfig &config)
            : state_dim(state_dim), action_dim(action_dim)
        {
            // Create actor network with tanh output activation [-1, 1]
            network.createNetwork(state_dim, config.hidden_layers, action_dim,
                                  config.activation, "tanh", config.batch_norm);
        }

        std::unique_ptr<Model> clone() const override
        {
            auto cloned = std::make_unique<ActorImpl>(state_dim, action_dim,
                                                      NetworkConfig{}); // Dummy config
            cloned->network = this->network.clone();
            return std::unique_ptr<Model>(cloned.release());
        }

        std::vector<Scalar> forward(const std::vector<Scalar> &input) override
        {
            return network.forward(input);
        }

        Result save(const std::string &path) override
        {
            return network.save(path);
        }

        Result load(const std::string &path) override
        {
            return network.load(path);
        }

        void updateFrom(const Model &other, Scalar tau) override
        {
            const ActorImpl *other_actor = dynamic_cast<const ActorImpl *>(&other);
            if (!other_actor)
            {
                throw std::runtime_error("Invalid model type in updateFrom");
            }
            network.updateFrom(other_actor->network, tau);
        }

        Action getAction(const State &state) override
        {
            if (state.size() != state_dim)
            {
                throw std::invalid_argument("Invalid state dimension");
            }

            // Convert state to vector if needed
            std::vector<Scalar> input(state.begin(), state.end());

            // Forward pass through the network
            std::vector<Scalar> output = forward(input);

            // Convert to Action type
            Action action;
            action.resize(output.size());
            std::copy(output.begin(), output.end(), action.begin());

            return action;
        }

        Action getNoisyAction(const State &state, Scalar noise_std) override
        {
            // Get deterministic action
            Action action = getAction(state);

            // Add Gaussian noise
            std::random_device rd;
            std::mt19937 gen(rd());
            std::normal_distribution<Scalar> dist(0.0f, noise_std);

            // Add noise and clip to [-1, 1]
            for (auto &a : action)
            {
                a += dist(gen);
                a = std::max(-1.0f, std::min(a, 1.0f));
            }

            return action;
        }

    private:
        int state_dim;
        int action_dim;
        NeuralNetwork network;
    };

    // Critic model implementation
    class CriticImpl : public Critic
    {
    public:
        CriticImpl(int state_dim, int action_dim, const NetworkConfig &config)
            : state_dim(state_dim), action_dim(action_dim)
        {
            // Create critic network with linear output (Q-value)
            network.createNetwork(state_dim + action_dim, config.hidden_layers, 1,
                                  config.activation, "linear", config.batch_norm);
        }

        std::unique_ptr<Model> clone() const override
        {
            auto cloned = std::make_unique<CriticImpl>(state_dim, action_dim,
                                                       NetworkConfig{}); // Dummy config
            cloned->network = this->network.clone();
            return std::unique_ptr<Model>(cloned.release());
        }

        std::vector<Scalar> forward(const std::vector<Scalar> &input) override
        {
            return network.forward(input);
        }

        Result save(const std::string &path) override
        {
            return network.save(path);
        }

        Result load(const std::string &path) override
        {
            return network.load(path);
        }

        void updateFrom(const Model &other, Scalar tau) override
        {
            const CriticImpl *other_critic = dynamic_cast<const CriticImpl *>(&other);
            if (!other_critic)
            {
                throw std::runtime_error("Invalid model type in updateFrom");
            }
            network.updateFrom(other_critic->network, tau);
        }

        Scalar getValue(const State &state, const Action &action) override
        {
            if (state.size() != state_dim || action.size() != action_dim)
            {
                throw std::invalid_argument("Invalid state or action dimension");
            }

            // Concatenate state and action
            std::vector<Scalar> input(state_dim + action_dim);
            std::copy(state.begin(), state.end(), input.begin());
            std::copy(action.begin(), action.end(), input.begin() + state_dim);

            // Forward pass through the network
            std::vector<Scalar> output = forward(input);

            // Return Q-value (should be a single scalar)
            return output[0];
        }

    private:
        int state_dim;
        int action_dim;
        NeuralNetwork network;
    };

    // Static device selection for ModelFactory
    static std::string current_device = "cpu";

    // Actor factory implementation
    std::unique_ptr<Actor> Actor::create(int state_dim, int action_dim, const NetworkConfig &config)
    {
        return std::make_unique<ActorImpl>(state_dim, action_dim, config);
    }

    // Critic factory implementation
    std::unique_ptr<Critic> Critic::create(int state_dim, int action_dim, const NetworkConfig &config)
    {
        return std::make_unique<CriticImpl>(state_dim, action_dim, config);
    }

    // ModelFactory implementation
    bool ModelFactory::setDevice(const std::string &device)
    {
        // Save the requested device
        current_device = device;

        // Handle different device types
        if (device == "opencl")
        {
#ifdef ENABLE_OPENCL
            // Initialize OpenCL
            try
            {
                // OpenCL setup would normally go here
                utils::Logger::info("OpenCL device selected, but implementation not complete");
                return true;
            }
            catch (const std::exception &e)
            {
                // Use std::exception instead of cl::Error for broader exception handling
                utils::Logger::error("OpenCL initialization error: " + std::string(e.what()));
                current_device = "cpu"; // Fallback to CPU
                return false;
            }
#else
            utils::Logger::warning("OpenCL support not enabled, falling back to CPU");
            current_device = "cpu";
            return false;
#endif
        }
        else if (device == "cpu")
        {
            current_device = "cpu";
            utils::Logger::info("Using CPU for model inference");
            return true;
        }
        else if (device == "tidl")
        {
#ifdef ENABLE_TIDL
            try
            {
                // Initialize TIDL device
                int available_cores = TidlGetPreferredBatchSize(1);
                if (available_cores > 0)
                {
                    utils::Logger::info("Using TIDL acceleration with " +
                                        std::to_string(available_cores) + " available cores");
                    current_device = "tidl";
                    return true;
                }
                utils::Logger::warning("No TIDL cores available, falling back to CPU");
            }
            catch (const std::exception &e)
            {
                utils::Logger::warning("TIDL initialization error: " + std::string(e.what()));
            }
            current_device = "cpu";
            return false;
#else
            utils::Logger::warning("TIDL support not compiled in, using CPU");
            current_device = "cpu";
            return false;
#endif
        }

        utils::Logger::warning("Unknown device type: " + device + ", using CPU");
        current_device = "cpu";
        return false;
    }

    std::string ModelFactory::getDevice()
    {
        return current_device;
    }

    std::unique_ptr<Actor> ModelFactory::createActor(
        int state_dim, int action_dim, const NetworkConfig &config)
    {
        return Actor::create(state_dim, action_dim, config);
    }

    std::unique_ptr<Critic> ModelFactory::createCritic(
        int state_dim, int action_dim, const NetworkConfig &config)
    {
        return Critic::create(state_dim, action_dim, config);
    }

} // namespace td3learn
