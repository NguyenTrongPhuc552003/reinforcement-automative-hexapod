#ifndef TD3LEARN_CONFIG_HPP
#define TD3LEARN_CONFIG_HPP

#include <string>
#include <memory>
#include "td3learn/types.hpp"

namespace td3learn
{

    /**
     * @brief Neural network configuration
     */
    struct NetworkConfig
    {
        std::vector<int> hidden_layers{256, 256}; // Hidden layer sizes
        std::string activation = "relu";          // Activation function
        Scalar learning_rate = 0.001f;            // Learning rate for optimizer
        Scalar weight_decay = 0.0f;               // L2 regularization
        bool batch_norm = false;                  // Use batch normalization
    };

    /**
     * @brief TD3 algorithm configuration
     */
    struct TD3Config
    {
        int state_dim = 0;                   // State dimension
        int action_dim = 0;                  // Action dimension
        Scalar gamma = 0.99f;                // Discount factor
        Scalar tau = 0.005f;                 // Target network update rate
        Scalar policy_noise = 0.2f;          // Noise added to target policy
        Scalar noise_clip = 0.5f;            // Noise clip value
        int policy_freq = 2;                 // Policy update frequency
        NetworkConfig actor;                 // Actor network config
        NetworkConfig critic;                // Critic network config
        size_t batch_size = 256;             // Batch size for training
        size_t buffer_size = 1000000;        // Replay buffer size
        std::string device = "cpu";          // Device to run on (cpu, tidl, opencl)
        bool exploration_noise = true;       // Add exploration noise during training
        Scalar exploration_noise_std = 0.1f; // Standard deviation of exploration noise
        std::string noise_type = "gaussian"; // Noise type: "gaussian" or "ou"

        // Hexapod-specific TD3 parameters
        bool adaptive_noise = false;     // Reduce noise over time
        bool prioritized_replay = false; // Use prioritized experience replay
    };

    /**
     * @brief Environment configuration
     * Must have virtual destructor for polymorphism to work
     */
    struct EnvironmentConfig
    {
        std::string type = "simulation";         // Environment type (simulation, real)
        Scalar max_episode_steps = 1000;         // Maximum steps per episode
        std::string reward_function = "default"; // Reward function to use
        Scalar reward_scale = 1.0f;              // Scaling factor for rewards

        // Virtual destructor for polymorphism
        virtual ~EnvironmentConfig() = default;
    };

    /**
     * @brief Hexapod-specific environment configuration
     */
    struct HexapodEnvironmentConfig : public EnvironmentConfig
    {
        Scalar velocity_weight = 1.0f;   // Weight for velocity reward component
        Scalar energy_weight = 0.1f;     // Weight for energy efficiency component
        Scalar stability_weight = 1.0f;  // Weight for stability component
        Scalar smoothness_weight = 0.5f; // Weight for motion smoothness component
        bool use_imu = true;             // Use IMU data in state
        bool use_leg_positions = true;   // Use leg positions in state
        Scalar step_time = 0.05f;        // Time step duration in seconds
        int action_dim = 18;             // Action dimension (6 legs x 3 joints)

        // Virtual destructor
        virtual ~HexapodEnvironmentConfig() = default;
    };

    /**
     * @brief Training configuration
     */
    struct TrainingConfig
    {
        int max_epochs = 1000;            // Maximum training epochs
        int steps_per_epoch = 5000;       // Steps per epoch
        int evaluate_every = 5;           // Evaluate every N epochs
        int evaluation_episodes = 10;     // Number of episodes for evaluation
        std::string save_path = "models"; // Path to save models
        std::string log_path = "logs";    // Path for tensorboard logs
        bool render = false;              // Render environment during training
    };

    /**
     * @brief TIDL acceleration configuration
     */
    struct TIDLConfig
    {
        std::string model_path;             // Path to TIDL model
        int num_eve_cores = 4;              // Number of EVE cores to use
        int num_dsp_cores = 2;              // Number of DSP cores to use
        bool quantize = true;               // Quantize model for inference
        int quantization_bits = 8;          // Quantization bit depth
        std::string calibration_file;       // Calibration data for quantization
        int layer_group_size = 3;           // Group size for layer fusion
        bool optimize_memory = true;        // Optimize for memory usage
        bool enable_pre_processing = true;  // Enable pre-processing on DSP
        bool enable_post_processing = true; // Enable post-processing on DSP

        // Additional parameters for hardware-specific configuration
        int input_height = 1;    // Input height dimension
        int input_width = 1;     // Input width dimension
        int input_channels = 24; // Input channels (state dimension)
        int preprocess_type = 0; // Preprocessing type (0 for RL models)
        int batch_size = 1;      // Batch size for inference
    };

    /**
     * @brief OpenCL acceleration configuration
     */
    struct OpenCLConfig
    {
        int platform_index = 0;                  // OpenCL platform index
        int device_index = 0;                    // OpenCL device index
        bool use_local_mem = true;               // Use local memory optimization
        bool profiling = false;                  // Enable profiling
        bool enable_cache = true;                // Enable OpenCL program cache
        std::string cache_dir = ".opencl_cache"; // OpenCL program cache directory
        bool optimize_matrix_mult = true;        // Use optimized matrix multiplication
        int work_group_size = 64;                // Default work group size
        bool fp16_enabled = false;               // Enable FP16 computation if supported
    };

    /**
     * @brief Complete configuration
     */
    struct Config
    {
        TD3Config td3;                 // TD3 algorithm configuration
        EnvironmentConfig environment; // Environment configuration
        TrainingConfig training;       // Training configuration
        TIDLConfig tidl;               // TIDL configuration
        OpenCLConfig opencl;           // OpenCL configuration

        /**
         * @brief Load configuration from file
         * @param filename Path to configuration file
         * @return Result code
         */
        Result loadFromFile(const std::string &filename);

        /**
         * @brief Save configuration to file
         * @param filename Path to configuration file
         * @return Result code
         */
        Result saveToFile(const std::string &filename) const;

        /**
         * @brief Create default configuration
         * @return Default configuration
         */
        static Config createDefault();
    };

} // namespace td3learn

#endif // TD3LEARN_CONFIG_HPP
