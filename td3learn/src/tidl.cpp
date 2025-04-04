#include "td3learn/tidl.hpp"
#include "td3learn/utils.hpp"
#include <fstream>
#include <vector>
#include <string>
#include <stdexcept>
#include <cstring>

#ifdef ENABLE_TIDL
// Include TIDL API headers when enabled
#include "configuration.h"
#include "executor.h"
#include "execution_object.h"
#endif

namespace td3learn
{

#ifdef ENABLE_TIDL
    // Implementation classes with TIDL API support
    class TIDLModelImpl
    {
    public:
        TIDLModelImpl(const TIDLConfig &config) : config(config)
        {
            tidl_config = std::make_unique<tidl::Configuration>();
        }

        ~TIDLModelImpl() = default;

        Result init()
        {
            try
            {
                // Set up TIDL configuration parameters
                tidl_config->preProcType = 0; // Default preprocessing
                tidl_config->inWidth = input_shape[2];
                tidl_config->inHeight = input_shape[1];
                tidl_config->inNumChannels = input_shape[0];

                // Set network files if specified
                if (!config.model_path.empty())
                {
                    std::string base_path = config.model_path;
                    tidl_config->netBinFile = base_path + "_net.bin";
                    tidl_config->paramsBinFile = base_path + "_params.bin";

                    // Verify files exist
                    std::ifstream net_file(tidl_config->netBinFile);
                    std::ifstream param_file(tidl_config->paramsBinFile);

                    if (!net_file.good() || !param_file.good())
                    {
                        utils::Logger::error("TIDL model files not found at " + base_path);
                        return Result::ERROR_INVALID_ARGUMENT;
                    }
                }

                // Setup quantization parameters
                tidl_config->quantHistoryParam1 = 50; // Default value
                tidl_config->quantHistoryParam2 = 25; // Default value
                tidl_config->quantMargin = 10;        // Default value

                // Configure heap sizes
                tidl_config->PARAM_HEAP_SIZE = 9 * 1024 * 1024;    // Default 9MB
                tidl_config->NETWORK_HEAP_SIZE = 64 * 1024 * 1024; // Default 64MB

                // Validate configuration
                if (!tidl_config->Validate())
                {
                    utils::Logger::error("TIDL configuration validation failed");
                    return Result::ERROR_INVALID_ARGUMENT;
                }

                utils::Logger::info("TIDL model initialized successfully");
                return Result::SUCCESS;
            }
            catch (const std::exception &e)
            {
                utils::Logger::error("TIDL initialization error: " + std::string(e.what()));
                return Result::ERROR_INITIALIZATION;
            }
        }

        Result load(const std::string &path)
        {
            std::string base_path = path;

            // Update configuration with new files
            tidl_config->netBinFile = base_path + "_net.bin";
            tidl_config->paramsBinFile = base_path + "_params.bin";

            // Verify files exist
            std::ifstream net_file(tidl_config->netBinFile);
            std::ifstream param_file(tidl_config->paramsBinFile);

            if (!net_file.good() || !param_file.good())
            {
                utils::Logger::error("TIDL model files not found at " + base_path);
                return Result::ERROR_INVALID_ARGUMENT;
            }

            utils::Logger::info("TIDL model loaded from " + base_path);
            return Result::SUCCESS;
        }

        Result save(const std::string &path) const
        {
            // TIDL models are created by the import tool, we don't save them directly
            utils::Logger::warning("Direct TIDL model saving not supported - use exportTIDLModel instead");
            return Result::ERROR_NOT_IMPLEMENTED;
        }

        std::vector<int> getInputShape() const
        {
            return input_shape;
        }

        std::vector<int> getOutputShape() const
        {
            return output_shape;
        }

        // TIDL configuration
        TIDLConfig config;
        std::unique_ptr<tidl::Configuration> tidl_config;

        // Model shape information
        std::vector<int> input_shape = {1, 224, 224}; // Default shape, channels, height, width
        std::vector<int> output_shape = {10};         // Default output size
    };

    class TIDLExecutorImpl
    {
    public:
        TIDLExecutorImpl(const TIDLModel &model)
        {
            tidl_config = model.pImpl->tidl_config.get();
        }

        ~TIDLExecutorImpl() = default;

        Result init()
        {
            try
            {
                // Create TIDL executors for EVE and DSP
                int num_eve = 0;
                int num_dsp = 0;

                // Query available devices
                tidl::DeviceIds ids = tidl::Executor::GetDeviceIds();

                for (auto &id : ids)
                {
                    if (id.device_type == tidl::DeviceType::EVE)
                    {
                        num_eve++;
                    }
                    else if (id.device_type == tidl::DeviceType::DSP)
                    {
                        num_dsp++;
                    }
                }

                utils::Logger::info("Found " + std::to_string(num_eve) + " EVE and " +
                                    std::to_string(num_dsp) + " DSP cores");

                if (num_eve > 0)
                {
                    // Create EVE executor
                    eve_executor = std::make_unique<tidl::Executor>(tidl::DeviceType::EVE,
                                                                    *tidl_config,
                                                                    num_eve);
                }

                if (num_dsp > 0)
                {
                    // Create DSP executor
                    dsp_executor = std::make_unique<tidl::Executor>(tidl::DeviceType::DSP,
                                                                    *tidl_config,
                                                                    num_dsp);
                }

                // Need at least one executor
                if (!eve_executor && !dsp_executor)
                {
                    utils::Logger::error("No EVE or DSP cores available");
                    return Result::ERROR_HARDWARE;
                }

                // Create execution objects from available executors
                if (eve_executor)
                {
                    for (uint32_t i = 0; i < eve_executor->GetNumExecutionObjects(); i++)
                    {
                        execution_objects.push_back(&eve_executor->operator[](i));
                    }
                }

                if (dsp_executor)
                {
                    for (uint32_t i = 0; i < dsp_executor->GetNumExecutionObjects(); i++)
                    {
                        execution_objects.push_back(&dsp_executor->operator[](i));
                    }
                }

                utils::Logger::info("Created " + std::to_string(execution_objects.size()) +
                                    " execution objects");

                if (execution_objects.empty())
                {
                    utils::Logger::error("Failed to create any execution objects");
                    return Result::ERROR_INITIALIZATION;
                }

                // Allocate input/output memory for each execution object
                for (auto eo : execution_objects)
                {
                    // Allocate input buffer
                    void *input_buffer = nullptr;
                    if (!eo->AllocateMemory(tidl::BufferType::INPUT, &input_buffer))
                    {
                        utils::Logger::error("Failed to allocate input memory");
                        return Result::ERROR_MEMORY;
                    }
                    input_buffers.push_back(input_buffer);

                    // Allocate output buffer
                    void *output_buffer = nullptr;
                    if (!eo->AllocateMemory(tidl::BufferType::OUTPUT, &output_buffer))
                    {
                        utils::Logger::error("Failed to allocate output memory");
                        return Result::ERROR_MEMORY;
                    }
                    output_buffers.push_back(output_buffer);
                }

                current_eo_index = 0;
                return Result::SUCCESS;
            }
            catch (const std::exception &e)
            {
                utils::Logger::error("TIDL executor initialization error: " + std::string(e.what()));
                return Result::ERROR_INITIALIZATION;
            }
        }

        Result execute(const std::vector<Scalar> &input, std::vector<Scalar> &output)
        {
            if (execution_objects.empty() || current_eo_index >= execution_objects.size())
            {
                return Result::ERROR_INITIALIZATION;
            }

            try
            {
                // Get current execution object and buffers
                auto eo = execution_objects[current_eo_index];
                void *input_buffer = input_buffers[current_eo_index];
                void *output_buffer = output_buffers[current_eo_index];

                // Copy input data to input buffer
                std::memcpy(input_buffer, input.data(), input.size() * sizeof(Scalar));

                // Wait for previous frame to complete if still processing
                bool was_processed = eo->ProcessFrameWait();

                // Process the frame
                bool success = eo->ProcessFrameStartAsync(input_buffer, output_buffer);
                if (!success)
                {
                    utils::Logger::error("Failed to start frame processing");
                    return Result::ERROR_EXECUTION;
                }

                // Wait for this frame to complete
                was_processed = eo->ProcessFrameWait();
                if (!was_processed)
                {
                    utils::Logger::error("Frame was not processed");
                    return Result::ERROR_EXECUTION;
                }

                // Resize output buffer if needed
                size_t output_size = eo->GetOutputBufferSize() / sizeof(Scalar);
                output.resize(output_size);

                // Copy output data to result
                std::memcpy(output.data(), output_buffer, output_size * sizeof(Scalar));

                // Move to next execution object in round-robin fashion
                current_eo_index = (current_eo_index + 1) % execution_objects.size();

                return Result::SUCCESS;
            }
            catch (const std::exception &e)
            {
                utils::Logger::error("TIDL execution error: " + std::string(e.what()));
                return Result::ERROR_EXECUTION;
            }
        }

        static bool isAvailable()
        {
            try
            {
                tidl::DeviceIds ids = tidl::Executor::GetDeviceIds();
                return !ids.empty();
            }
            catch (...)
            {
                return false;
            }
        }

        static std::string getDeviceInfo()
        {
            std::string info = "TIDL Devices:\n";
            try
            {
                tidl::DeviceIds ids = tidl::Executor::GetDeviceIds();
                int eve_count = 0, dsp_count = 0;

                for (auto &id : ids)
                {
                    if (id.device_type == tidl::DeviceType::EVE)
                    {
                        eve_count++;
                    }
                    else if (id.device_type == tidl::DeviceType::DSP)
                    {
                        dsp_count++;
                    }
                }

                info += "  EVE cores: " + std::to_string(eve_count) + "\n";
                info += "  DSP cores: " + std::to_string(dsp_count);

                return info;
            }
            catch (const std::exception &e)
            {
                return "Error querying TIDL devices: " + std::string(e.what());
            }
        }

    private:
        tidl::Configuration *tidl_config;
        std::unique_ptr<tidl::Executor> eve_executor;
        std::unique_ptr<tidl::Executor> dsp_executor;
        std::vector<tidl::ExecutionObject *> execution_objects;
        std::vector<void *> input_buffers;
        std::vector<void *> output_buffers;
        size_t current_eo_index = 0;
    };

#else
    // Stub implementation classes when TIDL is not available
    class TIDLModelImpl
    {
    public:
        TIDLModelImpl(const TIDLConfig &config) : config(config) {}

        Result init()
        {
            utils::Logger::warning("TIDL support not enabled at compile time");
            return Result::ERROR_NOT_IMPLEMENTED;
        }

        Result load(const std::string &path)
        {
            return Result::ERROR_NOT_IMPLEMENTED;
        }

        Result save(const std::string &path) const
        {
            return Result::ERROR_NOT_IMPLEMENTED;
        }

        std::vector<int> getInputShape() const
        {
            return {1, 224, 224};
        }

        std::vector<int> getOutputShape() const
        {
            return {10};
        }

        TIDLConfig config;
    };

    class TIDLExecutorImpl
    {
    public:
        TIDLExecutorImpl(const TIDLModel &) {}

        Result init()
        {
            utils::Logger::warning("TIDL support not enabled at compile time");
            return Result::ERROR_NOT_IMPLEMENTED;
        }

        Result execute(const std::vector<Scalar> &, std::vector<Scalar> &)
        {
            return Result::ERROR_NOT_IMPLEMENTED;
        }

        static bool isAvailable()
        {
            return false;
        }

        static std::string getDeviceInfo()
        {
            return "TIDL support not enabled at compile time";
        }
    };
#endif

    // TIDLModel implementation
    TIDLModel::TIDLModel(const TIDLConfig &config)
        : pImpl(std::make_unique<TIDLModelImpl>(config)) {}

    TIDLModel::~TIDLModel() = default;

    Result TIDLModel::init()
    {
        return pImpl->init();
    }

    Result TIDLModel::load(const std::string &path)
    {
        return pImpl->load(path);
    }

    Result TIDLModel::save(const std::string &path) const
    {
        return pImpl->save(path);
    }

    std::vector<int> TIDLModel::getInputShape() const
    {
        return pImpl->getInputShape();
    }

    std::vector<int> TIDLModel::getOutputShape() const
    {
        return pImpl->getOutputShape();
    }

    // TIDLExecutor implementation
    TIDLExecutor::TIDLExecutor(const TIDLModel &model)
        : pImpl(std::make_unique<TIDLExecutorImpl>(model)) {}

    TIDLExecutor::~TIDLExecutor() = default;

    Result TIDLExecutor::init()
    {
        return pImpl->init();
    }

    Result TIDLExecutor::execute(const std::vector<Scalar> &input, std::vector<Scalar> &output)
    {
        return pImpl->execute(input, output);
    }

    bool TIDLExecutor::isAvailable()
    {
        return TIDLExecutorImpl::isAvailable();
    }

    std::string TIDLExecutor::getDeviceInfo()
    {
        return TIDLExecutorImpl::getDeviceInfo();
    }

    // TIDLConverter implementation
    Result TIDLConverter::convertModel(
        const std::string &source_path,
        const std::string &target_path,
        const TIDLConfig &config)
    {

        utils::Logger::info("Converting model from " + source_path + " to " + target_path);

#ifdef ENABLE_TIDL
        // Model conversion would require using TIDL import tools
        // This is typically done offline with separate tools
        utils::Logger::warning("Direct model conversion not supported - use TI TIDL import tools");
#endif

        return Result::ERROR_NOT_IMPLEMENTED;
    }

} // namespace td3learn
