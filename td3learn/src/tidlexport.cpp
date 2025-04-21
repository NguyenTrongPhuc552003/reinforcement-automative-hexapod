#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <filesystem>
#include "td3learn/tidlexport.hpp"

#ifdef ENABLE_TIDL
#include "configuration.h"
#include "executor.h"
#include "execution_object.h"
#include "execution_object_pipeline.h"
#include "runtime_initialization.h"
#include "param_structure.h"

// Add local stub header definitions if the real header isn't found
// Define stub structures if custom.h is missing
#ifndef TIDL_CUSTOM_H_INCLUDED
#define TIDL_CUSTOM_H_INCLUDED
namespace tidl {
    // Minimal stub definitions for custom.h
    struct Configuration {
        int numLayers;
        int inputHeight;
        int inputWidth;
        int inputChannels;
        int outputSize;
    };
}
#endif

#endif

namespace td3learn
{

    TIDLExporter::TIDLExporter(const TIDLConfig &config)
        : config_(config), initialized_(false) {}

    TIDLExporter::~TIDLExporter()
    {
        cleanup();
    }

    Result TIDLExporter::init()
    {
#ifdef ENABLE_TIDL
        try
        {
            utils::Logger::info("Initializing TIDL exporter...");

            // Initialize TIDL runtime with specified cores
            ERT_STATUS status = initializeRuntime(config_.num_eve_cores, config_.num_dsp_cores);
            if (status != ERT_SUCCESS)
            {
                utils::Logger::error("Failed to initialize TIDL runtime: " + std::to_string(status));
                return Result::ERROR_INITIALIZATION;
            }

            initialized_ = true;
            utils::Logger::info("TIDL exporter initialized with " +
                                std::to_string(config_.num_eve_cores) + " EVE cores and " +
                                std::to_string(config_.num_dsp_cores) + " DSP cores");
            return Result::SUCCESS;
        }
        catch (const std::exception &e)
        {
            utils::Logger::error("Exception during TIDL initialization: " + std::string(e.what()));
            return Result::ERROR_INITIALIZATION;
        }
#else
        utils::Logger::error("TIDL support not enabled at compile time");
        return Result::ERROR_NOT_IMPLEMENTED;
#endif
    }

    void TIDLExporter::cleanup()
    {
#ifdef ENABLE_TIDL
        if (initialized_)
        {
            utils::Logger::info("Cleaning up TIDL resources...");
            finalizeRuntime();
            initialized_ = false;
        }
#endif
    }

    Result TIDLExporter::exportModel(const std::string &path, const std::unique_ptr<Actor> &actor)
    {
#ifdef ENABLE_TIDL
        try
        {
            if (!initialized_ && init() != Result::SUCCESS)
            {
                return Result::ERROR_INITIALIZATION;
            }

            utils::Logger::info("Exporting model to TIDL format: " + path);

            // Create output directory if it doesn't exist
            if (!std::filesystem::exists(path))
            {
                if (!std::filesystem::create_directories(path))
                {
                    utils::Logger::error("Failed to create output directory: " + path);
                    return Result::ERROR_IO;
                }
            }

            // 1. Create TIDL configuration
            Configuration tidl_config;
            setupTIDLConfiguration(tidl_config);

            // 2. Export network structure
            std::string network_path = path + "/network.bin";
            std::string params_path = path + "/params.bin";
            std::string config_path = path + "/config.cfg";

            if (!exportNetworkStructure(network_path, params_path, *actor))
            {
                utils::Logger::error("Failed to export network structure");
                return Result::ERROR_EXECUTION;
            }

            // 3. Save configuration file
            tidl_config.netBinFile = network_path;
            tidl_config.paramsBinFile = params_path;

            std::ofstream config_file(config_path);
            if (!config_file.is_open())
            {
                utils::Logger::error("Failed to open configuration file for writing");
                return Result::ERROR_IO;
            }

            tidl_config.Print(config_file);
            config_file.close();

            // 4. Create deployment script
            createDeploymentScript(path);

            utils::Logger::info("TIDL model export completed successfully");
            return Result::SUCCESS;
        }
        catch (const std::exception &e)
        {
            utils::Logger::error("Exception during TIDL export: " + std::string(e.what()));
            return Result::ERROR_EXECUTION;
        }
#else
        utils::Logger::error("TIDL support not enabled at compile time");
        return Result::ERROR_NOT_IMPLEMENTED;
#endif
    }

#ifdef ENABLE_TIDL
    void TIDLExporter::setupTIDLConfiguration(Configuration &tidl_config)
    {
        // Basic configuration
        tidl_config.numFrames = config_.batch_size;
        tidl_config.inHeight = config_.input_height;
        tidl_config.inWidth = config_.input_width;
        tidl_config.inNumChannels = config_.input_channels;

        // Layer groups
        tidl_config.layerGroupNum = 1; // Single layer group for RL models

        // Memory configuration - optimized for BeagleBone AI
        tidl_config.NETWORK_HEAP_SIZE = 50 * 1024 * 1024; // 50MB network heap
        tidl_config.PARAM_HEAP_SIZE = 50 * 1024 * 1024;   // 50MB parameter heap
        tidl_config.DATA_HEAP_SIZE = 20 * 1024 * 1024;    // 20MB data heap

        // Processing configuration
        tidl_config.preProcType = config_.preprocess_type;
        tidl_config.inElementType = 0; // Float input
        tidl_config.inData1Signed = 1; // Signed input

        // Output configuration
        tidl_config.outElementType = 0; // Float output
        tidl_config.outData1Signed = 1; // Signed output

        // Core allocation
        tidl_config.numEveDevices = config_.num_eve_cores;
        tidl_config.numDspDevices = config_.num_dsp_cores;

        // Quantization settings for float16/int8 acceleration
        if (config_.quantize)
        {
            tidl_config.quantizationStyle = 1; // Dynamic fixed point quantization
            tidl_config.quantizationBits = config_.quantization_bits;

            if (!config_.calibration_file.empty() &&
                std::filesystem::exists(config_.calibration_file))
            {
                tidl_config.calibrationBinFile = config_.calibration_file;
            }
        }
    }

    bool TIDLExporter::exportNetworkStructure(
        const std::string &network_path,
        const std::string &params_path,
        const Actor &actor)
    {

        try
        {
            // Export network structure
            std::ofstream net_file(network_path, std::ios::binary);
            if (!net_file.is_open())
            {
                utils::Logger::error("Failed to open network file for writing");
                return false;
            }

            // Write network header information
            NetHeader header;
            header.version = 0x0105; // v1.5
            header.layer_count = computeLayerCount(actor);
            header.input_count = 1;  // Single input for RL
            header.output_count = 1; // Single output for RL

            net_file.write(reinterpret_cast<const char *>(&header), sizeof(header));

            // Export layers
            if (!exportLayers(net_file, actor))
            {
                utils::Logger::error("Failed to export layers");
                return false;
            }

            net_file.close();

            // Export parameters
            std::ofstream param_file(params_path, std::ios::binary);
            if (!param_file.is_open())
            {
                utils::Logger::error("Failed to open parameters file for writing");
                return false;
            }

            // Export weights and biases
            if (!exportWeights(param_file, actor))
            {
                utils::Logger::error("Failed to export weights");
                return false;
            }

            param_file.close();

            return true;
        }
        catch (const std::exception &e)
        {
            utils::Logger::error("Exception during network export: " + std::string(e.what()));
            return false;
        }
    }

    int TIDLExporter::computeLayerCount(const Actor &actor)
    {
        // For now, just return a reasonable default
        // In a real implementation, we'd inspect the model structure
        return 3; // Input, hidden, output
    }

    bool TIDLExporter::exportLayers(std::ofstream &file, const Actor &actor)
    {
        // TODO: Implement layer export for actor model
        // This is a stub implementation

        // In practice, we'd extract the actual layers from the actor model
        // and convert them to the TIDL format

        return true;
    }

    bool TIDLExporter::exportWeights(std::ofstream &file, const Actor &actor)
    {
        // TODO: Implement weight export for actor model
        // This is a stub implementation

        // In practice, we'd extract the actual weights from the actor model
        // and convert them to the TIDL format

        return true;
    }
#endif

    void TIDLExporter::createDeploymentScript(const std::string &path)
    {
        std::string script_path = path + "/deploy_tidl.sh";
        std::ofstream script(script_path);

        if (script.is_open())
        {
            script << "#!/bin/bash\n"
                   << "# Auto-generated TIDL deployment script\n\n"
                   << "MODEL_DIR=\"$(dirname \"$0\")\"\n"
                   << "export LD_LIBRARY_PATH=\"${LD_LIBRARY_PATH}:${MODEL_DIR}\"\n\n"
                   << "# Run the model with TIDL acceleration\n"
                   << "td3learn_run \\\n"
                   << "  --model \"${MODEL_DIR}/model\" \\\n"
                   << "  --tidl \"${MODEL_DIR}\" \\\n"
                   << "  --hardware \"$@\"\n";

            script.close();

            // Make script executable
            std::filesystem::permissions(
                script_path,
                std::filesystem::perms::owner_exec |
                    std::filesystem::perms::group_exec |
                    std::filesystem::perms::others_exec,
                std::filesystem::perm_options::add);
        }
    }

} // namespace td3learn
