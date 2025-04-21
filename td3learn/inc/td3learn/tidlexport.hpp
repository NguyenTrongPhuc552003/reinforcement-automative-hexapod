#ifndef TD3LEARN_TIDLEXPORT_HPP
#define TD3LEARN_TIDLEXPORT_HPP

#include <string>
#include <memory>
#include <vector>
#include "td3learn/types.hpp"
#include "td3learn/config.hpp"
#include "td3learn/models.hpp"
#include "td3learn/utils.hpp"

#ifdef ENABLE_TIDL
// Forward declarations for TIDL types to avoid including headers in our header
class Configuration;
#endif

namespace td3learn
{

    /**
     * @brief TIDL exporter for hardware acceleration on BeagleBone AI
     *
     * This class handles converting TD3Learn models to the format required by
     * TI Deep Learning (TIDL) API for hardware acceleration on DSP and EVE cores.
     */
    class TIDLExporter
    {
    public:
        /**
         * @brief Constructor with configuration
         * @param config TIDL configuration parameters
         */
        explicit TIDLExporter(const TIDLConfig &config);

        /**
         * @brief Destructor - cleans up TIDL resources
         */
        ~TIDLExporter();

        /**
         * @brief Initialize the TIDL exporter
         * @return Result code
         */
        Result init();

        /**
         * @brief Clean up TIDL resources
         */
        void cleanup();

        /**
         * @brief Export actor model to TIDL format
         *
         * @param path Path to save exported model
         * @param actor Actor to export
         * @return Result code
         */
        Result exportModel(const std::string &path, const std::unique_ptr<Actor> &actor);

    private:
        /**
         * @brief Create a deployment script for the exported model
         * @param path Output directory path
         */
        void createDeploymentScript(const std::string &path);

#ifdef ENABLE_TIDL
        /**
         * @brief Set up TIDL configuration parameters
         * @param tidl_config Configuration object to set up
         */
        void setupTIDLConfiguration(Configuration &tidl_config);

        /**
         * @brief Export network structure to TIDL format
         *
         * @param network_path Path to save network structure
         * @param params_path Path to save parameters
         * @param actor Actor model to export
         * @return True if successful, false otherwise
         */
        bool exportNetworkStructure(
            const std::string &network_path,
            const std::string &params_path,
            const Actor &actor);

        /**
         * @brief Compute number of layers in the model
         * @param actor Actor model to analyze
         * @return Number of layers
         */
        int computeLayerCount(const Actor &actor);

        /**
         * @brief Export layer definitions
         * @param file Output file stream
         * @param actor Actor model to export
         * @return True if successful, false otherwise
         */
        bool exportLayers(std::ofstream &file, const Actor &actor);

        /**
         * @brief Export weights and biases
         * @param file Output file stream
         * @param actor Actor model to export
         * @return True if successful, false otherwise
         */
        bool exportWeights(std::ofstream &file, const Actor &actor);

        /**
         * @brief Network header structure for TIDL format
         */
        struct NetHeader
        {
            uint16_t version;
            uint16_t layer_count;
            uint16_t input_count;
            uint16_t output_count;
        };
#endif

        // Member variables
        TIDLConfig config_;
        bool initialized_;
    };

} // namespace td3learn

#endif // TD3LEARN_TIDLEXPORT_HPP
