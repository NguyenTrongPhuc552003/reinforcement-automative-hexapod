#ifndef TD3LEARN_TIDL_HPP
#define TD3LEARN_TIDL_HPP

#include <memory>
#include <vector>
#include <string>
#include "td3learn/types.hpp"
#include "td3learn/config.hpp"

namespace td3learn
{

    // Forward declarations
    class TIDLModelImpl;
    class TIDLExecutorImpl;

    /**
     * @brief TIDL model wrapper for accelerated inference
     */
    class TIDLModel
    {
    public:
        /**
         * @brief Create a TIDL model
         * @param config TIDL configuration
         */
        explicit TIDLModel(const TIDLConfig &config);

        /**
         * @brief Destroy the TIDL model
         */
        ~TIDLModel();

        /**
         * @brief Initialize the model
         * @return Result code
         */
        Result init();

        /**
         * @brief Load from file
         * @param path Model file path
         * @return Result code
         */
        Result load(const std::string &path);

        /**
         * @brief Save to file
         * @param path Output file path
         * @return Result code
         */
        Result save(const std::string &path) const;

        /**
         * @brief Get input shape
         * @return Vector of input dimensions
         */
        std::vector<int> getInputShape() const;

        /**
         * @brief Get output shape
         * @return Vector of output dimensions
         */
        std::vector<int> getOutputShape() const;

    private:
        std::unique_ptr<TIDLModelImpl> pImpl;
        friend class TIDLExecutor;
    };

    /**
     * @brief TIDL execution engine
     */
    class TIDLExecutor
    {
    public:
        /**
         * @brief Create a TIDL executor
         * @param model TIDL model
         */
        explicit TIDLExecutor(const TIDLModel &model);

        /**
         * @brief Destroy the TIDL executor
         */
        ~TIDLExecutor();

        /**
         * @brief Initialize the executor
         * @return Result code
         */
        Result init();

        /**
         * @brief Execute inference
         * @param input Input data
         * @param[out] output Output data
         * @return Result code
         */
        Result execute(const std::vector<Scalar> &input, std::vector<Scalar> &output);

        /**
         * @brief Check if TIDL acceleration is available
         * @return True if available
         */
        static bool isAvailable();

        /**
         * @brief Get information about available TIDL devices
         * @return Information string
         */
        static std::string getDeviceInfo();

    private:
        std::unique_ptr<TIDLExecutorImpl> pImpl;
    };

    /**
     * @brief Utilities for converting between formats
     */
    class TIDLConverter
    {
    public:
        /**
         * @brief Convert neural network model to TIDL format
         * @param source_path Source model path
         * @param target_path Target TIDL model path
         * @param config TIDL configuration
         * @return Result code
         */
        static Result convertModel(
            const std::string &source_path,
            const std::string &target_path,
            const TIDLConfig &config);
    };

} // namespace td3learn

#endif // TD3LEARN_TIDL_HPP
