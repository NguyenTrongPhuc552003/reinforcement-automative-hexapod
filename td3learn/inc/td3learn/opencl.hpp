#ifndef TD3LEARN_OPENCL_HPP
#define TD3LEARN_OPENCL_HPP

#include <memory>
#include <vector>
#include <string>
#include "td3learn/types.hpp"
#include "td3learn/config.hpp"

#ifdef ENABLE_OPENCL
// Forward declaration of OpenCL types to avoid including OpenCL headers
typedef struct _cl_platform_id *cl_platform_id;
typedef struct _cl_device_id *cl_device_id;
typedef struct _cl_context *cl_context;
typedef struct _cl_command_queue *cl_command_queue;
typedef struct _cl_program *cl_program;
typedef struct _cl_kernel *cl_kernel;
typedef struct _cl_mem *cl_mem;
#endif

namespace td3learn
{

    // Forward declarations
    class OpenCLContextImpl;
    class OpenCLKernelImpl;

    /**
     * @brief OpenCL context wrapper
     */
    class OpenCLContext
    {
    public:
        /**
         * @brief Create an OpenCL context
         * @param config OpenCL configuration
         */
        explicit OpenCLContext(const OpenCLConfig &config);

        /**
         * @brief Destroy the OpenCL context
         */
        ~OpenCLContext();

        /**
         * @brief Initialize the context
         * @return Result code
         */
        Result init();

        /**
         * @brief Create kernel from source
         * @param source Kernel source code
         * @param kernel_name Kernel function name
         * @return Kernel object
         */
        class OpenCLKernel createKernel(
            const std::string &source,
            const std::string &kernel_name);

        /**
         * @brief Create kernel from file
         * @param file_path Path to kernel file
         * @param kernel_name Kernel function name
         * @return Kernel object
         */
        class OpenCLKernel createKernelFromFile(
            const std::string &file_path,
            const std::string &kernel_name);

        /**
         * @brief Check if OpenCL acceleration is available
         * @return True if available
         */
        static bool isAvailable();

        /**
         * @brief Get information about available OpenCL platforms and devices
         * @return Information string
         */
        static std::string getDeviceInfo();

    private:
        std::unique_ptr<OpenCLContextImpl> pImpl;
        friend class OpenCLKernel;
    };

    /**
     * @brief OpenCL kernel wrapper
     */
    class OpenCLKernel
    {
    public:
        /**
         * @brief Create an uninitialized kernel
         */
        OpenCLKernel();

        /**
         * @brief Destroy the kernel
         */
        ~OpenCLKernel();

        /**
         * @brief Set kernel arguments
         * @param index Argument index
         * @param size Argument size in bytes
         * @param value Pointer to argument value
         * @return Result code
         */
        Result setArg(int index, size_t size, const void *value);

        /**
         * @brief Set kernel argument vector
         * @param index Argument index
         * @param values Vector of values
         * @return Result code
         */
        template <typename T>
        Result setArgVector(int index, const std::vector<T> &values);

        /**
         * @brief Set kernel argument scalar
         * @param index Argument index
         * @param value Scalar value
         * @return Result code
         */
        template <typename T>
        Result setArgScalar(int index, const T &value);

        /**
         * @brief Execute kernel
         * @param global_size Global work size
         * @param local_size Local work size
         * @return Result code
         */
        Result execute(const std::vector<size_t> &global_size, const std::vector<size_t> &local_size);

        /**
         * @brief Read output data
         * @param index Argument index of output buffer
         * @param[out] output Output vector
         * @return Result code
         */
        template <typename T>
        Result readOutput(int index, std::vector<T> &output);

#ifdef ENABLE_OPENCL
        // Forward declare needed types
        namespace cl
        {
            class Kernel;
        }
        class OpenCLContextImpl;

        // Constructor for internal use by OpenCLContextImpl
        OpenCLKernel(const cl::Kernel &kernel, OpenCLContextImpl *context);
#endif

    private:
        std::unique_ptr<OpenCLKernelImpl> pImpl;
        friend class OpenCLContext;
    };

} // namespace td3learn

#endif // TD3LEARN_OPENCL_HPP
