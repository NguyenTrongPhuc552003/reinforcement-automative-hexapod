#include "td3learn/opencl.hpp"
#include "td3learn/utils.hpp"
#include <string>
#include <vector>
#include <stdexcept>
#include <sstream>
#include <fstream>
#include <iostream>

#ifdef ENABLE_OPENCL
#include <CL/cl.hpp>
#endif

namespace td3learn
{

#ifdef ENABLE_OPENCL
    // Forward declaration of OpenCLKernelImpl for friendship
    class OpenCLKernelImpl;

    // Implementation with real OpenCL support
    class OpenCLContextImpl
    {
    public:
        OpenCLContextImpl(const OpenCLConfig &config) : config(config), initialized(false)
        {
        }

        ~OpenCLContextImpl()
        {
        }

        Result init()
        {
            try
            {
                // Get available platforms
                std::vector<cl::Platform> platforms;
                cl::Platform::get(&platforms);

                if (platforms.empty())
                {
                    utils::Logger::error("No OpenCL platforms found");
                    return Result::ERROR_HARDWARE;
                }

                // Select platform based on config (or default to first)
                unsigned int platform_idx = config.platform_index;
                if (platform_idx >= platforms.size())
                {
                    utils::Logger::warning("Platform index out of range, using platform 0");
                    platform_idx = 0;
                }
                cl::Platform platform = platforms[platform_idx];

                // Get platform info
                platform_name = platform.getInfo<CL_PLATFORM_NAME>();
                platform_vendor = platform.getInfo<CL_PLATFORM_VENDOR>();
                platform_version = platform.getInfo<CL_PLATFORM_VERSION>();

                utils::Logger::info("Using OpenCL platform: " + platform_name + " (" + platform_vendor + ")");
                utils::Logger::debug("Platform version: " + platform_version);

                // Get devices for the selected platform
                std::vector<cl::Device> devices;
                platform.getDevices(CL_DEVICE_TYPE_ALL, &devices);

                if (devices.empty())
                {
                    utils::Logger::error("No OpenCL devices found for platform");
                    return Result::ERROR_HARDWARE;
                }

                // Select device based on config (or default to first)
                unsigned int device_idx = config.device_index;
                if (device_idx >= devices.size())
                {
                    utils::Logger::warning("Device index out of range, using device 0");
                    device_idx = 0;
                }
                device = devices[device_idx];

                // Get device info
                device_name = device.getInfo<CL_DEVICE_NAME>();
                device_vendor = device.getInfo<CL_DEVICE_VENDOR>();
                device_version = device.getInfo<CL_DEVICE_VERSION>();

                utils::Logger::info("Using OpenCL device: " + device_name + " (" + device_vendor + ")");
                utils::Logger::debug("Device version: " + device_version);

                // Create OpenCL context
                context = cl::Context(device);

                // Create command queue with profiling if requested
                if (config.profiling)
                {
                    queue = cl::CommandQueue(context, device, CL_QUEUE_PROFILING_ENABLE);
                }
                else
                {
                    queue = cl::CommandQueue(context, device);
                }

                initialized = true;
                return Result::SUCCESS;
            }
            catch (cl::Error &e)
            {
                std::string error_msg = "OpenCL error: " + std::string(e.what()) + " (" + std::to_string(e.err()) + ")";
                utils::Logger::error(error_msg);
                return Result::ERROR_INITIALIZATION;
            }
            catch (std::exception &e)
            {
                utils::Logger::error("Exception during OpenCL initialization: " + std::string(e.what()));
                return Result::ERROR_INITIALIZATION;
            }
        }

        OpenCLKernel createKernel(const std::string &source, const std::string &kernel_name)
        {
            if (!initialized)
            {
                utils::Logger::error("OpenCL context not initialized");
                throw std::runtime_error("OpenCL context not initialized");
            }

            try
            {
                // Create program from source
                cl::Program::Sources sources;
                sources.push_back(std::make_pair(source.c_str(), source.length()));
                cl::Program program = cl::Program(context, sources);

                // Build program
                try
                {
                    program.build({device});
                }
                catch (cl::Error &e)
                {
                    // Get build log on error
                    std::string build_log = program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device);
                    utils::Logger::error("OpenCL build error: " + build_log);
                    throw;
                }

                // Create kernel
                cl::Kernel kernel = cl::Kernel(program, kernel_name.c_str());

                // Create and return OpenCLKernel object - properly initialize it
                return OpenCLKernel(kernel, this);
            }
            catch (cl::Error &e)
            {
                std::string error_msg = "OpenCL error creating kernel: " + std::string(e.what()) + " (" + std::to_string(e.err()) + ")";
                utils::Logger::error(error_msg);
                throw std::runtime_error(error_msg);
            }
        }

        OpenCLKernel createKernelFromFile(const std::string &file_path, const std::string &kernel_name)
        {
            // Read source from file
            std::ifstream file(file_path);
            if (!file.is_open())
            {
                utils::Logger::error("Failed to open kernel file: " + file_path);
                throw std::runtime_error("Failed to open kernel file");
            }

            std::stringstream buffer;
            buffer << file.rdbuf();
            std::string source = buffer.str();

            return createKernel(source, kernel_name);
        }

        static bool isAvailable()
        {
            try
            {
                std::vector<cl::Platform> platforms;
                cl::Platform::get(&platforms);
                return !platforms.empty();
            }
            catch (...)
            {
                return false;
            }
        }

        static std::string getDeviceInfo()
        {
            std::stringstream ss;
            try
            {
                std::vector<cl::Platform> platforms;
                cl::Platform::get(&platforms);

                if (platforms.empty())
                {
                    return "No OpenCL platforms found";
                }

                ss << "OpenCL Platforms:\n";
                for (size_t i = 0; i < platforms.size(); ++i)
                {
                    cl::Platform &platform = platforms[i];
                    ss << "  [" << i << "] " << platform.getInfo<CL_PLATFORM_NAME>() << " ("
                       << platform.getInfo<CL_PLATFORM_VENDOR>() << ")\n";
                    ss << "      Version: " << platform.getInfo<CL_PLATFORM_VERSION>() << "\n";

                    try
                    {
                        std::vector<cl::Device> devices;
                        platform.getDevices(CL_DEVICE_TYPE_ALL, &devices);

                        ss << "      Devices:\n";
                        for (size_t j = 0; j < devices.size(); ++j)
                        {
                            cl::Device &device = devices[j];
                            ss << "        [" << j << "] " << device.getInfo<CL_DEVICE_NAME>() << "\n";
                            ss << "            Vendor: " << device.getInfo<CL_DEVICE_VENDOR>() << "\n";
                            ss << "            Version: " << device.getInfo<CL_DEVICE_VERSION>() << "\n";
                            ss << "            Global Mem: " << (device.getInfo<CL_DEVICE_GLOBAL_MEM_SIZE>() / (1024 * 1024)) << " MB\n";
                            ss << "            Max Work Group Size: " << device.getInfo<CL_DEVICE_MAX_WORK_GROUP_SIZE>() << "\n";
                        }
                    }
                    catch (...)
                    {
                        ss << "      Error querying devices\n";
                    }
                }

                return ss.str();
            }
            catch (cl::Error &e)
            {
                return "OpenCL error while getting device info: " + std::string(e.what()) + " (" + std::to_string(e.err()) + ")";
            }
            catch (...)
            {
                return "Unknown error while getting OpenCL device info";
            }
        }

        // Data members
        OpenCLConfig config;
        bool initialized;

        // OpenCL objects
        cl::Platform platform;
        cl::Device device;
        cl::Context context;
        cl::CommandQueue queue;

        // Info strings
        std::string platform_name;
        std::string platform_vendor;
        std::string platform_version;
        std::string device_name;
        std::string device_vendor;
        std::string device_version;

        friend class OpenCLKernelImpl; // Give access to OpenCLKernelImpl
    };

    class OpenCLKernelImpl
    {
    public:
        OpenCLKernelImpl() : context(nullptr)
        {
        }

        // Constructor that properly initializes kernel
        OpenCLKernelImpl(const cl::Kernel &k, OpenCLContextImpl *ctx)
            : kernel(k), context(ctx)
        {
        }

        ~OpenCLKernelImpl()
        {
        }

        Result setArg(int index, size_t size, const void *value)
        {
            if (!context || !kernel())
            {
                utils::Logger::error("OpenCL kernel not initialized");
                return Result::ERROR_INITIALIZATION;
            }

            try
            {
                kernel.setArg(index, size, value);
                return Result::SUCCESS;
            }
            catch (cl::Error &e)
            {
                std::string error_msg = "OpenCL error setting kernel arg: " + std::string(e.what()) + " (" + std::to_string(e.err()) + ")";
                utils::Logger::error(error_msg);
                return Result::ERROR_INVALID_ARGUMENT;
            }
        }

        template <typename T>
        Result setArgVector(int index, const std::vector<T> &values)
        {
            if (values.empty())
            {
                utils::Logger::error("Empty vector passed to setArgVector");
                return Result::ERROR_INVALID_ARGUMENT;
            }

            try
            {
                // Create OpenCL buffer
                cl::Buffer buffer = cl::Buffer(
                    context->context,
                    CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                    values.size() * sizeof(T),
                    const_cast<T *>(values.data()));

                // Set kernel argument to buffer
                kernel.setArg(index, buffer);

                // Store buffer for later use
                buffers.push_back(buffer);
                return Result::SUCCESS;
            }
            catch (cl::Error &e)
            {
                std::string error_msg = "OpenCL error setting vector arg: " + std::string(e.what()) + " (" + std::to_string(e.err()) + ")";
                utils::Logger::error(error_msg);
                return Result::ERROR_EXECUTION;
            }
        }

        template <typename T>
        Result setArgScalar(int index, T value)
        {
            try
            {
                kernel.setArg(index, value);
                return Result::SUCCESS;
            }
            catch (cl::Error &e)
            {
                std::string error_msg = "OpenCL error setting scalar arg: " + std::string(e.what()) + " (" + std::to_string(e.err()) + ")";
                utils::Logger::error(error_msg);
                return Result::ERROR_INVALID_ARGUMENT;
            }
        }

        Result execute(const std::vector<size_t> &global_size, const std::vector<size_t> &local_size = {})
        {
            if (!context || !kernel())
            {
                utils::Logger::error("OpenCL kernel not initialized");
                return Result::ERROR_INITIALIZATION;
            }

            try
            {
                cl::NDRange global;
                cl::NDRange local;

                // Set up global range
                switch (global_size.size())
                {
                case 1:
                    global = cl::NDRange(global_size[0]);
                    break;
                case 2:
                    global = cl::NDRange(global_size[0], global_size[1]);
                    break;
                case 3:
                    global = cl::NDRange(global_size[0], global_size[1], global_size[2]);
                    break;
                default:
                    utils::Logger::error("Invalid global size dimension: " + std::to_string(global_size.size()));
                    return Result::ERROR_INVALID_ARGUMENT;
                }

                // Set up local range if provided
                if (!local_size.empty())
                {
                    switch (local_size.size())
                    {
                    case 1:
                        local = cl::NDRange(local_size[0]);
                        break;
                    case 2:
                        local = cl::NDRange(local_size[0], local_size[1]);
                        break;
                    case 3:
                        local = cl::NDRange(local_size[0], local_size[1], local_size[2]);
                        break;
                    default:
                        utils::Logger::error("Invalid local size dimension: " + std::to_string(local_size.size()));
                        return Result::ERROR_INVALID_ARGUMENT;
                    }
                }
                else
                {
                    // Use default local size
                    local = cl::NullRange;
                }

                // Execute kernel
                if (!local_size.empty())
                {
                    context->queue.enqueueNDRangeKernel(kernel, cl::NullRange, global, local);
                }
                else
                {
                    context->queue.enqueueNDRangeKernel(kernel, cl::NullRange, global, cl::NullRange);
                }

                // Finish queue to ensure kernel execution completes
                context->queue.finish();
                return Result::SUCCESS;
            }
            catch (cl::Error &e)
            {
                std::string error_msg = "OpenCL error executing kernel: " + std::string(e.what()) + " (" + std::to_string(e.err()) + ")";
                utils::Logger::error(error_msg);
                return Result::ERROR_EXECUTION;
            }
        }

        template <typename T>
        Result readOutput(int buffer_index, std::vector<T> &output)
        {
            if (buffer_index < 0 || static_cast<size_t>(buffer_index) >= buffers.size())
            {
                utils::Logger::error("Invalid buffer index: " + std::to_string(buffer_index));
                return Result::ERROR_INVALID_ARGUMENT;
            }

            try
            {
                // Get buffer size
                size_t buffer_size = buffers[buffer_index].getInfo<CL_MEM_SIZE>();
                size_t element_count = buffer_size / sizeof(T);

                // Resize output vector
                output.resize(element_count);

                // Read buffer
                context->queue.enqueueReadBuffer(
                    buffers[buffer_index],
                    CL_TRUE, // blocking read
                    0,
                    buffer_size,
                    output.data());

                return Result::SUCCESS;
            }
            catch (cl::Error &e)
            {
                std::string error_msg = "OpenCL error reading output: " + std::string(e.what()) + " (" + std::to_string(e.err()) + ")";
                utils::Logger::error(error_msg);
                return Result::ERROR_EXECUTION;
            }
        }

        // Data members
        cl::Kernel kernel;
        OpenCLContextImpl *context;
        std::vector<cl::Buffer> buffers;
    };

#else
    // Stub implementation when OpenCL is not available
    class OpenCLContextImpl
    {
    public:
        OpenCLContextImpl(const OpenCLConfig &config)
        {
            utils::Logger::warning("OpenCL support not enabled at compile time");
        }

        ~OpenCLContextImpl() {}

        Result init()
        {
            return Result::ERROR_NOT_IMPLEMENTED;
        }

        OpenCLKernel createKernel(const std::string &source, const std::string &kernel_name)
        {
            utils::Logger::warning("OpenCL support not enabled at compile time");
            throw std::runtime_error("OpenCL not supported in this build");
        }

        OpenCLKernel createKernelFromFile(const std::string &file_path, const std::string &kernel_name)
        {
            utils::Logger::warning("OpenCL support not enabled at compile time");
            throw std::runtime_error("OpenCL not supported in this build");
        }

        static bool isAvailable()
        {
            return false;
        }

        static std::string getDeviceInfo()
        {
            return "OpenCL support not enabled at compile time";
        }
    };

    class OpenCLKernelImpl
    {
    public:
        OpenCLKernelImpl() {}
        ~OpenCLKernelImpl() {}

        Result setArg(int index, size_t size, const void *value)
        {
            return Result::ERROR_NOT_IMPLEMENTED;
        }

        template <typename T>
        Result setArgVector(int index, const std::vector<T> &values)
        {
            return Result::ERROR_NOT_IMPLEMENTED;
        }

        template <typename T>
        Result setArgScalar(int index, T value)
        {
            return Result::ERROR_NOT_IMPLEMENTED;
        }

        Result execute(const std::vector<size_t> &global_size, const std::vector<size_t> &local_size = {})
        {
            return Result::ERROR_NOT_IMPLEMENTED;
        }

        template <typename T>
        Result readOutput(int buffer_index, std::vector<T> &output)
        {
            return Result::ERROR_NOT_IMPLEMENTED;
        }
    };
#endif

    // OpenCLContext implementation
    OpenCLContext::OpenCLContext(const OpenCLConfig &config)
        : pImpl(std::make_unique<OpenCLContextImpl>(config))
    {
    }

    OpenCLContext::~OpenCLContext() = default;

    Result OpenCLContext::init()
    {
        return pImpl->init();
    }

    OpenCLKernel OpenCLContext::createKernel(const std::string &source, const std::string &kernel_name)
    {
        return pImpl->createKernel(source, kernel_name);
    }

    OpenCLKernel OpenCLContext::createKernelFromFile(const std::string &file_path, const std::string &kernel_name)
    {
        return pImpl->createKernelFromFile(file_path, kernel_name);
    }

    bool OpenCLContext::isAvailable()
    {
        return OpenCLContextImpl::isAvailable();
    }

    std::string OpenCLContext::getDeviceInfo()
    {
        return OpenCLContextImpl::getDeviceInfo();
    }

    // OpenCLKernel implementation
    OpenCLKernel::OpenCLKernel()
        : pImpl(std::make_unique<OpenCLKernelImpl>())
    {
    }

    // New constructor for proper initialization from OpenCLContextImpl::createKernel
#ifdef ENABLE_OPENCL
    OpenCLKernel::OpenCLKernel(const cl::Kernel &kernel, OpenCLContextImpl *context)
        : pImpl(std::make_unique<OpenCLKernelImpl>(kernel, context))
    {
    }
#endif

    OpenCLKernel::~OpenCLKernel() = default;

    Result OpenCLKernel::setArg(int index, size_t size, const void *value)
    {
        return pImpl->setArg(index, size, value);
    }

    template <typename T>
    Result OpenCLKernel::setArgVector(int index, const std::vector<T> &values)
    {
        return pImpl->setArgVector(index, values);
    }

    template <typename T>
    Result OpenCLKernel::setArgScalar(int index, const T &value)
    {
        return pImpl->setArgScalar(index, value);
    }

    Result OpenCLKernel::execute(const std::vector<size_t> &global_size, const std::vector<size_t> &local_size)
    {
        return pImpl->execute(global_size, local_size);
    }

    template <typename T>
    Result OpenCLKernel::readOutput(int buffer_index, std::vector<T> &output)
    {
        return pImpl->readOutput(buffer_index, output);
    }

    // Template instantiations for common types
    template Result OpenCLKernel::setArgVector<float>(int, const std::vector<float> &);
    template Result OpenCLKernel::setArgVector<int>(int, const std::vector<int> &);
    template Result OpenCLKernel::setArgVector<double>(int, const std::vector<double> &);

    template Result OpenCLKernel::setArgScalar<float>(int, const float &);
    template Result OpenCLKernel::setArgScalar<int>(int, const int &);
    template Result OpenCLKernel::setArgScalar<double>(int, const double &);

    template Result OpenCLKernel::readOutput<float>(int, std::vector<float> &);
    template Result OpenCLKernel::readOutput<int>(int, std::vector<int> &);
    template Result OpenCLKernel::readOutput<double>(int, std::vector<double> &);

} // namespace td3learn
