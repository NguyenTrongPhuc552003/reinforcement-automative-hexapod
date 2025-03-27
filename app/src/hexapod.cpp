#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <time.h>
#include <stdexcept>
#include <system_error>
#include "hexapod.hpp"

//==============================================================================
// Kernel Driver Interface Definitions
//==============================================================================

// Device file path
#define HEXAPOD_DEVICE "/dev/hexapod"

// IOCTL commands - must match kernel definitions
#define HEXAPOD_IOC_MAGIC 'H'
#define HEXAPOD_IOCTL_SET_LEG _IOW(HEXAPOD_IOC_MAGIC, 1, struct hexapod_leg_cmd)
#define HEXAPOD_IOCTL_GET_IMU _IOR(HEXAPOD_IOC_MAGIC, 2, struct hexapod_imu_data)
#define HEXAPOD_IOCTL_CALIBRATE _IOW(HEXAPOD_IOC_MAGIC, 3, struct hexapod_calibration)
#define HEXAPOD_IOCTL_CENTER_ALL _IO(HEXAPOD_IOC_MAGIC, 4)

namespace hexapod
{

    //==============================================================================
    // Kernel-compatible Structure Definitions
    //==============================================================================

    // These structures maintain binary compatibility with kernel driver
    struct hexapod_leg_joint
    {
        int16_t hip;
        int16_t knee;
        int16_t ankle;
    };

    struct hexapod_leg_cmd
    {
        uint8_t leg_num;
        struct hexapod_leg_joint joint;
    };

    struct hexapod_imu_data
    {
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
    };

    struct hexapod_calibration
    {
        uint8_t leg_num;
        struct hexapod_leg_joint offsets;
    };

    //==============================================================================
    // Utility Functions
    //==============================================================================

    /**
     * @brief Get the name of an error category for logging
     *
     * @param category Error category
     * @return const char* Category name as string
     */
    const char *getCategoryName(ErrorCategory category)
    {
        switch (category)
        {
        case ErrorCategory::NONE:
            return "NONE";
        case ErrorCategory::DEVICE:
            return "DEVICE";
        case ErrorCategory::COMMUNICATION:
            return "COMM";
        case ErrorCategory::PARAMETER:
            return "PARAM";
        case ErrorCategory::SYSTEM:
            return "SYSTEM";
        case ErrorCategory::HARDWARE:
            return "HARDWARE";
        default:
            return "UNKNOWN";
        }
    }

    //==============================================================================
    // Implementation Class (PIMPL idiom)
    //==============================================================================

    class HexapodImpl
    {
    public:
        int fd;                                      ///< File descriptor for device
        bool initialized;                            ///< Initialization flag
        LegPosition leg_positions[Config::NUM_LEGS]; ///< Current leg positions
        ImuData imu_data;                            ///< Current IMU data
        ErrorInfo lastError;                         ///< Last error information

        /**
         * @brief Construct a new Hexapod Impl object
         */
        HexapodImpl() : fd(-1), initialized(false)
        {
            // Initialize leg positions
            for (int i = 0; i < Config::NUM_LEGS; i++)
            {
                leg_positions[i] = LegPosition();
                leg_positions[i].leg_num = i;
            }
        }

        /**
         * @brief Set error information with logging
         *
         * @param code Error code
         * @param category Error category
         * @param message Error message
         */
        void setError(int code, ErrorCategory category, const std::string &message)
        {
            lastError = ErrorInfo(code, category, message);
            if (code != 0)
            {
                fprintf(stderr, "Hexapod error [%s-%d]: %s\n",
                        getCategoryName(category), code, message.c_str());
            }
        }

        /**
         * @brief Clear error information
         */
        void clearError()
        {
            lastError = ErrorInfo();
        }

        /**
         * @brief Execute IOCTL with error handling
         *
         * @param request IOCTL request code
         * @param arg IOCTL argument
         * @param errorMessage Error message prefix
         * @return true if IOCTL successful
         * @return false if IOCTL failed
         */
        bool executeIoctl(unsigned long request, void *arg, const char *errorMessage)
        {
            if (!initialized || fd < 0)
            {
                setError(ErrorInfo::ErrorCode::NOT_INITIALIZED, ErrorCategory::PARAMETER,
                         "Hexapod not initialized or invalid file descriptor");
                return false;
            }

            int ret = ioctl(fd, request, arg);
            if (ret < 0)
            {
                setError(ErrorInfo::ErrorCode::COMM_ERROR, ErrorCategory::COMMUNICATION,
                         std::string(errorMessage) + ": " + strerror(errno));
                return false;
            }

            return true;
        }
    };

    //==============================================================================
    // LegPosition Implementation
    //==============================================================================

    LegPosition::LegPosition(int16_t hip, int16_t knee, int16_t ankle) : leg_num(0)
    {
        joints.hip = hip;
        joints.knee = knee;
        joints.ankle = ankle;
    }

    LegPosition::LegPosition(const LegPosition &other) : leg_num(other.leg_num)
    {
        joints.hip = other.joints.hip;
        joints.knee = other.joints.knee;
        joints.ankle = other.joints.ankle;
    }

    LegPosition &LegPosition::operator=(const LegPosition &other)
    {
        if (this != &other)
        {
            leg_num = other.leg_num;
            joints.hip = other.joints.hip;
            joints.knee = other.joints.knee;
            joints.ankle = other.joints.ankle;
        }
        return *this;
    }

    //==============================================================================
    // Hexapod Class Implementation
    //==============================================================================

    // Constructor and Destructor
    Hexapod::Hexapod() : pImpl(std::make_unique<HexapodImpl>()) {}

    Hexapod::~Hexapod()
    {
        cleanup();
    }

    // Move operations
    Hexapod::Hexapod(Hexapod &&other) noexcept : pImpl(std::move(other.pImpl)) {}

    Hexapod &Hexapod::operator=(Hexapod &&other) noexcept
    {
        if (this != &other)
        {
            pImpl = std::move(other.pImpl);
        }
        return *this;
    }

    // Initialization
    bool Hexapod::init()
    {
        // If already initialized, just return success
        if (pImpl->initialized)
        {
            return true;
        }

        // Clear any previous errors
        pImpl->clearError();

        // Try to open the device
        pImpl->fd = open(HEXAPOD_DEVICE, O_RDWR);
        if (pImpl->fd < 0)
        {
            // Handle different error cases with specific messages
            if (access(HEXAPOD_DEVICE, F_OK) != 0)
            {
                pImpl->setError(ErrorInfo::ErrorCode::DEVICE_NOT_FOUND, ErrorCategory::DEVICE,
                                "Device node not found. Is the driver loaded? Try 'sudo modprobe hexapod_driver'");
            }
            else if (access(HEXAPOD_DEVICE, R_OK | W_OK) != 0)
            {
                pImpl->setError(ErrorInfo::ErrorCode::PERMISSION_DENIED, ErrorCategory::SYSTEM,
                                "Permission denied. Try running with sudo or add your user to the correct group.");
            }
            else
            {
                pImpl->setError(ErrorInfo::ErrorCode::IO_ERROR, ErrorCategory::COMMUNICATION,
                                std::string("Failed to open hexapod device: ") + strerror(errno));
            }
            return false;
        }

        // Set initialized flag
        pImpl->initialized = true;
        printf("Hexapod interface initialized successfully\n");
        return true;
    }

    // Cleanup resources
    void Hexapod::cleanup()
    {
        if (pImpl && pImpl->initialized)
        {
            // Center legs for safety if we still have a valid fd
            if (pImpl->fd >= 0)
            {
                // Try to center, but don't fail if we can't
                centerAll();

                // Close device file with proper error checking
                if (close(pImpl->fd) < 0)
                {
                    fprintf(stderr, "Warning: Error closing hexapod device: %s\n", strerror(errno));
                }
                pImpl->fd = -1;
            }
            pImpl->initialized = false;
        }
    }

    // Leg Position Control
    bool Hexapod::setLegPosition(uint8_t leg_num, const LegPosition &position)
    {
        // Parameter validation
        if (!pImpl->initialized)
        {
            pImpl->setError(ErrorInfo::ErrorCode::NOT_INITIALIZED, ErrorCategory::PARAMETER,
                            "Hexapod not initialized");
            return false;
        }

        if (leg_num >= Config::NUM_LEGS)
        {
            pImpl->setError(ErrorInfo::ErrorCode::INVALID_PARAM, ErrorCategory::PARAMETER,
                            "Invalid leg number: " + std::to_string(leg_num));
            return false;
        }

        if (pImpl->fd < 0)
        {
            pImpl->setError(ErrorInfo::ErrorCode::BAD_FILE, ErrorCategory::SYSTEM,
                            "Invalid file descriptor");
            return false;
        }

        // Check angle limits
        if (position.getHip() < AngleLimits::HIP_MIN || position.getHip() > AngleLimits::HIP_MAX ||
            position.getKnee() < AngleLimits::KNEE_MIN || position.getKnee() > AngleLimits::KNEE_MAX ||
            position.getAnkle() < AngleLimits::ANKLE_MIN || position.getAnkle() > AngleLimits::ANKLE_MAX)
        {

            pImpl->setError(ErrorInfo::ErrorCode::INVALID_PARAM, ErrorCategory::PARAMETER,
                            "Joint angle out of range");
            return false;
        }

        // Create the command structure for the kernel
        struct hexapod_leg_cmd cmd;
        cmd.leg_num = leg_num;
        cmd.joint.hip = position.getHip();
        cmd.joint.knee = position.getKnee();
        cmd.joint.ankle = position.getAnkle();

        // Send command through IOCTL
        int ret = ioctl(pImpl->fd, HEXAPOD_IOCTL_SET_LEG, &cmd);
        if (ret < 0)
        {
            pImpl->setError(ErrorInfo::ErrorCode::COMM_ERROR, ErrorCategory::COMMUNICATION,
                            std::string("Failed to set leg position: ") + strerror(errno));
            return false;
        }

        // Store position in local state
        pImpl->leg_positions[leg_num] = position;
        pImpl->leg_positions[leg_num].leg_num = leg_num;
        return true;
    }

    bool Hexapod::getLegPosition(uint8_t leg_num, LegPosition &position) const
    {
        // Parameter validation
        if (!pImpl->initialized)
        {
            const_cast<HexapodImpl *>(pImpl.get())->setError(ErrorInfo::ErrorCode::NOT_INITIALIZED, ErrorCategory::PARAMETER, "Hexapod not initialized");
            return false;
        }

        if (leg_num >= Config::NUM_LEGS)
        {
            const_cast<HexapodImpl *>(pImpl.get())->setError(ErrorInfo::ErrorCode::INVALID_PARAM, ErrorCategory::PARAMETER, "Invalid leg number: " + std::to_string(leg_num));
            return false;
        }

        // Return cached position
        position = pImpl->leg_positions[leg_num];
        return true;
    }

    // IMU Data Access
    bool Hexapod::getImuData(ImuData &data) const
    {
        // Parameter validation
        if (!pImpl->initialized)
        {
            const_cast<HexapodImpl *>(pImpl.get())->setError(ErrorInfo::ErrorCode::NOT_INITIALIZED, ErrorCategory::PARAMETER, "Hexapod not initialized");
            return false;
        }

        if (pImpl->fd < 0)
        {
            const_cast<HexapodImpl *>(pImpl.get())->setError(ErrorInfo::ErrorCode::BAD_FILE, ErrorCategory::SYSTEM, "Invalid file descriptor");
            return false;
        }

        // Create kernel-compatible structure
        struct hexapod_imu_data kernel_data;

        // Request data via IOCTL
        int ret = ioctl(pImpl->fd, HEXAPOD_IOCTL_GET_IMU, &kernel_data);
        if (ret < 0)
        {
            const_cast<HexapodImpl *>(pImpl.get())->setError(ErrorInfo::ErrorCode::COMM_ERROR, ErrorCategory::COMMUNICATION, std::string("Failed to get IMU data: ") + strerror(errno));
            return false;
        }

        // Copy data to output parameter
        data.accel_x = kernel_data.accel_x;
        data.accel_y = kernel_data.accel_y;
        data.accel_z = kernel_data.accel_z;
        data.gyro_x = kernel_data.gyro_x;
        data.gyro_y = kernel_data.gyro_y;
        data.gyro_z = kernel_data.gyro_z;

        // Cache data
        pImpl->imu_data = data;
        return true;
    }

    // Calibration
    bool Hexapod::setCalibration(uint8_t leg_num, int16_t hip_offset, int16_t knee_offset, int16_t ankle_offset)
    {
        // Parameter validation
        if (!pImpl->initialized)
        {
            pImpl->setError(ErrorInfo::ErrorCode::NOT_INITIALIZED, ErrorCategory::PARAMETER,
                            "Hexapod not initialized");
            return false;
        }

        if (leg_num >= Config::NUM_LEGS)
        {
            pImpl->setError(ErrorInfo::ErrorCode::INVALID_PARAM, ErrorCategory::PARAMETER,
                            "Invalid leg number: " + std::to_string(leg_num));
            return false;
        }

        if (pImpl->fd < 0)
        {
            pImpl->setError(ErrorInfo::ErrorCode::BAD_FILE, ErrorCategory::SYSTEM,
                            "Invalid file descriptor");
            return false;
        }

        // Create kernel-compatible calibration structure
        struct hexapod_calibration cal;
        cal.leg_num = leg_num;
        cal.offsets.hip = hip_offset;
        cal.offsets.knee = knee_offset;
        cal.offsets.ankle = ankle_offset;

        // Send calibration through IOCTL
        int ret = ioctl(pImpl->fd, HEXAPOD_IOCTL_CALIBRATE, &cal);
        if (ret < 0)
        {
            pImpl->setError(ErrorInfo::ErrorCode::COMM_ERROR, ErrorCategory::COMMUNICATION,
                            std::string("Failed to set calibration: ") + strerror(errno));
            return false;
        }

        return true;
    }

    // Center all legs
    bool Hexapod::centerAll()
    {
        // Validation
        if (!pImpl->initialized)
        {
            pImpl->setError(ErrorInfo::ErrorCode::NOT_INITIALIZED, ErrorCategory::PARAMETER,
                            "Hexapod not initialized");
            return false;
        }

        if (pImpl->fd < 0)
        {
            pImpl->setError(ErrorInfo::ErrorCode::BAD_FILE, ErrorCategory::SYSTEM,
                            "Invalid file descriptor");
            return false;
        }

        // Send center command through IOCTL
        int ret = ioctl(pImpl->fd, HEXAPOD_IOCTL_CENTER_ALL);
        if (ret < 0)
        {
            pImpl->setError(ErrorInfo::ErrorCode::COMM_ERROR, ErrorCategory::COMMUNICATION,
                            std::string("Failed to center all legs: ") + strerror(errno));
            return false;
        }

        // Update local state to reflect centered position
        for (int i = 0; i < Config::NUM_LEGS; i++)
        {
            pImpl->leg_positions[i] = LegPosition(0, 0, 0);
            pImpl->leg_positions[i].leg_num = i;
        }

        return true;
    }

    // Error handling
    ErrorInfo Hexapod::getLastError() const
    {
        return pImpl->lastError;
    }

    std::string Hexapod::getLastErrorMessage() const
    {
        return pImpl->lastError.getMessage();
    }

    int Hexapod::getLastErrorCode() const
    {
        return pImpl->lastError.getCode();
    }

    ErrorCategory Hexapod::getLastErrorCategory() const
    {
        return pImpl->lastError.getCategory();
    }

    // Debug utilities
    void Hexapod::printLegPosition(const LegPosition &position)
    {
        printf("Hip: %d, Knee: %d, Ankle: %d\n",
               position.getHip(), position.getKnee(), position.getAnkle());
    }

    void Hexapod::printImuData(const ImuData &data)
    {
        printf("\rAccel: X=%+6.2fg Y=%+6.2fg Z=%+6.2fg | Gyro: X=%+7.2f° Y=%+7.2f° Z=%+7.2f°/s",
               data.getAccelX(), data.getAccelY(), data.getAccelZ(),
               data.getGyroX(), data.getGyroY(), data.getGyroZ());
    }

    // Time utility
    double Hexapod::getCurrentTime() const
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ts.tv_sec + (ts.tv_nsec / 1.0e9);
    }

} // namespace hexapod
