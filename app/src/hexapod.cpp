#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string>
#include "hexapod.hpp"

// Device file path
#define HEXAPOD_DEVICE "/dev/hexapod"

// IOCTL commands - must match kernel definitions
#define HEXAPOD_IOC_MAGIC 'H'
#define HEXAPOD_IOCTL_SET_LEG _IOW(HEXAPOD_IOC_MAGIC, 1, leg_command_t)
#define HEXAPOD_IOCTL_GET_IMU _IOR(HEXAPOD_IOC_MAGIC, 2, ImuData)
#define HEXAPOD_IOCTL_CALIBRATE _IOW(HEXAPOD_IOC_MAGIC, 3, Calibration)
#define HEXAPOD_IOCTL_CENTER_ALL _IO(HEXAPOD_IOC_MAGIC, 4)

// Legacy C structures for compatibility with kernel
typedef struct
{
    int16_t hip;
    int16_t knee;
    int16_t ankle;
} leg_position_t;

typedef struct
{
    uint8_t leg_num;
    leg_position_t position;
} leg_command_t;

// Implementation class using PIMPL idiom
class HexapodImpl
{
public:
    int fd;
    bool initialized;
    LegPosition leg_positions[NUM_LEGS];
    ImuData imu_data;
    int last_error_code;
    std::string last_error_message;

    HexapodImpl() : fd(-1), initialized(false), last_error_code(0)
    {
        for (int i = 0; i < NUM_LEGS; i++)
        {
            leg_positions[i] = LegPosition();
        }
    }

    void setError(int code, const std::string &message)
    {
        last_error_code = code;
        last_error_message = message;
        if (code != 0)
        {
            fprintf(stderr, "Hexapod error: %s\n", message.c_str());
        }
    }
};

// LegPosition implementation
LegPosition::LegPosition(int16_t hip, int16_t knee, int16_t ankle)
    : hip(hip), knee(knee), ankle(ankle) {}

// Hexapod implementation
Hexapod::Hexapod() : pImpl(new HexapodImpl()) {}

Hexapod::~Hexapod()
{
    cleanup();
}

Hexapod::Hexapod(Hexapod &&other) noexcept : pImpl(std::move(other.pImpl)) {}

Hexapod &Hexapod::operator=(Hexapod &&other) noexcept
{
    if (this != &other)
    {
        pImpl = std::move(other.pImpl);
    }
    return *this;
}

bool Hexapod::init()
{
    if (pImpl->initialized)
    {
        return true;
    }

    pImpl->fd = open(HEXAPOD_DEVICE, O_RDWR);
    if (pImpl->fd < 0)
    {
        int err = errno;
        pImpl->setError(err, std::string("Failed to open hexapod device: ") + strerror(err));
        return false;
    }

    pImpl->initialized = true;
    printf("Hexapod initialized successfully\n");
    return true;
}

void Hexapod::cleanup()
{
    if (pImpl && pImpl->initialized)
    {
        if (pImpl->fd >= 0)
        {
            close(pImpl->fd);
            pImpl->fd = -1;
        }
        pImpl->initialized = false;
    }
}

bool Hexapod::setLegPosition(uint8_t leg_num, const LegPosition &position)
{
    if (!pImpl->initialized || leg_num >= NUM_LEGS)
    {
        pImpl->setError(EINVAL, "Invalid leg number or not initialized");
        return false;
    }

    if (pImpl->fd < 0)
    {
        pImpl->setError(EBADF, "Invalid file descriptor");
        return false;
    }

#ifdef DEBUG_POSITIONS
    // More efficient debug approach - only print every 100 calls and only for specific legs
    static unsigned int calls = 0;
    if (++calls % 100 == 0 && (leg_num == 0 || leg_num == 3))
    {
        printf("Leg %d at %.2fs: hip=%d, knee=%d, ankle=%d\n",
               leg_num, getCurrentTime(), position.hip, position.knee, position.ankle);
    }
#endif

    // Prepare command using legacy struct for ioctl compatibility
    leg_command_t cmd;
    cmd.leg_num = leg_num;
    cmd.position.hip = position.hip;
    cmd.position.knee = position.knee;
    cmd.position.ankle = position.ankle;

    // Send command to kernel
    int ret = ioctl(pImpl->fd, HEXAPOD_IOCTL_SET_LEG, &cmd);
    if (ret < 0)
    {
        int err = errno;
        pImpl->setError(err, std::string("Failed to set leg position: ") + strerror(err));
#ifdef DEBUG_ERRORS
        // Print a more detailed error message
        printf("ioctl SET_LEG failed with error %d (%s) for leg %d\n",
               err, strerror(err), leg_num);
#endif
        return false;
    }

    // Store position in local state
    pImpl->leg_positions[leg_num] = position;
    return true;
}

bool Hexapod::getLegPosition(uint8_t leg_num, LegPosition &position) const
{
    if (!pImpl->initialized || leg_num >= NUM_LEGS)
    {
        const_cast<HexapodImpl *>(pImpl.get())->setError(EINVAL, "Invalid leg number or not initialized");
        return false;
    }

    position = pImpl->leg_positions[leg_num];
    return true;
}

bool Hexapod::getImuData(ImuData &data) const
{
    if (!pImpl->initialized)
    {
        const_cast<HexapodImpl *>(pImpl.get())->setError(EINVAL, "Hexapod not initialized");
        return false;
    }

    if (pImpl->fd < 0)
    {
        const_cast<HexapodImpl *>(pImpl.get())->setError(EBADF, "Invalid file descriptor");
        return false;
    }

    int ret = ioctl(pImpl->fd, HEXAPOD_IOCTL_GET_IMU, &data);
    if (ret < 0)
    {
        int err = errno;
        const_cast<HexapodImpl *>(pImpl.get())->setError(err, std::string("Failed to get IMU data: ") + strerror(err));
        return false;
    }

    pImpl->imu_data = data;
    return true;
}

bool Hexapod::setCalibration(uint8_t leg_num, int16_t hip_offset, int16_t knee_offset, int16_t ankle_offset)
{
    if (!pImpl->initialized || leg_num >= NUM_LEGS)
    {
        pImpl->setError(EINVAL, "Invalid leg number or not initialized");
        return false;
    }

    if (pImpl->fd < 0)
    {
        pImpl->setError(EBADF, "Invalid file descriptor");
        return false;
    }

    Calibration cal;
    cal.leg_num = leg_num;
    cal.hip_offset = hip_offset;
    cal.knee_offset = knee_offset;
    cal.ankle_offset = ankle_offset;

    int ret = ioctl(pImpl->fd, HEXAPOD_IOCTL_CALIBRATE, &cal);
    if (ret < 0)
    {
        int err = errno;
        pImpl->setError(err, std::string("Failed to set calibration: ") + strerror(err));
        return false;
    }

    return true;
}

bool Hexapod::centerAll()
{
    if (!pImpl->initialized)
    {
        pImpl->setError(EINVAL, "Hexapod not initialized");
        return false;
    }

    if (pImpl->fd < 0)
    {
        pImpl->setError(EBADF, "Invalid file descriptor");
        return false;
    }

    int ret = ioctl(pImpl->fd, HEXAPOD_IOCTL_CENTER_ALL);
    if (ret < 0)
    {
        int err = errno;
        pImpl->setError(err, std::string("Failed to center all legs: ") + strerror(err));
        return false;
    }

    // Clear local state
    for (int i = 0; i < NUM_LEGS; i++)
    {
        pImpl->leg_positions[i] = LegPosition(0, 0, 0);
    }

    return true;
}

std::string Hexapod::getLastErrorMessage() const
{
    return pImpl->last_error_message;
}

int Hexapod::getLastErrorCode() const
{
    return pImpl->last_error_code;
}

void Hexapod::printLegPosition(const LegPosition &position)
{
    printf("Hip: %d, Knee: %d, Ankle: %d\n",
           position.hip, position.knee, position.ankle);
}

void Hexapod::printImuData(const ImuData &data)
{
    printf("\rAccel: X=%+6.2fg Y=%+6.2fg Z=%+6.2fg | Gyro: X=%+7.2f° Y=%+7.2f° Z=%+7.2f°/s",
           data.getAccelX(), data.getAccelY(), data.getAccelZ(),
           data.getGyroX(), data.getGyroY(), data.getGyroZ());
}

// Helper function to get consistent timestamps
double Hexapod::getCurrentTime() const
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + (ts.tv_nsec / 1.0e9);
}
