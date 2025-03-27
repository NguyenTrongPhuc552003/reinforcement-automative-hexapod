#include <cstdio>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <system_error>
#include <unistd.h>
#include "calibration.hpp"

// Default calibration filename
#define DEFAULT_CALIBRATION_FILE "calibration.cfg"

namespace calibration
{

    //==============================================================================
    // Exception Implementation
    //==============================================================================

    CalibrationException::CalibrationException(const std::string &message) : message(message) {}

    const char *CalibrationException::what() const noexcept
    {
        return message.c_str();
    }

    //==============================================================================
    // Implementation Class (PIMPL idiom)
    //==============================================================================

    class CalibrationManagerImpl
    {
    public:
        CalibrationManagerImpl() = default;

        /**
         * @brief Maximum allowed calibration offsets
         *
         * These values represent the maximum offsets that can be safely applied.
         * Exceeding these may risk mechanical damage.
         */
        struct CalibrationLimits
        {
            static constexpr int16_t MAX_HIP_OFFSET = 30;
            static constexpr int16_t MAX_KNEE_OFFSET = 30;
            static constexpr int16_t MAX_ANKLE_OFFSET = 30;
        };

        /**
         * @brief File header for calibration files
         *
         * Used to validate file format
         */
        static constexpr uint32_t FILE_MAGIC = 0x48435042; // 'HCPB' in ASCII
        static constexpr uint32_t FILE_VERSION = 1;

        /**
         * @brief File header structure
         */
        struct FileHeader
        {
            uint32_t magic;
            uint32_t version;
            uint32_t num_entries;
        };
    };

    //==============================================================================
    // Singleton Implementation
    //==============================================================================

    CalibrationManager &CalibrationManager::getInstance()
    {
        static CalibrationManager instance;
        return instance;
    }

    CalibrationManager::CalibrationManager() : pImpl(std::make_unique<CalibrationManagerImpl>()) {}

    //==============================================================================
    // Static Helper Methods
    //==============================================================================

    std::string CalibrationManager::getDefaultCalibrationPath()
    {
        char *home_path = getenv("HOME");
        if (home_path)
        {
            return std::string(home_path) + "/.config/hexapod/" + DEFAULT_CALIBRATION_FILE;
        }
        else
        {
            // Fallback to current directory
            return DEFAULT_CALIBRATION_FILE;
        }
    }

    //==============================================================================
    // File Operations
    //==============================================================================

    bool CalibrationManager::loadCalibration(std::vector<Calibration> &calibrations,
                                             const std::string &filename)
    {
        // Determine filename to use
        std::string path = filename.empty() ? getDefaultCalibrationPath() : filename;

        // Clear the output vector
        calibrations.clear();

        // Try to open the file
        std::ifstream file(path, std::ios::binary);
        if (!file)
        {
            fprintf(stderr, "Failed to open calibration file: %s\n", path.c_str());
            return false;
        }

        try
        {
            // Read file header
            CalibrationManagerImpl::FileHeader header;
            file.read(reinterpret_cast<char *>(&header), sizeof(header));

            // Validate file format
            if (header.magic != CalibrationManagerImpl::FILE_MAGIC)
            {
                fprintf(stderr, "Invalid calibration file format\n");
                return false;
            }

            // Check version compatibility
            if (header.version != CalibrationManagerImpl::FILE_VERSION)
            {
                fprintf(stderr, "Unsupported calibration file version: %u\n", header.version);
                return false;
            }

            // Validate entry count
            if (header.num_entries > 100)
            { // Sanity check to prevent huge allocations
                fprintf(stderr, "Invalid number of calibration entries: %u\n", header.num_entries);
                return false;
            }

            // Read calibration entries
            calibrations.resize(header.num_entries);
            file.read(reinterpret_cast<char *>(calibrations.data()),
                      sizeof(Calibration) * header.num_entries);

            // Verify we read the expected number of legs
            if (calibrations.size() != hexapod::Config::NUM_LEGS)
            {
                fprintf(stderr, "Warning: Expected %d legs, loaded %zu\n",
                        hexapod::Config::NUM_LEGS, calibrations.size());
                // Continue anyway - might be a different configuration
            }

            return !file.fail();
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception while loading calibration: %s\n", e.what());
            return false;
        }
    }

    bool CalibrationManager::saveCalibration(const std::vector<Calibration> &calibrations,
                                             const std::string &filename)
    {
        // Determine filename to use
        std::string path = filename.empty() ? getDefaultCalibrationPath() : filename;

        // Ensure directory exists
        if (path == getDefaultCalibrationPath())
        {
            try
            {
                // Create ~/.config/hexapod/ if it doesn't exist
                std::filesystem::path dir_path = std::filesystem::path(path).parent_path();
                if (!std::filesystem::exists(dir_path))
                {
                    std::filesystem::create_directories(dir_path);
                }
            }
            catch (const std::filesystem::filesystem_error &e)
            {
                fprintf(stderr, "Failed to create directory: %s\n", e.what());
                return false;
            }
        }

        // Try to open the file
        std::ofstream file(path, std::ios::binary | std::ios::trunc);
        if (!file)
        {
            fprintf(stderr, "Failed to open calibration file for writing: %s\n", path.c_str());
            return false;
        }

        try
        {
            // Write file header
            CalibrationManagerImpl::FileHeader header;
            header.magic = CalibrationManagerImpl::FILE_MAGIC;
            header.version = CalibrationManagerImpl::FILE_VERSION;
            header.num_entries = static_cast<uint32_t>(calibrations.size());

            file.write(reinterpret_cast<const char *>(&header), sizeof(header));

            // Write calibration entries
            file.write(reinterpret_cast<const char *>(calibrations.data()),
                       sizeof(Calibration) * calibrations.size());

            file.flush();
            return !file.fail();
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception while saving calibration: %s\n", e.what());
            return false;
        }
    }

    //==============================================================================
    // Calibration Operations
    //==============================================================================

    std::vector<Calibration> CalibrationManager::getDefaultCalibration()
    {
        std::vector<Calibration> defaults(hexapod::Config::NUM_LEGS);

        for (int i = 0; i < hexapod::Config::NUM_LEGS; i++)
        {
            defaults[i].leg_num = i;
            defaults[i].hip_offset = 0;
            defaults[i].knee_offset = 0;
            defaults[i].ankle_offset = 0;
        }

        return defaults;
    }

    bool CalibrationManager::applyCalibration(hexapod::Hexapod &hexapod,
                                              const std::vector<Calibration> &calibrations)
    {
        if (calibrations.empty())
        {
            fprintf(stderr, "No calibration data to apply\n");
            return false;
        }

        // Before applying, validate the calibrations
        if (!validateCalibration(calibrations))
        {
            fprintf(stderr, "Calibration validation failed\n");
            return false;
        }

        // Apply calibration to each leg
        size_t success_count = 0;
        for (const auto &cal : calibrations)
        {
            // Skip entries with invalid leg numbers
            if (cal.leg_num >= hexapod::Config::NUM_LEGS)
            {
                fprintf(stderr, "Invalid leg number in calibration: %d\n", cal.leg_num);
                continue;
            }

            // Apply calibration with retry logic
            bool success = false;
            for (int retry = 0; retry < 3 && !success; retry++)
            {
                if (hexapod.setCalibration(cal.leg_num, cal.hip_offset,
                                           cal.knee_offset, cal.ankle_offset))
                {
                    success = true;
                    success_count++;
                }
                else if (retry < 2)
                {
                    fprintf(stderr, "Retrying calibration for leg %d (attempt %d)...\n",
                            cal.leg_num, retry + 2);
                    usleep(100000); // 100ms delay between retries
                }
            }

            // Report failure for this leg
            if (!success)
            {
                fprintf(stderr, "Failed to apply calibration to leg %d: %s\n",
                        cal.leg_num, hexapod.getLastErrorMessage().c_str());
            }
        }

        // Report overall result
        fprintf(stderr, "%zu of %zu legs calibrated successfully\n",
                success_count, calibrations.size());

        // Success if all calibrations were applied
        return (success_count == calibrations.size());
    }

    bool CalibrationManager::applyCalibrationFromFile(hexapod::Hexapod &hexapod,
                                                      const std::string &filename)
    {
        std::vector<Calibration> calibrations;

        // Load calibration data
        if (!loadCalibration(calibrations, filename))
        {
            return false;
        }

        // Apply loaded calibration to the hexapod
        return applyCalibration(hexapod, calibrations);
    }

    //==============================================================================
    // Validation
    //==============================================================================

    bool CalibrationManager::validateCalibration(const std::vector<Calibration> &calibrations)
    {
        // Check if we have the right number of legs
        if (calibrations.size() != hexapod::Config::NUM_LEGS)
        {
            fprintf(stderr, "Invalid number of calibration entries: %zu (expected %d)\n",
                    calibrations.size(), hexapod::Config::NUM_LEGS);
            return false;
        }

        // Create a vector to track which legs we've seen
        std::vector<bool> legs_found(hexapod::Config::NUM_LEGS, false);

        // Validate each entry
        for (const auto &cal : calibrations)
        {
            // Check leg number is valid
            if (cal.leg_num >= hexapod::Config::NUM_LEGS)
            {
                fprintf(stderr, "Invalid leg number: %d\n", cal.leg_num);
                return false;
            }

            // Check if this leg has already been seen
            if (legs_found[cal.leg_num])
            {
                fprintf(stderr, "Duplicate calibration for leg %d\n", cal.leg_num);
                return false;
            }

            // Mark this leg as seen
            legs_found[cal.leg_num] = true;

            // Validate offset ranges
            if (std::abs(cal.hip_offset) > CalibrationManagerImpl::CalibrationLimits::MAX_HIP_OFFSET)
            {
                fprintf(stderr, "Hip offset for leg %d out of range: %d\n", cal.leg_num, cal.hip_offset);
                return false;
            }

            if (std::abs(cal.knee_offset) > CalibrationManagerImpl::CalibrationLimits::MAX_KNEE_OFFSET)
            {
                fprintf(stderr, "Knee offset for leg %d out of range: %d\n", cal.leg_num, cal.knee_offset);
                return false;
            }

            if (std::abs(cal.ankle_offset) > CalibrationManagerImpl::CalibrationLimits::MAX_ANKLE_OFFSET)
            {
                fprintf(stderr, "Ankle offset for leg %d out of range: %d\n", cal.leg_num, cal.ankle_offset);
                return false;
            }
        }

        // Check that all legs have calibration data
        for (int i = 0; i < hexapod::Config::NUM_LEGS; i++)
        {
            if (!legs_found[i])
            {
                fprintf(stderr, "Missing calibration for leg %d\n", i);
                return false;
            }
        }

        // All validation checks passed
        return true;
    }

} // namespace calibration
