/*
 * Hexapod Project - A Reinforcement Learning-based Autonomous Hexapod
 * Copyright (C) 2025  Nguyen Trong Phuc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <cstdio>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <system_error>
#include <unistd.h>
#include <iostream>
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
            std::cerr << "Failed to open calibration file: " << path << std::endl;
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
                std::cerr << "Invalid calibration file format" << std::endl;
                return false;
            }

            // Check version compatibility
            if (header.version != CalibrationManagerImpl::FILE_VERSION)
            {
                std::cerr << "Unsupported calibration file version: " << header.version << std::endl;
                return false;
            }

            // Validate entry count
            if (header.num_entries > 100)
            { // Sanity check to prevent huge allocations
                std::cerr << "Invalid number of calibration entries: " << header.num_entries << std::endl;
                return false;
            }

            // Read calibration entries
            calibrations.resize(header.num_entries);
            file.read(reinterpret_cast<char *>(calibrations.data()),
                      sizeof(Calibration) * header.num_entries);

            // Verify we read the expected number of legs
            if (calibrations.size() != hexapod::Config::NUM_LEGS)
            {
                std::cerr << "Warning: Expected " << hexapod::Config::NUM_LEGS
                          << " legs, loaded " << calibrations.size() << std::endl;
                // Continue anyway - might be a different configuration
            }

            return !file.fail();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Exception while loading calibration: " << e.what() << std::endl;
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
                std::cerr << "Failed to create directory: " << e.what() << std::endl;
                return false;
            }
        }

        // Try to open the file
        std::ofstream file(path, std::ios::binary | std::ios::trunc);
        if (!file)
        {
            std::cerr << "Failed to open calibration file for writing: " << path << std::endl;
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
            std::cerr << "Exception while saving calibration: " << e.what() << std::endl;
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
            std::cerr << "No calibration data to apply" << std::endl;
            return false;
        }

        // Before applying, validate the calibrations
        if (!validateCalibration(calibrations))
        {
            std::cerr << "Calibration validation failed" << std::endl;
            return false;
        }

        // Apply calibration to each leg
        size_t success_count = 0;
        for (const auto &cal : calibrations)
        {
            // Skip entries with invalid leg numbers
            if (cal.leg_num >= hexapod::Config::NUM_LEGS)
            {
                std::cerr << "Invalid leg number in calibration: " << cal.leg_num << std::endl;
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
                    std::cerr << "Retrying calibration for leg " << cal.leg_num
                              << " (attempt " << (retry + 2) << ")..." << std::endl;
                    usleep(100000); // 100ms delay between retries
                }
            }

            // Report failure for this leg
            if (!success)
            {
                std::cerr << "Failed to apply calibration to leg " << cal.leg_num
                          << ": " << hexapod.getLastErrorMessage() << std::endl;
            }
        }

        // Report overall result
        std::cerr << success_count << " of " << calibrations.size()
                  << " legs calibrated successfully" << std::endl;

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
            std::cerr << "Invalid number of calibration entries: " << calibrations.size()
                      << " (expected " << hexapod::Config::NUM_LEGS << ")" << std::endl;
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
                std::cerr << "Invalid leg number: " << cal.leg_num << std::endl;
                return false;
            }

            // Check if this leg has already been seen
            if (legs_found[cal.leg_num])
            {
                std::cerr << "Duplicate calibration for leg " << cal.leg_num << std::endl;
                return false;
            }

            // Mark this leg as seen
            legs_found[cal.leg_num] = true;

            // Validate offset ranges
            if (std::abs(cal.hip_offset) > CalibrationManagerImpl::CalibrationLimits::MAX_HIP_OFFSET)
            {
                std::cerr << "Hip offset for leg " << cal.leg_num
                          << " out of range: " << cal.hip_offset << std::endl;
                return false;
            }

            if (std::abs(cal.knee_offset) > CalibrationManagerImpl::CalibrationLimits::MAX_KNEE_OFFSET)
            {
                std::cerr << "Knee offset for leg " << cal.leg_num
                          << " out of range: " << cal.knee_offset << std::endl;
                return false;
            }

            if (std::abs(cal.ankle_offset) > CalibrationManagerImpl::CalibrationLimits::MAX_ANKLE_OFFSET)
            {
                std::cerr << "Ankle offset for leg " << cal.leg_num
                          << " out of range: " << cal.ankle_offset << std::endl;
                return false;
            }
        }

        // Check that all legs have calibration data
        for (int i = 0; i < hexapod::Config::NUM_LEGS; i++)
        {
            if (!legs_found[i])
            {
                std::cerr << "Missing calibration for leg " << i << std::endl;
                return false;
            }
        }

        // All validation checks passed
        return true;
    }

} // namespace calibration
