#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include "hexapod.hpp"

/**
 * @brief Hexapod calibration system
 *
 * This namespace contains classes and functions for managing
 * calibration data for the hexapod robot.
 */
namespace calibration
{

    //==============================================================================
    // Calibration Data Structure
    //==============================================================================

    /**
     * @brief Calibration data for a single leg
     *
     * Contains the joint angle offsets to compensate for mechanical variations
     */
    struct Calibration
    {
        /**
         * @brief Construct a new Calibration object with default values
         */
        Calibration() : leg_num(0),
                        hip_offset(0),
                        knee_offset(0),
                        ankle_offset(0) {}

        /**
         * @brief Construct a new Calibration object with specific values
         *
         * @param leg Leg number (0-5)
         * @param hip Hip joint offset in degrees
         * @param knee Knee joint offset in degrees
         * @param ankle Ankle joint offset in degrees
         */
        Calibration(uint8_t leg, int16_t hip, int16_t knee, int16_t ankle) : leg_num(leg),
                                                                             hip_offset(hip),
                                                                             knee_offset(knee),
                                                                             ankle_offset(ankle) {}

        uint8_t leg_num;      ///< Leg number (0-5)
        int16_t hip_offset;   ///< Hip joint calibration offset in degrees
        int16_t knee_offset;  ///< Knee joint calibration offset in degrees
        int16_t ankle_offset; ///< Ankle joint calibration offset in degrees
    };

    //==============================================================================
    // Forward declarations
    //==============================================================================

    // Implementation class (PIMPL idiom)
    class CalibrationManagerImpl;

    //==============================================================================
    // Calibration Manager Class
    //==============================================================================

    /**
     * @brief Manages calibration data for the hexapod robot
     *
     * Provides methods for loading, saving, and applying calibration data
     * to compensate for mechanical variations in the robot.
     */
    class CalibrationManager
    {
    public:
        /**
         * @brief Get the singleton instance of the CalibrationManager
         *
         * @return CalibrationManager& Reference to the singleton instance
         */
        static CalibrationManager &getInstance();

        // Non-copyable
        CalibrationManager(const CalibrationManager &) = delete;
        CalibrationManager &operator=(const CalibrationManager &) = delete;

        // Non-movable
        CalibrationManager(CalibrationManager &&) = delete;
        CalibrationManager &operator=(CalibrationManager &&) = delete;

        //--------------------------------------------------------------------------
        // File Operations
        //--------------------------------------------------------------------------

        /**
         * @brief Load calibration data from file
         *
         * @param[out] calibrations Vector to store loaded calibration data
         * @param filename Optional custom filename
         * @return bool True if load successful
         */
        static bool loadCalibration(std::vector<Calibration> &calibrations,
                                    const std::string &filename = "");

        /**
         * @brief Save calibration data to file
         *
         * @param calibrations Vector of calibration data to save
         * @param filename Optional custom filename
         * @return bool True if save successful
         */
        static bool saveCalibration(const std::vector<Calibration> &calibrations,
                                    const std::string &filename = "");

        //--------------------------------------------------------------------------
        // Calibration Operations
        //--------------------------------------------------------------------------

        /**
         * @brief Get default calibration values (all zeros)
         *
         * @return std::vector<Calibration> Vector of default calibrations for all legs
         */
        static std::vector<Calibration> getDefaultCalibration();

        /**
         * @brief Apply calibration to the hexapod
         *
         * @param hexapod Reference to hexapod controller
         * @param calibrations Vector of calibration data to apply
         * @return bool True if all calibrations were applied successfully
         */
        static bool applyCalibration(hexapod::Hexapod &hexapod,
                                     const std::vector<Calibration> &calibrations);

        /**
         * @brief Apply calibration from file to the hexapod
         *
         * @param hexapod Reference to hexapod controller
         * @param filename Optional custom filename
         * @return bool True if calibration was applied successfully
         */
        static bool applyCalibrationFromFile(hexapod::Hexapod &hexapod,
                                             const std::string &filename = "");

        //--------------------------------------------------------------------------
        // Utility Functions
        //--------------------------------------------------------------------------

        /**
         * @brief Validate calibration values are within acceptable ranges
         *
         * @param calibrations Vector of calibration data to validate
         * @return bool True if all calibrations are valid
         */
        static bool validateCalibration(const std::vector<Calibration> &calibrations);

        /**
         * @brief Get the default calibration file path
         *
         * @return std::string Path to default calibration file
         */
        static std::string getDefaultCalibrationPath();

    private:
        /**
         * @brief Private constructor for singleton pattern
         */
        CalibrationManager();

        /**
         * @brief Implementation pointer (PIMPL idiom)
         */
        std::unique_ptr<CalibrationManagerImpl> pImpl;
    };

    //==============================================================================
    // Calibration Exception Class
    //==============================================================================

    /**
     * @brief Exception class for calibration errors
     */
    class CalibrationException : public std::exception
    {
    public:
        /**
         * @brief Construct a new CalibrationException
         *
         * @param message Error message
         */
        explicit CalibrationException(const std::string &message);

        /**
         * @brief Get error message
         *
         * @return const char* Error message
         */
        const char *what() const noexcept override;

    private:
        std::string message; ///< Error message
    };

} // namespace calibration

// For backward compatibility, import types into global namespace
using calibration::Calibration;
using calibration::CalibrationException;
using calibration::CalibrationManager;

#endif /* CALIBRATION_HPP */
