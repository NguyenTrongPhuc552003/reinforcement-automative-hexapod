#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "calibration.hpp"

// Generate default calibration values for all legs
std::vector<Calibration> CalibrationManager::getDefaultCalibration()
{
    std::vector<Calibration> calibrations;
    calibrations.resize(NUM_LEGS);

    // Initialize default calibration (no offsets)
    for (uint8_t i = 0; i < NUM_LEGS; i++)
    {
        calibrations[i].leg_num = i;
        calibrations[i].hip_offset = 0;
        calibrations[i].knee_offset = 0;
        calibrations[i].ankle_offset = 0;
    }

    return calibrations;
}

bool CalibrationManager::saveCalibration(const std::vector<Calibration> &calibrations)
{
    // Create directories if needed
    size_t pos = std::string(CALIBRATION_FILE).find_last_of("/\\");
    if (pos != std::string::npos)
    {
        std::string dir = std::string(CALIBRATION_FILE).substr(0, pos);
        // Ensure directory exists
        mkdir(dir.c_str(), 0755); // No error check - it's okay if it already exists
    }

    FILE *file = fopen(CALIBRATION_FILE, "wb");
    if (!file)
    {
        fprintf(stderr, "Failed to open calibration file for writing: %s\n", CALIBRATION_FILE);
        return false;
    }

    // Write number of calibrations
    size_t count = calibrations.size();
    fwrite(&count, sizeof(count), 1, file);

    // Write each calibration
    fwrite(calibrations.data(), sizeof(Calibration), count, file);

    fclose(file);
    return true;
}

bool CalibrationManager::loadCalibration(std::vector<Calibration> &calibrations)
{
    FILE *file = fopen(CALIBRATION_FILE, "rb");
    if (!file)
    {
        fprintf(stderr, "No calibration file found at %s, creating default\n", CALIBRATION_FILE);
        // Create default calibration values
        calibrations = getDefaultCalibration();
        // Write the defaults to file for next time
        saveCalibration(calibrations);
        return true;
    }

    // Read number of calibrations
    size_t count = 0;
    if (fread(&count, sizeof(count), 1, file) != 1)
    {
        fclose(file);
        fprintf(stderr, "Error reading calibration count, using defaults\n");
        calibrations = getDefaultCalibration();
        return false;
    }

    // Check for reasonable count
    if (count > MAX_LEGS)
    {
        fclose(file);
        fprintf(stderr, "Invalid calibration count %zu, using defaults\n", count);
        calibrations = getDefaultCalibration();
        return false;
    }

    // Read calibrations
    calibrations.resize(count);
    if (fread(calibrations.data(), sizeof(Calibration), count, file) != count)
    {
        fclose(file);
        fprintf(stderr, "Error reading calibration data, using defaults\n");
        calibrations = getDefaultCalibration();
        return false;
    }

    fclose(file);
    return true;
}

// Create calibration file with defaults if it doesn't exist
bool CalibrationManager::initializeDefaultCalibration()
{
    // Check if file already exists
    FILE *file = fopen(CALIBRATION_FILE, "rb");
    if (file)
    {
        fclose(file);
        return true; // File exists, nothing to do
    }

    // Create default calibration
    std::vector<Calibration> defaults = getDefaultCalibration();
    return saveCalibration(defaults);
}
