#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "calibration.hpp"

bool CalibrationManager::saveCalibration(const std::vector<Calibration> &calibrations)
{
    FILE *file = fopen(CALIBRATION_FILE, "wb");
    if (!file)
    {
        fprintf(stderr, "Failed to open calibration file for writing\n");
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
        fprintf(stderr, "No calibration file found\n");
        return false;
    }

    // Read number of calibrations
    size_t count = 0;
    if (fread(&count, sizeof(count), 1, file) != 1)
    {
        fclose(file);
        return false;
    }

    // Check for reasonable count
    if (count > MAX_LEGS)
    {
        fclose(file);
        return false;
    }

    // Read calibrations
    calibrations.resize(count);
    if (fread(calibrations.data(), sizeof(Calibration), count, file) != count)
    {
        fclose(file);
        return false;
    }

    fclose(file);
    return true;
}
