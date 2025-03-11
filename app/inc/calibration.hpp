#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <vector>
#include <string>
#include "hexapod.hpp"

#define CALIBRATION_FILE "calibration.cfg"
#define MAX_LEGS 8 // More than we need but provides safety margin

class CalibrationManager
{
public:
    static std::vector<Calibration> getDefaultCalibration();
    static bool saveCalibration(const std::vector<Calibration> &calibrations);
    static bool loadCalibration(std::vector<Calibration> &calibrations);
    static bool initializeDefaultCalibration();
};

#endif /* CALIBRATION_HPP */
