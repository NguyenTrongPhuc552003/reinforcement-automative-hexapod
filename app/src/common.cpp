#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <cstring>
#include "common.hpp"

namespace common
{

    //==============================================================================
    // Time Utilities Implementation
    //==============================================================================

    double getCurrentTime()
    {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration<double>(duration).count();
    }

    unsigned long getCurrentTimeMs()
    {
        return static_cast<unsigned long>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }

    void sleepMs(unsigned int milliseconds)
    {
        usleep(milliseconds * 1000);
    }

    void sleepUs(unsigned int microseconds)
    {
        usleep(microseconds);
    }

    //==============================================================================
    // Terminal Manager Implementation
    //==============================================================================

    struct termios TerminalManager::s_originalTermios = {};
    bool TerminalManager::s_isModified = false;

    bool TerminalManager::setupNonBlocking()
    {
        // Save original terminal settings
        if (tcgetattr(STDIN_FILENO, &s_originalTermios) < 0)
        {
            ErrorReporter::reportError("TerminalManager", "setupNonBlocking",
                                       "Failed to get terminal attributes");
            return false;
        }

        // Configure for non-blocking input
        struct termios newTermios = s_originalTermios;
        newTermios.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | IEXTEN);
        newTermios.c_iflag &= ~(ISTRIP | INLCR | ICRNL | IGNCR | IXON | IXOFF);
        newTermios.c_cflag &= ~CSIZE;
        newTermios.c_cflag |= CS8;
        newTermios.c_cc[VMIN] = 0;
        newTermios.c_cc[VTIME] = 0;

        if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &newTermios) < 0)
        {
            ErrorReporter::reportError("TerminalManager", "setupNonBlocking",
                                       "Failed to set terminal attributes");
            return false;
        }

        // Set stdin to non-blocking mode
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags != -1)
        {
            fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        }

        s_isModified = true;
        return true;
    }

    bool TerminalManager::setupImmediate()
    {
        // Save original terminal settings
        if (tcgetattr(STDIN_FILENO, &s_originalTermios) < 0)
        {
            return false;
        }

        // Configure for immediate input
        struct termios newTermios = s_originalTermios;
        newTermios.c_lflag &= ~(ICANON | ECHO);

        if (tcsetattr(STDIN_FILENO, TCSANOW, &newTermios) < 0)
        {
            return false;
        }

        s_isModified = true;
        return true;
    }

    void TerminalManager::restore()
    {
        if (s_isModified)
        {
            tcsetattr(STDIN_FILENO, TCSAFLUSH, &s_originalTermios);
            s_isModified = false;
        }
    }

    bool TerminalManager::isModified()
    {
        return s_isModified;
    }

    bool TerminalManager::readChar(char &ch)
    {
        return read(STDIN_FILENO, &ch, 1) > 0;
    }

    //==============================================================================
    // Signal Manager Implementation
    //==============================================================================

    std::atomic<bool> *SignalManager::s_runningFlag = nullptr;
    SignalHandler SignalManager::s_customHandler = nullptr;

    void SignalManager::setupGracefulShutdown(std::atomic<bool> &running, SignalHandler handler)
    {
        s_runningFlag = &running;
        s_customHandler = handler;

        std::signal(SIGINT, [](int sig)
                    {
            if (s_runningFlag) {
                s_runningFlag->store(false);
            }
            if (s_customHandler) {
                s_customHandler(sig);
            } else {
                defaultHandler(sig);
            } });

        std::signal(SIGTERM, [](int sig)
                    {
            if (s_runningFlag) {
                s_runningFlag->store(false);
            }
            if (s_customHandler) {
                s_customHandler(sig);
            } else {
                defaultHandler(sig);
            } });
    }

    void SignalManager::defaultHandler(int signal)
    {
        std::cout << "\nReceived signal " << signal << ", shutting down gracefully..." << std::endl;
    }

    //==============================================================================
    // Performance Monitor Implementation
    //==============================================================================

    PerformanceMonitor::PerformanceMonitor()
        : m_frameCount(0), m_totalFrameTime(0.0), m_maxFrameTime(0.0), m_frameInProgress(false)
    {
    }

    void PerformanceMonitor::startFrame()
    {
        m_frameStart = std::chrono::high_resolution_clock::now();
        m_frameInProgress = true;
    }

    void PerformanceMonitor::endFrame()
    {
        if (!m_frameInProgress)
            return;

        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - m_frameStart);
        double frameTimeMs = duration.count() / 1000.0;

        recordFrame(frameTimeMs);
        m_frameInProgress = false;
    }

    void PerformanceMonitor::recordFrame(double durationMs)
    {
        m_frameCount++;
        m_totalFrameTime += durationMs;
        m_maxFrameTime = std::max(m_maxFrameTime, durationMs);
    }

    double PerformanceMonitor::getAverageFrameTime() const
    {
        return (m_frameCount > 0) ? (m_totalFrameTime / m_frameCount) : 0.0;
    }

    double PerformanceMonitor::getMaxFrameTime() const
    {
        return m_maxFrameTime;
    }

    double PerformanceMonitor::getFPS() const
    {
        double avgFrameTime = getAverageFrameTime();
        return (avgFrameTime > 0.0) ? (1000.0 / avgFrameTime) : 0.0;
    }

    unsigned long PerformanceMonitor::getFrameCount() const
    {
        return m_frameCount;
    }

    void PerformanceMonitor::reset()
    {
        m_frameCount = 0;
        m_totalFrameTime = 0.0;
        m_maxFrameTime = 0.0;
        m_frameInProgress = false;
    }

    void PerformanceMonitor::printReport(const std::string &prefix) const
    {
        std::cout << prefix << "Performance: "
                  << "Frames: " << m_frameCount
                  << ", Avg: " << StringUtils::formatNumber(getAverageFrameTime()) << "ms"
                  << ", Max: " << StringUtils::formatNumber(m_maxFrameTime) << "ms"
                  << ", FPS: " << StringUtils::formatNumber(getFPS(), 1)
                  << std::endl;
    }

    //==============================================================================
    // Progress Bar Implementation
    //==============================================================================

    void ProgressBar::display(int current, int total, int width, const std::string &label)
    {
        double percentage = (total > 0) ? static_cast<double>(current) / total : 0.0;
        displayPercent(percentage, width, label);
    }

    void ProgressBar::displayPercent(double percentage, int width, const std::string &label)
    {
        percentage = Validator::clamp(percentage, 0.0, 1.0);
        int pos = static_cast<int>(width * percentage);

        std::cout << "\r";
        if (!label.empty())
        {
            std::cout << label << " ";
        }

        std::cout << "[";
        for (int i = 0; i < width; ++i)
        {
            if (i < pos)
            {
                std::cout << "=";
            }
            else if (i == pos)
            {
                std::cout << ">";
            }
            else
            {
                std::cout << " ";
            }
        }
        std::cout << "] " << std::setw(3) << static_cast<int>(percentage * 100) << "%";
        std::cout << std::flush;
    }

    void ProgressBar::clear()
    {
        std::cout << "\r" << std::string(80, ' ') << "\r" << std::flush;
    }

    //==============================================================================
    // Error Reporter Implementation
    //==============================================================================

    void ErrorReporter::reportError(const std::string &component,
                                    const std::string &operation,
                                    const std::string &details)
    {
        std::cerr << "[ERROR] " << component << "::" << operation << " - " << details << std::endl;
    }

    void ErrorReporter::reportWarning(const std::string &component,
                                      const std::string &message)
    {
        std::cerr << "[WARNING] " << component << " - " << message << std::endl;
    }

    void ErrorReporter::reportInfo(const std::string &component,
                                   const std::string &message)
    {
        std::cout << "[INFO] " << component << " - " << message << std::endl;
    }

    //==============================================================================
    // Validator Implementation
    //==============================================================================

    bool Validator::validateAngle(double angle, double min, double max)
    {
        return (angle >= min && angle <= max);
    }

    bool Validator::validateLegNumber(int legNum, int maxLegs)
    {
        return (legNum >= 0 && legNum < maxLegs);
    }

    bool Validator::validateSpeed(double speed)
    {
        return (speed >= Constants::MIN_SPEED && speed <= Constants::MAX_SPEED);
    }

    bool Validator::validateDirection(double direction)
    {
        return (direction >= Constants::MIN_DIRECTION && direction <= Constants::MAX_DIRECTION);
    }

    //==============================================================================
    // String Utils Implementation
    //==============================================================================

    std::string StringUtils::formatNumber(double value, int precision)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << value;
        return oss.str();
    }

    std::string StringUtils::formatDuration(double seconds)
    {
        int minutes = static_cast<int>(seconds / 60);
        double remainingSeconds = seconds - (minutes * 60);

        std::ostringstream oss;
        if (minutes > 0)
        {
            oss << minutes << "m ";
        }
        oss << std::fixed << std::setprecision(1) << remainingSeconds << "s";
        return oss.str();
    }

    std::string StringUtils::padString(const std::string &text, int width, char fillChar)
    {
        if (static_cast<int>(text.length()) >= width)
        {
            return text;
        }
        return text + std::string(width - text.length(), fillChar);
    }

    std::string StringUtils::toLower(const std::string &str)
    {
        std::string result = str;
        std::transform(result.begin(), result.end(), result.begin(), ::tolower);
        return result;
    }

    std::string StringUtils::toUpper(const std::string &str)
    {
        std::string result = str;
        std::transform(result.begin(), result.end(), result.begin(), ::toupper);
        return result;
    }

    //==============================================================================
    // Math Utils Implementation
    //==============================================================================

    double MathUtils::degToRad(double degrees)
    {
        return degrees * Constants::DEG_TO_RAD;
    }

    double MathUtils::radToDeg(double radians)
    {
        return radians * Constants::RAD_TO_DEG;
    }

    double MathUtils::lerp(double a, double b, double t)
    {
        return a + (b - a) * Validator::clamp(t, 0.0, 1.0);
    }

    double MathUtils::normalizeAngle(double angle)
    {
        while (angle < 0.0)
            angle += 360.0;
        while (angle >= 360.0)
            angle -= 360.0;
        return angle;
    }

    std::vector<double> MathUtils::movingAverage(const std::vector<double> &values, int windowSize)
    {
        std::vector<double> result;
        if (values.empty() || windowSize <= 0)
        {
            return result;
        }

        result.reserve(values.size());
        for (size_t i = 0; i < values.size(); ++i)
        {
            int start = std::max(0, static_cast<int>(i) - windowSize + 1);
            int end = static_cast<int>(i) + 1;

            double sum = 0.0;
            for (int j = start; j < end; ++j)
            {
                sum += values[j];
            }
            result.push_back(sum / (end - start));
        }
        return result;
    }

} // namespace common
