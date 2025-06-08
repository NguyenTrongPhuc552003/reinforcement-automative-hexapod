#ifndef HEXAPODVISUALIZATION_H
#define HEXAPODVISUALIZATION_H

#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <array>
#include <vector>
#include "hexapodprotocol.h"

class HexapodVisualization : public QWidget
{
    Q_OBJECT

public:
    explicit HexapodVisualization(QWidget *parent = nullptr);

    void setTilt(double pitchDegrees, double rollDegrees);
    void setLegPosition(uint8_t legIndex, int16_t hip, int16_t knee, int16_t ankle);
    void resetAllLegs();
    void setColorScheme(bool darkMode);
    void setShowCoordinateSystem(bool show);
    void setHighlightActiveLeg(bool highlight);

public slots:
    void updateFromImuData(double accelX, double accelY, double accelZ, double gyroX, double gyroY, double gyroZ);

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    // Robot dimensions
    static constexpr int NUM_LEGS = 6;

    // Visualization state
    double m_pitch; // X-axis tilt (degrees)
    double m_roll;  // Y-axis tilt (degrees)

    struct LegState
    {
        int16_t hip;
        int16_t knee;
        int16_t ankle;
        QPointF computedPosition; // 2D screen position
        bool active;              // Whether leg is active/moving
    };

    std::array<LegState, NUM_LEGS> m_legStates;

    // Cached values for visualization
    QPointF m_center;
    double m_scale;
    bool m_darkMode;
    bool m_showCoordinateSystem;
    bool m_highlightActiveLeg;

    // Colors for different themes
    struct ColorScheme
    {
        QColor background;
        QColor bodyFill;
        QColor bodyOutline;
        QColor legNormal;
        QColor legActive;
        QColor gridLines;
        QColor axisX;
        QColor axisY;
        QColor axisZ;
        QColor text;
    };

    ColorScheme m_lightColors;
    ColorScheme m_darkColors;

    // Helper drawing methods
    void drawHexapod(QPainter &painter);
    void drawLeg(QPainter &painter, int legIndex);
    void drawBody(QPainter &painter);
    void drawCoordinateSystem(QPainter &painter);
    void calculateLegPositions();
    const ColorScheme &currentColorScheme() const;
};

#endif // HEXAPODVISUALIZATION_H
