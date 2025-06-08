#include <cmath>
#include <QDebug>
#include <QPainterPath>
#include "hexapodvisualization.h"

HexapodVisualization::HexapodVisualization(QWidget *parent)
    : QWidget(parent), m_pitch(0.0), m_roll(0.0), m_center(0, 0), m_scale(1.0),
      m_darkMode(false), m_showCoordinateSystem(true), m_highlightActiveLeg(true)
{
    // Set minimum size for the widget
    setMinimumSize(300, 300);

    // Initialize leg positions
    resetAllLegs();

    // Enable mouse tracking to capture mouse movement for possible interactive controls
    setMouseTracking(true);

    // Initialize color schemes
    m_lightColors = {
        QColor(240, 240, 240), // background
        QColor(220, 220, 240), // bodyFill
        QColor(100, 100, 120), // bodyOutline
        QColor(80, 80, 100),   // legNormal
        QColor(40, 120, 200),  // legActive
        QColor(200, 200, 200), // gridLines
        QColor(200, 80, 80),   // axisX
        QColor(80, 180, 80),   // axisY
        QColor(80, 80, 200),   // axisZ
        QColor(40, 40, 40)     // text
    };

    m_darkColors = {
        QColor(40, 40, 50),    // background
        QColor(60, 60, 80),    // bodyFill
        QColor(140, 140, 160), // bodyOutline
        QColor(180, 180, 200), // legNormal
        QColor(100, 180, 240), // legActive
        QColor(70, 70, 80),    // gridLines
        QColor(240, 100, 100), // axisX
        QColor(100, 240, 100), // axisY
        QColor(100, 100, 240), // axisZ
        QColor(220, 220, 220)  // text
    };
}

void HexapodVisualization::setColorScheme(bool darkMode)
{
    m_darkMode = darkMode;
    update();
}

void HexapodVisualization::setShowCoordinateSystem(bool show)
{
    m_showCoordinateSystem = show;
    update();
}

void HexapodVisualization::setHighlightActiveLeg(bool highlight)
{
    m_highlightActiveLeg = highlight;
    update();
}

const HexapodVisualization::ColorScheme &HexapodVisualization::currentColorScheme() const
{
    return m_darkMode ? m_darkColors : m_lightColors;
}

void HexapodVisualization::setTilt(double pitchDegrees, double rollDegrees)
{
    m_pitch = pitchDegrees;
    m_roll = rollDegrees;
    update();
}

void HexapodVisualization::setLegPosition(uint8_t legIndex, int16_t hip, int16_t knee, int16_t ankle)
{
    if (legIndex < NUM_LEGS)
    {
        // Mark leg as active when its position changes significantly
        bool significantChange =
            std::abs(m_legStates[legIndex].hip - hip) > 5 ||
            std::abs(m_legStates[legIndex].knee - knee) > 5 ||
            std::abs(m_legStates[legIndex].ankle - ankle) > 5;

        m_legStates[legIndex].hip = hip;
        m_legStates[legIndex].knee = knee;
        m_legStates[legIndex].ankle = ankle;

        if (significantChange)
        {
            m_legStates[legIndex].active = true;
            // Start a timer to reset active status after a delay
            QTimer::singleShot(500, this, [this, legIndex]()
                               {
                m_legStates[legIndex].active = false;
                update(); });
        }

        calculateLegPositions();
        update();
    }
}

void HexapodVisualization::resetAllLegs()
{
    for (int i = 0; i < NUM_LEGS; ++i)
    {
        m_legStates[i].hip = 0;
        m_legStates[i].knee = 0;
        m_legStates[i].ankle = 0;
        m_legStates[i].active = false;
        // Position will be calculated in calculateLegPositions
    }
    calculateLegPositions();
    update();
}

void HexapodVisualization::updateFromImuData(double accelX, double accelY, double accelZ, double gyroX, double gyroY, double gyroZ)
{
    // Mark unused parameters to avoid warnings
    (void)gyroX;
    (void)gyroY;
    (void)gyroZ;

    // Calculate approximate pitch and roll from accelerometer data
    // This is a simple approximation, a proper implementation would use a complementary or Kalman filter

    // Convert accelerations to angles (in degrees)
    if (accelZ != 0.0)
    {
        double pitchRad = std::atan2(accelX, std::sqrt(accelY * accelY + accelZ * accelZ));
        double rollRad = std::atan2(accelY, std::sqrt(accelX * accelX + accelZ * accelZ));

        // Convert to degrees and invert as needed to match visualization
        double pitchDeg = pitchRad * 180.0 / M_PI;
        double rollDeg = rollRad * 180.0 / M_PI;

        setTilt(pitchDeg, rollDeg);
    }
}

void HexapodVisualization::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Fill background
    const ColorScheme &colors = currentColorScheme();
    painter.fillRect(rect(), colors.background);

    // Set up coordinate system centered on the widget with y-axis pointing up
    painter.translate(m_center);
    painter.scale(m_scale, -m_scale); // Invert y-axis to match standard coordinate system

    // Draw reference grid
    const int gridSize = 50; // Grid cell size
    const int numCells = 10; // Number of cells in each direction

    painter.setPen(QPen(colors.gridLines, 0.5 / m_scale));
    for (int i = -numCells; i <= numCells; i++)
    {
        // Horizontal lines
        painter.drawLine(-numCells * gridSize, i * gridSize, numCells * gridSize, i * gridSize);
        // Vertical lines
        painter.drawLine(i * gridSize, -numCells * gridSize, i * gridSize, numCells * gridSize);
    }

    // Draw coordinate system if enabled
    if (m_showCoordinateSystem)
    {
        drawCoordinateSystem(painter);
    }

    drawHexapod(painter);

    // Reset transform for HUD elements
    painter.resetTransform();

    // Draw tilt indicators
    painter.setPen(colors.text);
    painter.drawText(10, 20, QString("Pitch: %1°").arg(m_pitch, 0, 'f', 1));
    painter.drawText(10, 40, QString("Roll: %1°").arg(m_roll, 0, 'f', 1));
}

void HexapodVisualization::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);

    // Center point in widget coordinates
    m_center = QPointF(width() / 2.0, height() / 2.0);

    // Calculate scaling factor based on widget size
    m_scale = qMin(width(), height()) / 500.0;

    // Recalculate leg positions with new scale
    calculateLegPositions();
}

void HexapodVisualization::drawHexapod(QPainter &painter)
{
    // Apply tilt transformation
    painter.save();

    // 3D rotation simulation with 2D transforms - simplified approximation
    // First roll around Y axis (X-Z plane)
    double rollRad = m_roll * M_PI / 180.0;
    painter.scale(cos(rollRad), 1.0);

    // Then pitch around X axis (Y-Z plane)
    double pitchRad = m_pitch * M_PI / 180.0;
    painter.scale(1.0, cos(pitchRad));

    drawBody(painter);

    // Draw each leg
    for (int i = 0; i < NUM_LEGS; ++i)
    {
        drawLeg(painter, i);
    }

    painter.restore();
}

void HexapodVisualization::drawLeg(QPainter &painter, int legIndex)
{
    const ColorScheme &colors = currentColorScheme();
    const LegState &leg = m_legStates[legIndex];

    // Choose color based on leg state
    if (m_highlightActiveLeg && leg.active)
    {
        painter.setPen(QPen(colors.legActive, 2.0 / m_scale));
        painter.setBrush(colors.legActive.lighter(120));
    }
    else
    {
        painter.setPen(QPen(colors.legNormal, 2.0 / m_scale));
        painter.setBrush(Qt::NoBrush);
    }

    // Draw leg joint connection to body
    QPointF bodyPoint;
    switch (legIndex)
    {
    case 0:
        bodyPoint = QPointF(60, 60);
        break; // Front right
    case 1:
        bodyPoint = QPointF(0, 70);
        break; // Middle right
    case 2:
        bodyPoint = QPointF(-60, 60);
        break; // Back right
    case 3:
        bodyPoint = QPointF(60, -60);
        break; // Front left
    case 4:
        bodyPoint = QPointF(0, -70);
        break; // Middle left
    case 5:
        bodyPoint = QPointF(-60, -60);
        break; // Back left
    }

    // Draw line from body to leg position
    painter.drawLine(bodyPoint, leg.computedPosition);

    // Draw joint
    painter.setBrush(colors.legNormal);
    painter.drawEllipse(bodyPoint, 5 / m_scale, 5 / m_scale);

    // Draw foot endpoint
    painter.setBrush(m_highlightActiveLeg && leg.active ? colors.legActive : colors.legNormal);
    painter.drawEllipse(leg.computedPosition, 7 / m_scale, 7 / m_scale);
}

void HexapodVisualization::drawBody(QPainter &painter)
{
    const ColorScheme &colors = currentColorScheme();

    // Create hexagon path for body
    QPainterPath bodyPath;
    bodyPath.moveTo(60, 60);   // Front right
    bodyPath.lineTo(0, 70);    // Middle right
    bodyPath.lineTo(-60, 60);  // Back right
    bodyPath.lineTo(-60, -60); // Back left
    bodyPath.lineTo(0, -70);   // Middle left
    bodyPath.lineTo(60, -60);  // Front left
    bodyPath.closeSubpath();

    // Draw body
    painter.setPen(QPen(colors.bodyOutline, 2.0 / m_scale));
    painter.setBrush(colors.bodyFill);
    painter.drawPath(bodyPath);

    // Draw direction indicator (front)
    painter.setPen(QPen(colors.axisX, 2.0 / m_scale));
    painter.setBrush(colors.axisX);
    QPointF frontPoints[3] = {
        QPointF(70, 0),
        QPointF(60, 10),
        QPointF(60, -10)};
    painter.drawPolygon(frontPoints, 3);
}

void HexapodVisualization::drawCoordinateSystem(QPainter &painter)
{
    const ColorScheme &colors = currentColorScheme();

    // X axis (red)
    painter.setPen(QPen(colors.axisX, 2.0 / m_scale));
    painter.drawLine(0, 0, 50, 0);
    painter.drawText(52, 0, "X");

    // Y axis (green)
    painter.setPen(QPen(colors.axisY, 2.0 / m_scale));
    painter.drawLine(0, 0, 0, 50);
    painter.drawText(0, 52, "Y");

    // Z axis - drawn as a circle since we're in 2D (blue)
    painter.setPen(QPen(colors.axisZ, 2.0 / m_scale));
    painter.drawEllipse(-5, -5, 10, 10);
    painter.drawText(-5, -10, "Z");
}

void HexapodVisualization::calculateLegPositions()
{
    // In a real implementation, this would use forward kinematics
    // Here we're just using simple approximation for visualization

    // Define the base positions
    const QPointF legBasePositions[NUM_LEGS] = {
        QPointF(100, 100),  // Front right
        QPointF(0, 120),    // Middle right
        QPointF(-100, 100), // Back right
        QPointF(100, -100), // Front left
        QPointF(0, -120),   // Middle left
        QPointF(-100, -100) // Back left
    };

    // Calculate each leg's position based on joint angles
    for (int i = 0; i < NUM_LEGS; ++i)
    {
        // Convert angles to radians
        double hipRad = m_legStates[i].hip * M_PI / 180.0;
        double kneeRad = m_legStates[i].knee * M_PI / 180.0;
        double ankleRad = m_legStates[i].ankle * M_PI / 180.0;

        // Simple approximation - in reality would need proper forward kinematics
        double extend = 30.0 + cos(kneeRad) * 20.0 + cos(ankleRad) * 15.0;
        double heightAdjust = sin(kneeRad) * 20.0 + sin(ankleRad) * 15.0;

        // Start from base position and adjust based on calculated extension
        QPointF base = legBasePositions[i];

        // Get unit vector from body to base position
        double length = sqrt(base.x() * base.x() + base.y() * base.y());
        if (length > 0.0001)
        {
            double unitX = base.x() / length;
            double unitY = base.y() / length;

            // Apply hip rotation (around Z axis)
            double rotX = unitX * cos(hipRad) - unitY * sin(hipRad);
            double rotY = unitX * sin(hipRad) + unitY * cos(hipRad);

            // Calculate final position
            double finalX = base.x() + rotX * extend;
            double finalY = base.y() + rotY * extend;

            // Store computed position
            m_legStates[i].computedPosition = QPointF(finalX, finalY - heightAdjust);
        }
        else
        {
            m_legStates[i].computedPosition = base;
        }
    }
}
