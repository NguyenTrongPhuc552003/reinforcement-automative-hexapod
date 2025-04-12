#include <QDateTime>
#include <QJsonObject>
#include <QDebug>
#include "hexapodcontext.h"
#include "hexapodconnection.h"
#include "hexapodvisualization.h"

HexapodContext::HexapodContext(QObject *parent)
    : QObject(parent), m_connection(nullptr), m_visualization(nullptr),
      m_imuUpdateTimer(new QTimer(this)), m_statusMessage("Disconnected"), 
      m_speed(0.5), m_pitch(0.0), m_roll(0.0), 
      m_balanceEnabled(false), m_connected(false)
{
    // Initialize IMU update timer
    m_imuUpdateTimer->setInterval(100); // 10 updates per second
    connect(m_imuUpdateTimer, &QTimer::timeout, this, &HexapodContext::requestImuUpdate);
}

HexapodContext::~HexapodContext()
{
    // The context doesn't own these objects, just references them
}

void HexapodContext::setConnection(HexapodConnection *connection)
{
    m_connection = connection;
    
    // Connect signals from the connection to our handler methods
    if (m_connection) {
        connect(m_connection, &HexapodConnection::connectionStatusChanged,
                this, &HexapodContext::handleConnectionStatus);
        connect(m_connection, &HexapodConnection::imuDataReceived,
                this, &HexapodContext::handleImuDataReceived);
        connect(m_connection, &HexapodConnection::responseReceived,
                this, &HexapodContext::handleResponseReceived);
        connect(m_connection, &HexapodConnection::errorOccurred,
                this, &HexapodContext::handleErrorOccurred);
    }
}

HexapodConnection *HexapodContext::connection() const
{
    return m_connection;
}

void HexapodContext::setVisualization(HexapodVisualization *visualization)
{
    m_visualization = visualization;
}

HexapodVisualization *HexapodContext::visualization() const
{
    return m_visualization;
}

bool HexapodContext::isConnected() const
{
    return m_connection->isConnected();
}

QString HexapodContext::statusMessage() const
{
    return m_statusMessage;
}

double HexapodContext::speed() const
{
    return m_speed;
}

double HexapodContext::pitch() const
{
    return m_pitch;
}

double HexapodContext::roll() const
{
    return m_roll;
}

bool HexapodContext::balanceEnabled() const
{
    return m_balanceEnabled;
}

QVariantMap HexapodContext::imuData() const
{
    return m_imuData;
}

void HexapodContext::setSpeed(double speed)
{
    if (m_speed != speed)
    {
        m_speed = speed;
        emit speedChanged(m_speed);

        if (m_connection->isConnected())
        {
            QJsonObject cmd = HexapodProtocol::createSetSpeedCommand(m_speed);
            m_connection->sendCommand(cmd);
        }
    }
}

void HexapodContext::setPitch(double pitch)
{
    if (m_pitch != pitch)
    {
        m_pitch = pitch;
        emit pitchChanged(m_pitch);

        if (m_connection->isConnected())
        {
            QJsonObject cmd = HexapodProtocol::createSetTiltCommand(m_pitch, m_roll);
            m_connection->sendCommand(cmd);
        }
    }
}

void HexapodContext::setRoll(double roll)
{
    if (m_roll != roll)
    {
        m_roll = roll;
        emit rollChanged(m_roll);

        if (m_connection->isConnected())
        {
            QJsonObject cmd = HexapodProtocol::createSetTiltCommand(m_pitch, m_roll);
            m_connection->sendCommand(cmd);
        }
    }
}

void HexapodContext::setBalanceEnabled(bool enabled)
{
    if (m_balanceEnabled != enabled)
    {
        m_balanceEnabled = enabled;
        emit balanceEnabledChanged(m_balanceEnabled);

        if (m_connection->isConnected())
        {
            QJsonObject cmd = HexapodProtocol::createSetBalanceEnabledCommand(m_balanceEnabled);
            m_connection->sendCommand(cmd);
        }
    }
}

bool HexapodContext::connectToHost(const QString &hostname, int port)
{
    logMessage(QString("Connecting to %1:%2...").arg(hostname).arg(port));

    if (m_connection->connectToHost(hostname, port))
    {
        m_imuUpdateTimer->start();
        return true;
    }
    return false;
}

void HexapodContext::disconnect()
{
    if (isConnected())
    {
        stop();
        m_imuUpdateTimer->stop();
        m_connection->disconnect();
        logMessage("Disconnected from server");
    }
}

void HexapodContext::moveForward(double speed)
{
    if (!isConnected())
        return;

    setSpeed(speed);
    QJsonObject cmd = HexapodProtocol::createSetMovementCommand(0.0, speed); // 0 degrees = forward
    m_connection->sendCommand(cmd);
    logMessage(QString("Moving forward at %1% speed").arg(speed * 100));
}

void HexapodContext::moveBackward(double speed)
{
    if (!isConnected())
        return;

    setSpeed(speed);
    QJsonObject cmd = HexapodProtocol::createSetMovementCommand(180.0, speed); // 180 degrees = backward
    m_connection->sendCommand(cmd);
    logMessage(QString("Moving backward at %1% speed").arg(speed * 100));
}

void HexapodContext::turnLeft(double speed)
{
    if (!isConnected())
        return;

    setSpeed(speed);
    QJsonObject cmd = HexapodProtocol::createSetRotationCommand(-1.0, speed);
    m_connection->sendCommand(cmd);
    logMessage(QString("Turning left at %1% speed").arg(speed * 100));
}

void HexapodContext::turnRight(double speed)
{
    if (!isConnected())
        return;

    setSpeed(speed);
    QJsonObject cmd = HexapodProtocol::createSetRotationCommand(1.0, speed);
    m_connection->sendCommand(cmd);
    logMessage(QString("Turning right at %1% speed").arg(speed * 100));
}

void HexapodContext::stop()
{
    if (!isConnected())
        return;

    QJsonObject cmd = HexapodProtocol::createStopCommand();
    m_connection->sendCommand(cmd);
    logMessage("Stopping movement");
}

void HexapodContext::centerAllLegs()
{
    if (!isConnected())
        return;

    m_connection->centerAllLegs();
    logMessage("Centering all legs");
}

void HexapodContext::logMessage(const QString &message)
{
    QDateTime now = QDateTime::currentDateTime();
    QString timestamp = now.toString("yyyy-MM-dd hh:mm:ss");

    QString logMessage = QString("[%1] %2").arg(timestamp).arg(message);
    emit messageLogged(logMessage);

    qDebug() << logMessage;
}

void HexapodContext::handleConnectionStatus(bool connected)
{
    m_connected = connected;
    
    if (connected)
    {
        m_statusMessage = "Connected";
        logMessage("Connected to server");
        m_imuUpdateTimer->start();
    }
    else
    {
        m_statusMessage = "Disconnected";
        logMessage("Lost connection to server");
        m_imuUpdateTimer->stop();
    }

    emit connectionStatusChanged(connected);
    emit statusMessageChanged(m_statusMessage);
}

void HexapodContext::handleImuDataReceived(double accelX, double accelY, double accelZ,
                                           double gyroX, double gyroY, double gyroZ)
{
    // Update IMU data map
    m_imuData["accelX"] = accelX;
    m_imuData["accelY"] = accelY;
    m_imuData["accelZ"] = accelZ;
    m_imuData["gyroX"] = gyroX;
    m_imuData["gyroY"] = gyroY;
    m_imuData["gyroZ"] = gyroZ;

    emit imuDataChanged(m_imuData);
    
    // Re-emit the received signal for components that need raw values
    emit imuDataReceived(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
}

void HexapodContext::handleResponseReceived(const QJsonObject &response)
{
    // Process server responses as needed
    if (response.contains("status"))
    {
        QString status = response["status"].toString();
        if (status == "error" && response.contains("message"))
        {
            QString error = response["message"].toString();
            logMessage("Error: " + error);
        }
    }
    
    // Forward the response to interested components
    emit commandResponse(response);
}

void HexapodContext::handleErrorOccurred(const QString &error)
{
    m_statusMessage = "Error: " + error;
    emit statusMessageChanged(m_statusMessage);
    emit errorOccurred(error);  // Forward the error
    logMessage("Error: " + error);
}

void HexapodContext::requestImuUpdate()
{
    if (isConnected())
    {
        m_connection->requestImuData();
    }
}
