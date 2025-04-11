#include <QHostAddress>
#include <QJsonArray>
#include "hexapodconnection.h"

HexapodConnection::HexapodConnection(QObject *parent)
    : QObject(parent), m_socket(new QTcpSocket(this)), m_port(0)
{
    connect(m_socket, &QTcpSocket::connected, this, &HexapodConnection::handleSocketConnected);
    connect(m_socket, &QTcpSocket::disconnected, this, &HexapodConnection::handleSocketDisconnected);
    connect(m_socket, &QTcpSocket::errorOccurred, this, &HexapodConnection::handleSocketError);
    connect(m_socket, &QTcpSocket::readyRead, this, &HexapodConnection::handleSocketReadyRead);

    connect(&m_connectionCheckTimer, &QTimer::timeout, this, &HexapodConnection::checkConnection);
    m_connectionCheckTimer.setInterval(5000); // Check every 5 seconds
}

HexapodConnection::~HexapodConnection()
{
    disconnect();
}

bool HexapodConnection::connectToHost(const QString &hostname, int port)
{
    if (isConnected())
    {
        disconnect();
    }

    m_hostname = hostname;
    m_port = port;

    m_socket->connectToHost(hostname, port);
    m_connectionCheckTimer.start();

    // Return immediately, the actual status will be reported via signals
    return true;
}

void HexapodConnection::disconnect()
{
    m_connectionCheckTimer.stop();
    if (m_socket->state() != QAbstractSocket::UnconnectedState)
    {
        m_socket->disconnectFromHost();
    }
}

bool HexapodConnection::isConnected() const
{
    return m_socket->state() == QAbstractSocket::ConnectedState;
}

void HexapodConnection::sendCommand(const QJsonObject &command)
{
    if (!isConnected())
    {
        emit errorOccurred("Cannot send command: not connected");
        return;
    }

    QJsonDocument doc(command);
    QByteArray data = doc.toJson(QJsonDocument::Compact);

    // Add a terminator for message framing
    data.append('\n');

    m_socket->write(data);
}

QJsonObject HexapodConnection::createCommandObject(const QString &command)
{
    QJsonObject cmd;
    cmd["command"] = command;
    cmd["timestamp"] = QDateTime::currentMSecsSinceEpoch();
    return cmd;
}

void HexapodConnection::requestImuData()
{
    QJsonObject cmd = createCommandObject("getImuData");
    sendCommand(cmd);
}

bool HexapodConnection::setLegPosition(uint8_t legNum, int16_t hip, int16_t knee, int16_t ankle)
{
    if (!isConnected())
    {
        return false;
    }

    QJsonObject cmd = createCommandObject("setLegPosition");
    cmd["leg_num"] = legNum;
    cmd["hip"] = hip;
    cmd["knee"] = knee;
    cmd["ankle"] = ankle;

    sendCommand(cmd);
    return true; // Actual success will be known when response is received
}

bool HexapodConnection::centerAllLegs()
{
    if (!isConnected())
    {
        return false;
    }

    QJsonObject cmd = createCommandObject("centerAll");
    sendCommand(cmd);
    return true;
}

bool HexapodConnection::setCalibration(uint8_t legNum, int16_t hipOffset, int16_t kneeOffset, int16_t ankleOffset)
{
    if (!isConnected())
    {
        return false;
    }

    QJsonObject cmd = createCommandObject("setCalibration");
    cmd["leg_num"] = legNum;
    cmd["hip_offset"] = hipOffset;
    cmd["knee_offset"] = kneeOffset;
    cmd["ankle_offset"] = ankleOffset;

    sendCommand(cmd);
    return true;
}

void HexapodConnection::handleSocketConnected()
{
    emit connectionStatusChanged(true);
}

void HexapodConnection::handleSocketDisconnected()
{
    emit connectionStatusChanged(false);
}

void HexapodConnection::handleSocketError(QAbstractSocket::SocketError socketError)
{
    // Mark parameter as unused to avoid warning
    (void)socketError;

    emit errorOccurred("Socket error: " + m_socket->errorString());
}

void HexapodConnection::handleSocketReadyRead()
{
    m_buffer.append(m_socket->readAll());
    processIncomingData();
}

void HexapodConnection::processIncomingData()
{
    // Process complete messages in buffer
    int endIndex;
    while ((endIndex = m_buffer.indexOf('\n')) != -1)
    {
        QByteArray messageData = m_buffer.left(endIndex);
        m_buffer.remove(0, endIndex + 1);

        QJsonParseError parseError;
        QJsonDocument doc = QJsonDocument::fromJson(messageData, &parseError);

        if (parseError.error != QJsonParseError::NoError)
        {
            emit errorOccurred("JSON parse error: " + parseError.errorString());
            continue;
        }

        if (!doc.isObject())
        {
            emit errorOccurred("Invalid message format: not a JSON object");
            continue;
        }

        QJsonObject response = doc.object();

        // Handle specific response types
        if (response.contains("imu_data"))
        {
            QJsonObject imuData = response["imu_data"].toObject();
            emit imuDataReceived(
                imuData["accel_x"].toDouble(),
                imuData["accel_y"].toDouble(),
                imuData["accel_z"].toDouble(),
                imuData["gyro_x"].toDouble(),
                imuData["gyro_y"].toDouble(),
                imuData["gyro_z"].toDouble());
        }

        // General response notification
        emit responseReceived(response);
    }
}

void HexapodConnection::checkConnection()
{
    // If we're not connected but have connection details, try to reconnect
    if (!isConnected() && !m_hostname.isEmpty() && m_port > 0)
    {
        m_socket->connectToHost(m_hostname, m_port);
    }
}
