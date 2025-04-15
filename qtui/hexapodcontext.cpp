#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <QDebug>
#include <QOverload>
#include "hexapodcontext.h"

HexapodContext::HexapodContext(QObject *parent)
    : QObject(parent), m_hostname("beaglebone.local"), m_port(8080), m_connected(false), m_connecting(false), m_simulationMode(false), m_socket(new QTcpSocket(this)), m_speed(0.5), m_balanceEnabled(false), m_pitch(0.0), m_roll(0.0), m_accelX(0.0), m_accelY(0.0), m_accelZ(1.0), m_gyroX(0.0), m_gyroY(0.0), m_gyroZ(0.0), m_reconnectAttempts(0), m_shutdownRequested(false)
{
    // Connect socket signals
    connect(m_socket, &QTcpSocket::connected, this, &HexapodContext::onConnected);
    connect(m_socket, &QTcpSocket::disconnected, this, &HexapodContext::onDisconnected);
    connect(m_socket, &QTcpSocket::readyRead, this, &HexapodContext::onReadyRead);

    // Connect socket error signal using the new-style cast syntax
    connect(m_socket, &QAbstractSocket::errorOccurred, this, &HexapodContext::onSocketError);

    // Setup IMU update timer
    connect(&m_timer, &QTimer::timeout, this, &HexapodContext::onTimerTimeout);

    // Setup ping timer
    connect(&m_pingTimer, &QTimer::timeout, this, &HexapodContext::onPingTimerTimeout);
}

HexapodContext::~HexapodContext()
{
    m_shutdownRequested = true;

    // Disconnect gracefully
    if (m_connected)
    {
        disconnectFromServer();
    }

    m_socket->deleteLater();
}

void HexapodContext::setHostname(const QString &hostname)
{
    if (m_hostname != hostname)
    {
        m_hostname = hostname;
    }
}

void HexapodContext::setPort(int port)
{
    if (m_port != port && port > 0)
    {
        m_port = port;
    }
}

void HexapodContext::moveForward()
{
    if (!m_connected)
        return;

    QJsonObject command;
    command["command"] = "setMovement";
    command["direction"] = 0.0; // Forward
    command["speed"] = m_speed;

    sendCommand(command);
    logMessage("Moving forward");
}

void HexapodContext::moveBackward()
{
    if (!m_connected)
        return;

    QJsonObject command;
    command["command"] = "setMovement";
    command["direction"] = 180.0; // Backward
    command["speed"] = m_speed;

    sendCommand(command);
    logMessage("Moving backward");
}

void HexapodContext::moveLeft()
{
    if (!m_connected)
        return;

    QJsonObject command;
    command["command"] = "setRotation";
    command["direction"] = -1.0; // Left
    command["speed"] = m_speed;

    sendCommand(command);
    logMessage("Turning left");
}

void HexapodContext::moveRight()
{
    if (!m_connected)
        return;

    QJsonObject command;
    command["command"] = "setRotation";
    command["direction"] = 1.0; // Right
    command["speed"] = m_speed;

    sendCommand(command);
    logMessage("Turning right");
}

void HexapodContext::stopMovement()
{
    if (!m_connected)
        return;

    QJsonObject command;
    command["command"] = "stop";

    sendCommand(command);
    logMessage("Stop command sent");
}

void HexapodContext::setSpeed(double speed)
{
    if (m_speed != speed)
    {
        m_speed = qBound(0.0, speed, 1.0);
        emit speedChanged(m_speed);
        logMessage(QString("%1%").arg(int(m_speed * 100)));
    }
}

void HexapodContext::setBalanceEnabled(bool enabled)
{
    if (m_balanceEnabled != enabled)
    {
        m_balanceEnabled = enabled;

        if (m_connected)
        {
            QJsonObject command;
            command["command"] = "setBalance";
            command["enabled"] = m_balanceEnabled;

            sendCommand(command);
            logMessage(QString("Balance mode %1").arg(enabled ? "enabled" : "disabled"));
        }

        emit balanceEnabledChanged(m_balanceEnabled);
    }
}

void HexapodContext::setTilt(double pitch, double roll)
{
    if (m_pitch != pitch || m_roll != roll)
    {
        m_pitch = pitch;
        m_roll = roll;

        if (m_connected)
        {
            QJsonObject command;
            command["command"] = "setTilt";
            command["pitch"] = m_pitch;
            command["roll"] = m_roll;

            sendCommand(command);
            logMessage(QString("%1°").arg(int(m_roll)));
        }
    }
}

void HexapodContext::connectToServer()
{
    if (m_connected || m_connecting)
        return;

    m_connecting = true;
    m_buffer.clear();
    resetConnectionState();

    logMessage("Connecting to server");
    m_socket->connectToHost(m_hostname, m_port);
}

void HexapodContext::disconnectFromServer()
{
    if (!m_connected && !m_connecting)
        return;

    // Stop timers
    m_timer.stop();
    m_pingTimer.stop();

    // Send stop command before disconnecting to ensure robot safety
    if (m_connected)
    {
        QJsonObject command;
        command["command"] = "stop";
        sendCommand(command);
    }

    m_socket->disconnectFromHost();

    // If we're in the middle of a connection attempt
    if (m_socket->state() != QAbstractSocket::UnconnectedState)
    {
        m_socket->abort();
    }

    resetConnectionState();
    m_connected = false;
    m_connecting = false;

    emit connectionStateChanged(false);
    logMessage("Disconnected from server");
}

void HexapodContext::onConnected()
{
    m_connected = true;
    m_connecting = false;
    m_reconnectAttempts = 0;

    logMessage("Connected to server");
    emit connectionStateChanged(true);

    // Start timers for regular updates
    m_timer.start(IMU_UPDATE_INTERVAL_MS);
    m_pingTimer.start(PING_INTERVAL_MS);

    // Send initial ping to get server info
    QJsonObject pingCommand;
    pingCommand["command"] = "ping";
    sendCommand(pingCommand);

    // Apply current balance setting
    if (m_balanceEnabled)
    {
        QJsonObject balanceCommand;
        balanceCommand["command"] = "setBalance";
        balanceCommand["enabled"] = true;
        sendCommand(balanceCommand);
    }
}

void HexapodContext::onDisconnected()
{
    // Only attempt reconnect if not shutting down and we were previously connected
    if (!m_shutdownRequested && m_connected)
    {
        m_connected = false;
        m_connecting = false;
        emit connectionStateChanged(false);

        // Attempt reconnection if not intentionally disconnected
        if (m_reconnectAttempts < MAX_RECONNECT_ATTEMPTS)
        {
            m_reconnectAttempts++;
            logMessage(QString("Connection lost. Reconnecting (attempt %1/%2)...").arg(m_reconnectAttempts).arg(MAX_RECONNECT_ATTEMPTS), "INFO");

            // Wait before reconnecting
            QTimer::singleShot(RECONNECT_INTERVAL_MS, this, &HexapodContext::connectToServer);
        }
        else
        {
            logMessage("ERROR: Disconnected from server", "ERROR");
        }
    }
}

void HexapodContext::onSocketError(QAbstractSocket::SocketError error)
{
    QString errorMsg;

    switch (error)
    {
    case QAbstractSocket::ConnectionRefusedError:
        errorMsg = "Connection refused. Is the server running?";
        break;
    case QAbstractSocket::RemoteHostClosedError:
        errorMsg = "Connection error: The remote host closed the connection";
        break;
    case QAbstractSocket::HostNotFoundError:
        errorMsg = "Host not found. Check the hostname and network connection.";
        break;
    case QAbstractSocket::NetworkError:
        errorMsg = "Network error. Check your connection.";
        break;
    case QAbstractSocket::SocketTimeoutError:
        errorMsg = "Connection timed out.";
        break;
    default:
        errorMsg = QString("Socket error: %1").arg(m_socket->errorString());
    }

    // Log and emit the error
    logMessage(errorMsg, "ERROR");
    emit connectionError(errorMsg);

    // Reset connection state
    m_connecting = false;
}

void HexapodContext::onReadyRead()
{
    // Read all available data and append to buffer
    m_buffer.append(m_socket->readAll());

    // Process each complete message (assuming \n delimiter)
    while (m_buffer.contains('\n'))
    {
        int newlinePos = m_buffer.indexOf('\n');
        QByteArray message = m_buffer.left(newlinePos);
        m_buffer.remove(0, newlinePos + 1);

        // Skip empty messages
        if (message.isEmpty())
            continue;

        // Try to parse JSON
        QJsonParseError parseError;
        QJsonDocument jsonDoc = QJsonDocument::fromJson(message, &parseError);

        if (parseError.error != QJsonParseError::NoError)
        {
            // If the parse fails, try to handle HTML-like responses which may be
            // causing issues (e.g. from a proxy or misconfigured server)
            if (message.startsWith("<") || message.contains("</body>") || message.contains("</html>"))
            {
                logMessage(QString("ERROR: Received HTML instead of JSON. Check server configuration."), "ERROR");
                qDebug() << "Received HTML-like response:" << message.left(100) << "...";
                continue;
            }

            logMessage(QString("ERROR: JSON parse error: %1 (Data: %2)")
                           .arg(parseError.errorString())
                           .arg(QString(message.toHex())),
                       "ERROR");
            continue;
        }

        processResponse(jsonDoc);
    }
}

void HexapodContext::onTimerTimeout()
{
    // Get IMU data periodically
    if (m_connected)
    {
        QJsonObject command;
        command["command"] = "getImuData";
        sendCommand(command);
    }
}

void HexapodContext::onPingTimerTimeout()
{
    // Send a ping to check connection health
    if (m_connected)
    {
        m_lastPingTime = QDateTime::currentMSecsSinceEpoch();

        QJsonObject command;
        command["command"] = "ping";
        command["timestamp"] = m_lastPingTime;
        sendCommand(command);
    }
}

void HexapodContext::sendCommand(const QJsonObject &command)
{
    if (!m_connected && command["command"].toString() != "ping")
    {
        logMessage("Cannot send command: not connected", "ERROR");
        return;
    }

    QJsonDocument doc(command);
    QByteArray data = doc.toJson(QJsonDocument::Compact);
    data.append('\n'); // Server expects newline delimiter

    // Check if socket is valid and connection is active
    if (m_socket->state() == QAbstractSocket::ConnectedState)
    {
        qint64 bytesWritten = m_socket->write(data);

        if (bytesWritten == -1)
        {
            logMessage("Failed to write data to socket", "ERROR");
        }
        else if (bytesWritten != data.size())
        {
            logMessage("Incomplete write to socket", "ERROR");
        }
    }
    else
    {
        logMessage("Cannot send command: socket not connected", "ERROR");
        m_connected = false;
        emit connectionStateChanged(false);
    }
}

void HexapodContext::processResponse(const QJsonDocument &response)
{
    // Check if response is valid and is an object
    if (!response.isObject())
    {
        logMessage("ERROR: Invalid JSON response (not an object)", "ERROR");
        return;
    }

    QJsonObject obj = response.object();

    // Check status
    QString status = obj["status"].toString();

    if (status == "error")
    {
        QString errorMessage = obj["message"].toString();
        logMessage("ERROR: " + errorMessage, "ERROR");
        return;
    }

    // Process specific response types
    QString command = obj["command"].toString();

    // Ping response - contains server info
    if (obj.contains("timestamp") || command == "ping")
    {
        // Process ping response - check for simulation mode
        if (obj.contains("simulation"))
        {
            bool simMode = obj["simulation"].toBool();
            if (simMode != m_simulationMode)
            {
                m_simulationMode = simMode;
                emit simulationModeChanged(m_simulationMode);
                logMessage(m_simulationMode ? "Running in SIMULATION mode" : "Connected to hardware");
            }
        }

        // Calculate ping time if we have the timestamp
        if (obj.contains("timestamp") && m_lastPingTime > 0)
        {
            qint64 now = QDateTime::currentMSecsSinceEpoch();
            qint64 pingTime = now - m_lastPingTime;
            qDebug() << "Ping time:" << pingTime << "ms";
        }
    }

    // IMU data response
    if (obj.contains("imu_data"))
    {
        updateImuData(obj["imu_data"].toObject());
    }

    // Log successful response messages
    if (obj.contains("message"))
    {
        QString message = obj["message"].toString();
        if (!message.isEmpty())
        {
            logMessage("SUCCESS: " + message, "SUCCESS");
        }
    }
}

void HexapodContext::updateImuData(const QJsonObject &imuData)
{
    // Update IMU data values
    m_accelX = imuData["accel_x"].toDouble();
    m_accelY = imuData["accel_y"].toDouble();
    m_accelZ = imuData["accel_z"].toDouble();
    m_gyroX = imuData["gyro_x"].toDouble();
    m_gyroY = imuData["gyro_y"].toDouble();
    m_gyroZ = imuData["gyro_z"].toDouble();

    // Notify that IMU data has been updated
    emit imuDataUpdated();

    // Debug log IMU data at lower frequency to avoid console spam
    static int counter = 0;
    if (++counter % 20 == 0)
    { // Log every 20th update
        logMessage(QString("IMU: Accel(%.2f,%.2f,%.2f) Gyro(%.2f,%.2f,%.2f)")
                       .arg(m_accelX)
                       .arg(m_accelY)
                       .arg(m_accelZ)
                       .arg(m_gyroX)
                       .arg(m_gyroY)
                       .arg(m_gyroZ));
        counter = 0;
    }
}

void HexapodContext::resetConnectionState()
{
    m_simulationMode = false;
    m_buffer.clear();

    // Clear IMU data
    m_accelX = 0.0;
    m_accelY = 0.0;
    m_accelZ = 1.0; // Default to gravity
    m_gyroX = 0.0;
    m_gyroY = 0.0;
    m_gyroZ = 0.0;

    emit imuDataUpdated();
}

void HexapodContext::logMessage(const QString &message, const QString &type)
{
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString formattedMessage = QString("[%1] %2").arg(timestamp).arg(message);

    emit messageReceived(formattedMessage, type);

    // Also send to debug output
    if (type == "ERROR")
    {
        qDebug() << "ERROR: " << message;
    }
    else if (type == "SUCCESS")
    {
        qDebug() << "SUCCESS: " << message;
    }
    else
    {
        qDebug() << "INFO: " << message;
    }
}
