#include <QHostAddress>
#include <QJsonArray>
#include <QNetworkInterface>
#include <QDebug>
#include "hexapodconnection.h"

HexapodConnection::HexapodConnection(QObject *parent)
    : QObject(parent),
      m_socket(new QTcpSocket(this)),
      m_port(0),
      m_simulationMode(false),
      m_autoReconnect(true),
      m_reconnectAttempts(0),
      m_maxReconnectAttempts(3),
      m_reconnectDelay(2000), // 2 seconds
      m_discoveryInProgress(false),
      m_currentDiscoveryIndex(0)
{
    // Connect socket signals
    connect(m_socket, &QTcpSocket::connected, this, &HexapodConnection::handleSocketConnected);
    connect(m_socket, &QTcpSocket::disconnected, this, &HexapodConnection::handleSocketDisconnected);
    connect(m_socket, &QTcpSocket::errorOccurred, this, &HexapodConnection::handleSocketError);
    connect(m_socket, &QTcpSocket::readyRead, this, &HexapodConnection::handleSocketReadyRead);

    // Set up connection check timer
    connect(&m_connectionCheckTimer, &QTimer::timeout, this, &HexapodConnection::checkConnection);
    m_connectionCheckTimer.setInterval(5000); // Check every 5 seconds

    // Set up discovery timer
    connect(&m_discoveryTimer, &QTimer::timeout, this, &HexapodConnection::attemptNextDiscoveryServer);
    m_discoveryTimer.setInterval(500); // Try next server every 0.5 seconds

    // Initialize discovery server list
    initializeDiscoveryServerList();
}

HexapodConnection::~HexapodConnection()
{
    disconnect();
}

bool HexapodConnection::connectToHost(const QString &hostname, int port)
{
    if (m_discoveryInProgress)
    {
        stopServerDiscovery();
    }

    if (isConnected() && m_hostname == hostname && m_port == port)
    {
        // Already connected to requested host
        return true;
    }

    if (isConnected())
    {
        disconnect();
    }

    m_hostname = hostname;
    m_port = port;
    m_reconnectAttempts = 0;

    emit connectionInProgress(true);

    // Try to resolve hostname first
    QHostInfo::lookupHost(hostname, this, SLOT(handleHostLookupResult(QHostInfo)));
    return true;
}

void HexapodConnection::handleHostLookupResult(const QHostInfo &hostInfo)
{
    if (hostInfo.error() != QHostInfo::NoError)
    {
        emit errorOccurred(QString("Hostname lookup failed: %1").arg(hostInfo.errorString()));
        tryConnect(m_hostname, m_port); // Fall back to direct connect
        return;
    }

    // If we have addresses, use the first one
    if (!hostInfo.addresses().isEmpty())
    {
        QString ipAddress = hostInfo.addresses().first().toString();
        qDebug() << "Resolved" << m_hostname << "to" << ipAddress;
        tryConnect(ipAddress, m_port);
    }
    else
    {
        tryConnect(m_hostname, m_port); // Fall back to direct connect
    }
}

void HexapodConnection::tryConnect(const QString &hostname, int port)
{
    qDebug() << "Attempting connection to" << hostname << ":" << port;
    m_socket->connectToHost(hostname, port);
    m_connectionCheckTimer.start();
}

void HexapodConnection::disconnect()
{
    m_connectionCheckTimer.stop();
    m_discoveryTimer.stop();
    m_discoveryInProgress = false;
    m_autoReconnect = false;

    if (m_socket->state() != QAbstractSocket::UnconnectedState)
    {
        m_socket->disconnectFromHost();
        if (m_socket->state() != QAbstractSocket::UnconnectedState)
            m_socket->waitForDisconnected(1000);
    }
}

bool HexapodConnection::isConnected() const
{
    return m_socket->state() == QAbstractSocket::ConnectedState;
}

void HexapodConnection::startServerDiscovery()
{
    if (m_discoveryInProgress)
    {
        return;
    }

    if (isConnected())
    {
        disconnect();
    }

    m_discoveryInProgress = true;
    m_currentDiscoveryIndex = 0;

    emit discoveryProgress(0, m_discoveryServers.size());

    // Start discovery process
    attemptNextDiscoveryServer();
}

void HexapodConnection::stopServerDiscovery()
{
    if (!m_discoveryInProgress)
    {
        return;
    }

    m_discoveryTimer.stop();
    m_discoveryInProgress = false;

    if (m_socket->state() == QAbstractSocket::ConnectingState)
    {
        m_socket->abort();
    }

    emit discoveryComplete();
}

void HexapodConnection::attemptNextDiscoveryServer()
{
    if (!m_discoveryInProgress || m_currentDiscoveryIndex >= m_discoveryServers.size())
    {
        // Discovery complete
        m_discoveryInProgress = false;
        m_discoveryTimer.stop();
        emit discoveryComplete();
        return;
    }

    // Abort previous connection attempt if still in progress
    if (m_socket->state() == QAbstractSocket::ConnectingState)
    {
        m_socket->abort();
    }

    // Get next server to try
    const QPair<QString, int> &server = m_discoveryServers.at(m_currentDiscoveryIndex);
    QString hostname = server.first;
    int port = server.second;

    qDebug() << "Trying discovery connection to" << hostname << ":" << port;

    // Update progress
    emit discoveryProgress(m_currentDiscoveryIndex + 1, m_discoveryServers.size());

    // Try to connect to this server
    m_socket->connectToHost(hostname, port);

    // Move to next server for next attempt
    m_currentDiscoveryIndex++;

    // Start timer for next attempt
    m_discoveryTimer.start();
}

void HexapodConnection::sendCommand(const QJsonObject &command)
{
    if (!isConnected())
    {
        emit errorOccurred("Cannot send command: not connected");
        return;
    }

    QJsonDocument doc = QJsonDocument(command);
    QByteArray data = doc.toJson(QJsonDocument::Compact);

    // Add a terminator for message framing
    data.append('\n');

    m_socket->write(data);
}

void HexapodConnection::sendPing()
{
    if (!isConnected())
    {
        return;
    }

    QJsonObject cmd = createCommandObject("ping");
    cmd["timestamp"] = QDateTime::currentMSecsSinceEpoch();
    sendCommand(cmd);
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
    qDebug() << "Connected to server at" << m_socket->peerName() << ":" << m_socket->peerPort();

    // Reset reconnect attempts on successful connection
    m_reconnectAttempts = 0;

    // Send ping to check server status
    sendPing();

    if (m_discoveryInProgress)
    {
        // If we're in discovery mode, report this server
        m_discoveryTimer.stop();

        // Save the successful connection info
        m_hostname = m_socket->peerName().isEmpty() ? m_socket->peerAddress().toString() : m_socket->peerName();
        m_port = m_socket->peerPort();

        emit serverFound(m_hostname, m_port, m_simulationMode);
    }

    emit connectionStatusChanged(true);
    emit connectionInProgress(false);
}

void HexapodConnection::handleSocketDisconnected()
{
    qDebug() << "Disconnected from server";

    emit connectionStatusChanged(false);

    if (m_autoReconnect && !m_discoveryInProgress && m_reconnectAttempts < m_maxReconnectAttempts)
    {
        scheduleReconnect();
    }
}

void HexapodConnection::handleSocketError(QAbstractSocket::SocketError socketError)
{
    QString errorMessage = m_socket->errorString();
    qDebug() << "Socket error:" << socketError << "-" << errorMessage;

    if (m_discoveryInProgress)
    {
        // In discovery mode, just try next server
        return;
    }

    // For connection attempts, emit error and consider reconnection
    emit errorOccurred(QString("Connection error: %1").arg(errorMessage));
    emit connectionInProgress(false);

    if (m_autoReconnect && m_reconnectAttempts < m_maxReconnectAttempts)
    {
        scheduleReconnect();
    }
}

void HexapodConnection::scheduleReconnect()
{
    m_reconnectAttempts++;

    qDebug() << "Scheduling reconnection attempt" << m_reconnectAttempts
             << "of" << m_maxReconnectAttempts;

    // Use single-shot timer to delay reconnection
    QTimer::singleShot(m_reconnectDelay, this, [this]()
                       {
        if (m_autoReconnect && !isConnected()) {
            qDebug() << "Attempting to reconnect to" << m_hostname << ":" << m_port;
            tryConnect(m_hostname, m_port);
        } });
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

        // Skip empty messages
        if (messageData.trimmed().isEmpty())
        {
            continue;
        }

        QJsonParseError parseError;
        QJsonDocument doc = QJsonDocument::fromJson(messageData, &parseError);

        if (parseError.error != QJsonParseError::NoError)
        {
            emit errorOccurred("JSON parse error: " + parseError.errorString() +
                               " (Data: " + messageData.left(50).toHex() +
                               (messageData.size() > 50 ? "..." : "") + ")");
            continue;
        }

        if (!doc.isObject())
        {
            emit errorOccurred("Invalid message format: not a JSON object");
            continue;
        }

        QJsonObject response = doc.object();

        // Check if this is server info (simulation flag)
        parseServerInfo(response);

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

void HexapodConnection::parseServerInfo(const QJsonObject &response)
{
    // Check for simulation mode flag
    if (response.contains("simulation"))
    {
        m_simulationMode = response["simulation"].toBool();
    }

    // Check for ping response with timestamp
    if (response.contains("command") && response["command"].toString() == "ping")
    {
        // This is a ping response, check for simulation flag
        m_simulationMode = checkForSimulationFlag(response);
    }
}

bool HexapodConnection::checkForSimulationFlag(const QJsonObject &response)
{
    if (response.contains("simulation"))
    {
        return response["simulation"].toBool();
    }
    return false;
}

void HexapodConnection::checkConnection()
{
    // If we're connected, send a ping to check if connection is still alive
    if (isConnected())
    {
        sendPing();
    }
}

void HexapodConnection::initializeDiscoveryServerList()
{
    m_discoveryServers.clear();

    // Common hostnames to try
    QStringList hostnames = {
        "beaglebone.local", "beaglebone", "localhost", "192.168.7.2", "192.168.8.1"};

    // Common ports to try
    QList<int> ports = {8080, 8081, 8082, 9090, 8090};

    // Add all hostname/port combinations
    for (const QString &hostname : hostnames)
    {
        for (int port : ports)
        {
            m_discoveryServers.append(qMakePair(hostname, port));
        }
    }

    // Now try to find IPs on the local network
    for (const QHostAddress &address : QNetworkInterface::allAddresses())
    {
        if (address.protocol() != QAbstractSocket::IPv4Protocol ||
            address == QHostAddress::LocalHost ||
            address.toString().startsWith("169.254"))
        {
            continue; // Skip non-IPv4, localhost, and link-local addresses
        }

        // Get base network address
        QString baseIp = address.toString();
        int lastDotPos = baseIp.lastIndexOf('.');
        if (lastDotPos > 0)
        {
            baseIp = baseIp.left(lastDotPos + 1);

            // Add some common IPs in the subnet
            // We don't scan the whole subnet to keep the list manageable
            for (int i = 1; i <= 5; i++)
            {
                m_discoveryServers.append(qMakePair(baseIp + QString::number(i), 8080));
            }
        }
    }
}

void HexapodConnection::resetConnectionState()
{
    // Reset state variables
    m_reconnectAttempts = 0;
    m_autoReconnect = true;
}
