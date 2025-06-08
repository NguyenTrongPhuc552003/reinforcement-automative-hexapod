#ifndef HEXAPODCONNECTION_H
#define HEXAPODCONNECTION_H

#include <QObject>
#include <QTcpSocket>
#include <QTimer>
#include <QHostInfo>
#include <QJsonObject>
#include <QJsonDocument>
#include <QPair>
#include <memory>
#include <vector>

class HexapodConnection : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool connecting READ isConnecting NOTIFY connectionInProgress)

public:
    explicit HexapodConnection(QObject *parent = nullptr);
    ~HexapodConnection();

    bool connectToHost(const QString &hostname, int port);
    void disconnect();
    bool isConnected() const;

    // Server discovery
    void startServerDiscovery();
    void stopServerDiscovery();

    // Command interface mirroring hexapod.hpp functionality
    bool setLegPosition(uint8_t legNum, int16_t hip, int16_t knee, int16_t ankle);
    bool centerAllLegs();
    bool setCalibration(uint8_t legNum, int16_t hipOffset, int16_t kneeOffset, int16_t ankleOffset);

    // Connection info
    QString getConnectedHost() const { return m_hostname; }
    int getConnectedPort() const { return m_port; }
    bool isInSimulationMode() const { return m_simulationMode; }
    bool isConnecting() const
    {
        return m_socket->state() == QAbstractSocket::ConnectingState;
    }

public slots:
    void sendCommand(const QJsonObject &command);
    void requestImuData();
    void sendPing();

signals:
    void connectionStatusChanged(bool connected);
    void connectionInProgress(bool connecting);
    void responseReceived(const QJsonObject &response);
    void imuDataReceived(double accelX, double accelY, double accelZ, double gyroX, double gyroY, double gyroZ);
    void errorOccurred(const QString &errorMessage);

    // Discovery signals
    void serverFound(const QString &hostname, int port, bool isSimulation);
    void discoveryComplete();
    void discoveryProgress(int current, int total);

private slots:
    void handleSocketConnected();
    void handleSocketDisconnected();
    void handleSocketError(QAbstractSocket::SocketError socketError);
    void handleSocketReadyRead();
    void checkConnection();
    void attemptNextDiscoveryServer();
    void tryConnect(const QString &hostname, int port);
    void handleHostLookupResult(const QHostInfo &hostInfo);

private:
    QTcpSocket *m_socket;
    QTimer m_connectionCheckTimer;
    QTimer m_discoveryTimer;
    QString m_hostname;
    int m_port;
    QByteArray m_buffer;
    bool m_simulationMode;
    bool m_autoReconnect;
    int m_reconnectAttempts;
    int m_maxReconnectAttempts;
    int m_reconnectDelay;
    bool m_discoveryInProgress;

    QList<QPair<QString, int>> m_discoveryServers;
    int m_currentDiscoveryIndex;

    QJsonObject createCommandObject(const QString &command);
    void processIncomingData();
    void resetConnectionState();
    void scheduleReconnect();
    void initializeDiscoveryServerList();
    void parseServerInfo(const QJsonObject &response);
    bool checkForSimulationFlag(const QJsonObject &response);
};

#endif // HEXAPODCONNECTION_H
