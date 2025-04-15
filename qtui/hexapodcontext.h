#ifndef HEXAPODCONTEXT_H
#define HEXAPODCONTEXT_H

#include <QObject>
#include <QString>
#include <QHostAddress>
#include <QTcpSocket>
#include <QTimer>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDateTime>

/**
 * @brief The HexapodContext class manages shared application state
 * and communication with the hexapod server.
 */
class HexapodContext : public QObject
{
    Q_OBJECT

public:
    explicit HexapodContext(QObject *parent = nullptr);
    ~HexapodContext();

    // Connection settings
    QString hostname() const { return m_hostname; }
    void setHostname(const QString &hostname);

    int port() const { return m_port; }
    void setPort(int port);

    bool isConnected() const { return m_connected; }
    bool isSimulationMode() const { return m_simulationMode; }

    // Movement control
    void moveForward();
    void moveBackward();
    void moveLeft();
    void moveRight();
    void stopMovement();

    // Speed control
    double speed() const { return m_speed; }
    void setSpeed(double speed);

    // Balance control
    bool isBalanceEnabled() const { return m_balanceEnabled; }
    void setBalanceEnabled(bool enabled);

    // Tilt control
    void setTilt(double pitch, double roll);

    // Connection management
    void connectToServer();
    void disconnectFromServer();

    // IMU data access
    double accelX() const { return m_accelX; }
    double accelY() const { return m_accelY; }
    double accelZ() const { return m_accelZ; }
    double gyroX() const { return m_gyroX; }
    double gyroY() const { return m_gyroY; }
    double gyroZ() const { return m_gyroZ; }

signals:
    void connectionStateChanged(bool connected);
    void simulationModeChanged(bool simulation);
    void imuDataUpdated();
    void messageReceived(const QString &message, const QString &type);
    void balanceEnabledChanged(bool enabled);
    void speedChanged(double speed);
    void connectionError(const QString &errorMessage);

private slots:
    void onConnected();
    void onDisconnected();
    void onSocketError(QAbstractSocket::SocketError error);
    void onReadyRead();
    void onTimerTimeout();
    void onPingTimerTimeout();

private:
    void sendCommand(const QJsonObject &command);
    void processResponse(const QJsonDocument &response);
    void updateImuData(const QJsonObject &imuData);
    void resetConnectionState();
    void logMessage(const QString &message, const QString &type = "INFO");

    // Connection properties
    QString m_hostname;
    int m_port;
    bool m_connected;
    bool m_connecting;
    bool m_simulationMode;
    QTcpSocket *m_socket;
    QByteArray m_buffer;

    // Timers for polling and ping
    QTimer m_timer;
    QTimer m_pingTimer;
    qint64 m_lastPingTime;

    // Control state
    double m_speed;
    bool m_balanceEnabled;
    double m_pitch;
    double m_roll;

    // Sensor data
    double m_accelX;
    double m_accelY;
    double m_accelZ;
    double m_gyroX;
    double m_gyroY;
    double m_gyroZ;

    // Internal flags
    int m_reconnectAttempts;
    bool m_shutdownRequested;

    // Constants
    static constexpr int MAX_RECONNECT_ATTEMPTS = 3;
    static constexpr int RECONNECT_INTERVAL_MS = 2000;
    static constexpr int PING_INTERVAL_MS = 5000;
    static constexpr int IMU_UPDATE_INTERVAL_MS = 100;
};

#endif // HEXAPODCONTEXT_H
