#ifndef HEXAPODCONTEXT_H
#define HEXAPODCONTEXT_H

#include <QObject>
#include <QTimer>
#include <QJsonObject>
#include <QVariantMap>

// Forward declarations
class HexapodConnection;
class HexapodVisualization;

/**
 * @brief The HexapodContext class provides a bridge between C++ backend and QML frontend
 *
 * This class exposes hexapod control functionality to QML and provides
 * properties and signals that QML components can bind to.
 */
class HexapodContext : public QObject
{
    Q_OBJECT

    // Properties exposed to QML
    Q_PROPERTY(bool connected READ isConnected NOTIFY connectionStatusChanged)
    Q_PROPERTY(QString statusMessage READ statusMessage NOTIFY statusMessageChanged)
    Q_PROPERTY(double speed READ speed WRITE setSpeed NOTIFY speedChanged)
    Q_PROPERTY(double pitch READ pitch WRITE setPitch NOTIFY pitchChanged)
    Q_PROPERTY(double roll READ roll WRITE setRoll NOTIFY rollChanged)
    Q_PROPERTY(bool balanceEnabled READ balanceEnabled WRITE setBalanceEnabled NOTIFY balanceEnabledChanged)
    Q_PROPERTY(QVariantMap imuData READ imuData NOTIFY imuDataChanged)

public:
    explicit HexapodContext(QObject *parent = nullptr);
    ~HexapodContext();

    // Property getters
    bool isConnected() const;
    QString statusMessage() const;
    double speed() const;
    double pitch() const;
    double roll() const;
    bool balanceEnabled() const;
    QVariantMap imuData() const;

    // Property setters
    void setSpeed(double speed);
    void setPitch(double pitch);
    void setRoll(double roll);
    void setBalanceEnabled(bool enabled);

    // Connection instance management
    void setConnection(HexapodConnection *connection);
    HexapodConnection *connection() const;

    // Visualization instance management
    void setVisualization(HexapodVisualization *visualization);
    HexapodVisualization *visualization() const;

public slots:
    // Connection methods
    bool connectToHost(const QString &hostname, int port);
    void disconnect();

    // Movement control
    void moveForward(double speed);
    void moveBackward(double speed);
    void turnLeft(double speed);
    void turnRight(double speed);
    void stop();

    // Position control
    void centerAllLegs();

    // Log messages
    void logMessage(const QString &message);

    void setConnected(bool connected)
    {
        if (m_connected != connected)
        {
            m_connected = connected;
            emit connectionStatusChanged(m_connected);
        }
    }

signals:
    void connectionStatusChanged(bool connected);
    void statusMessageChanged(const QString &message);
    void speedChanged(double speed);
    void pitchChanged(double pitch);
    void rollChanged(double roll);
    void balanceEnabledChanged(bool enabled);
    void imuDataChanged(const QVariantMap &data);
    void messageLogged(const QString &message);
    void imuDataReceived(double accelX, double accelY, double accelZ,
                         double gyroX, double gyroY, double gyroZ);
    void commandResponse(const QJsonObject &response);
    void errorOccurred(const QString &message);

private slots:
    void handleConnectionStatus(bool connected);
    void handleImuDataReceived(double accelX, double accelY, double accelZ,
                               double gyroX, double gyroY, double gyroZ);
    void handleResponseReceived(const QJsonObject &response);
    void handleErrorOccurred(const QString &error);
    void requestImuUpdate();

private:
    HexapodConnection *m_connection;
    HexapodVisualization *m_visualization;
    QTimer *m_imuUpdateTimer;
    QString m_statusMessage;
    double m_speed;
    double m_pitch;
    double m_roll;
    bool m_balanceEnabled;
    QVariantMap m_imuData;
    bool m_connected;
};

#endif // HEXAPODCONTEXT_H
