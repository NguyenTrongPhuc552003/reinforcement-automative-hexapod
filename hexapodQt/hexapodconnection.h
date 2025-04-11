#ifndef HEXAPODCONNECTION_H
#define HEXAPODCONNECTION_H

#include <QObject>
#include <QTcpSocket>
#include <QJsonObject>
#include <QJsonDocument>
#include <QTimer>
#include <memory>

class HexapodConnection : public QObject
{
    Q_OBJECT

public:
    explicit HexapodConnection(QObject *parent = nullptr);
    ~HexapodConnection();

    bool connectToHost(const QString &hostname, int port);
    void disconnect();
    bool isConnected() const;

    // Command interface mirroring hexapod.hpp functionality
    bool setLegPosition(uint8_t legNum, int16_t hip, int16_t knee, int16_t ankle);
    bool centerAllLegs();
    bool setCalibration(uint8_t legNum, int16_t hipOffset, int16_t kneeOffset, int16_t ankleOffset);

public slots:
    void sendCommand(const QJsonObject &command);
    void requestImuData();

signals:
    void connectionStatusChanged(bool connected);
    void responseReceived(const QJsonObject &response);
    void imuDataReceived(double accelX, double accelY, double accelZ, double gyroX, double gyroY, double gyroZ);
    void errorOccurred(const QString &errorMessage);

private slots:
    void handleSocketConnected();
    void handleSocketDisconnected();
    void handleSocketError(QAbstractSocket::SocketError socketError);
    void handleSocketReadyRead();
    void checkConnection();

private:
    QTcpSocket *m_socket;
    QTimer m_connectionCheckTimer;
    QString m_hostname;
    int m_port;
    QByteArray m_buffer;

    QJsonObject createCommandObject(const QString &command);
    void processIncomingData();
};

#endif // HEXAPODCONNECTION_H
