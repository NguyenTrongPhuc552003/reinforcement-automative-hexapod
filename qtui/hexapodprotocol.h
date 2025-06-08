#ifndef HEXAPODPROTOCOL_H
#define HEXAPODPROTOCOL_H

#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>

// Command definitions to match the hexapod.hpp functionality
namespace HexapodProtocol
{

    enum CommandType
    {
        SetLegPosition,
        GetLegPosition,
        CenterAll,
        GetImuData,
        SetCalibration,
        SetMovement,
        SetRotation,
        SetTilt,
        SetSpeed,
        SetBalanceEnabled,
        Stop,
        Ping
    };

    struct ImuData
    {
        double accelX;
        double accelY;
        double accelZ;
        double gyroX;
        double gyroY;
        double gyroZ;

        QJsonObject toJson() const
        {
            QJsonObject obj;
            obj["accel_x"] = accelX;
            obj["accel_y"] = accelY;
            obj["accel_z"] = accelZ;
            obj["gyro_x"] = gyroX;
            obj["gyro_y"] = gyroY;
            obj["gyro_z"] = gyroZ;
            return obj;
        }

        static ImuData fromJson(const QJsonObject &obj)
        {
            ImuData data;
            data.accelX = obj["accel_x"].toDouble();
            data.accelY = obj["accel_y"].toDouble();
            data.accelZ = obj["accel_z"].toDouble();
            data.gyroX = obj["gyro_x"].toDouble();
            data.gyroY = obj["gyro_y"].toDouble();
            data.gyroZ = obj["gyro_z"].toDouble();
            return data;
        }
    };

    struct LegPosition
    {
        uint8_t legNum;
        int16_t hip;
        int16_t knee;
        int16_t ankle;

        QJsonObject toJson() const
        {
            QJsonObject obj;
            obj["leg_num"] = legNum;
            obj["hip"] = hip;
            obj["knee"] = knee;
            obj["ankle"] = ankle;
            return obj;
        }

        static LegPosition fromJson(const QJsonObject &obj)
        {
            LegPosition pos;
            pos.legNum = static_cast<uint8_t>(obj["leg_num"].toInt());
            pos.hip = static_cast<int16_t>(obj["hip"].toInt());
            pos.knee = static_cast<int16_t>(obj["knee"].toInt());
            pos.ankle = static_cast<int16_t>(obj["ankle"].toInt());
            return pos;
        }
    };

    // Helper functions to create command objects
    QJsonObject createBaseCommand(const QString &commandName);
    QJsonObject createSetLegPositionCommand(uint8_t legNum, int16_t hip, int16_t knee, int16_t ankle);
    QJsonObject createGetLegPositionCommand(uint8_t legNum);
    QJsonObject createCenterAllCommand();
    QJsonObject createGetImuDataCommand();
    QJsonObject createSetCalibrationCommand(uint8_t legNum, int16_t hipOffset, int16_t kneeOffset, int16_t ankleOffset);
    QJsonObject createSetMovementCommand(double direction, double speed);
    QJsonObject createSetRotationCommand(double direction, double speed);
    QJsonObject createSetTiltCommand(double pitch, double roll);
    QJsonObject createSetSpeedCommand(double speed);
    QJsonObject createSetBalanceEnabledCommand(bool enabled);
    QJsonObject createStopCommand();
    QJsonObject createPingCommand();

    // Helper functions to parse responses
    bool isSuccessResponse(const QJsonObject &response);
    bool isErrorResponse(const QJsonObject &response);
    QString getErrorMessage(const QJsonObject &response);
    ImuData parseImuDataResponse(const QJsonObject &response);
    LegPosition parseLegPositionResponse(const QJsonObject &response);

} // namespace HexapodProtocol

#endif // HEXAPODPROTOCOL_H
