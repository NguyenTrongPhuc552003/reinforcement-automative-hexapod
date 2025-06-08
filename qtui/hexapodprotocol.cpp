#include "hexapodprotocol.h"

namespace HexapodProtocol
{

    // Helper function to create a base command object
    QJsonObject createBaseCommand(const QString &commandName)
    {
        QJsonObject cmd;
        cmd["command"] = commandName;
        cmd["timestamp"] = QDateTime::currentMSecsSinceEpoch();
        return cmd;
    }

    QJsonObject createSetLegPositionCommand(uint8_t legNum, int16_t hip, int16_t knee, int16_t ankle)
    {
        QJsonObject cmd = createBaseCommand("setLegPosition");
        cmd["leg_num"] = static_cast<int>(legNum);
        cmd["hip"] = hip;
        cmd["knee"] = knee;
        cmd["ankle"] = ankle;
        return cmd;
    }

    QJsonObject createGetLegPositionCommand(uint8_t legNum)
    {
        QJsonObject cmd = createBaseCommand("getLegPosition");
        cmd["leg_num"] = static_cast<int>(legNum);
        return cmd;
    }

    QJsonObject createCenterAllCommand()
    {
        return createBaseCommand("centerAll");
    }

    QJsonObject createGetImuDataCommand()
    {
        return createBaseCommand("getImuData");
    }

    QJsonObject createSetCalibrationCommand(uint8_t legNum, int16_t hipOffset, int16_t kneeOffset, int16_t ankleOffset)
    {
        QJsonObject cmd = createBaseCommand("setCalibration");
        cmd["leg_num"] = static_cast<int>(legNum);
        cmd["hip_offset"] = hipOffset;
        cmd["knee_offset"] = kneeOffset;
        cmd["ankle_offset"] = ankleOffset;
        return cmd;
    }

    QJsonObject createSetMovementCommand(double direction, double speed)
    {
        QJsonObject cmd = createBaseCommand("setMovement");
        cmd["direction"] = direction;
        cmd["speed"] = speed;
        return cmd;
    }

    QJsonObject createSetRotationCommand(double direction, double speed)
    {
        QJsonObject cmd = createBaseCommand("setRotation");
        cmd["direction"] = direction;
        cmd["speed"] = speed;
        return cmd;
    }

    QJsonObject createSetTiltCommand(double pitch, double roll)
    {
        QJsonObject cmd = createBaseCommand("setTilt");
        cmd["pitch"] = pitch;
        cmd["roll"] = roll;
        return cmd;
    }

    QJsonObject createSetSpeedCommand(double speed)
    {
        QJsonObject cmd = createBaseCommand("setSpeed");
        cmd["speed"] = speed;
        return cmd;
    }

    QJsonObject createSetBalanceEnabledCommand(bool enabled)
    {
        QJsonObject cmd = createBaseCommand("setBalanceEnabled");
        cmd["enabled"] = enabled;
        return cmd;
    }

    QJsonObject createStopCommand()
    {
        return createBaseCommand("stop");
    }

    QJsonObject createPingCommand()
    {
        return createBaseCommand("ping");
    }

    bool isSuccessResponse(const QJsonObject &response)
    {
        return response.contains("status") && response["status"].toString() == "success";
    }

    bool isErrorResponse(const QJsonObject &response)
    {
        return response.contains("status") && response["status"].toString() == "error";
    }

    QString getErrorMessage(const QJsonObject &response)
    {
        if (response.contains("message"))
        {
            return response["message"].toString();
        }
        return "Unknown error";
    }

    ImuData parseImuDataResponse(const QJsonObject &response)
    {
        if (response.contains("imu_data") && response["imu_data"].isObject())
        {
            return ImuData::fromJson(response["imu_data"].toObject());
        }
        // Initialize all struct members
        ImuData empty = {0, 0, 0, 0, 0, 0};
        return empty;
    }

    LegPosition parseLegPositionResponse(const QJsonObject &response)
    {
        if (response.contains("leg_position") && response["leg_position"].isObject())
        {
            return LegPosition::fromJson(response["leg_position"].toObject());
        }
        // Initialize all struct members
        LegPosition empty = {0, 0, 0, 0};
        return empty;
    }

} // namespace HexapodProtocol
