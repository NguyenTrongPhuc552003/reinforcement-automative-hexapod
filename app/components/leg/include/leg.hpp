#pragma once

#include <array>

class Leg {
public:
    Leg(int leg_id, int coxa_channel, int femur_channel, int tibia_channel);

    // Delete default constructor
    Leg() = delete;

    // Add move constructor
    Leg(Leg&&) = default;
    Leg& operator=(Leg&&) = default;

    // Delete copy constructor/assignment
    Leg(const Leg&) = delete;
    Leg& operator=(const Leg&) = delete;

    // Đặt vị trí mục tiêu (đơn vị: mm)
    void setTargetPosition(float x, float y, float z);

    // Gọi mỗi chu kỳ để cập nhật góc servo tương ứng
    void update();

    // Lấy góc servo hiện tại (debug)
    std::array<float, 3> getServoAngles() const;

private:
    int leg_id;
    int coxa_channel;
    int femur_channel;
    int tibia_channel;

    float target_x;
    float target_y;
    float target_z;

    float coxa_angle;
    float femur_angle;
    float tibia_angle;

    // Hàm nội bộ: inverse kinematics để tính góc servo từ tọa độ
    void computeIK();
};
