#include "leg.hpp"
#include "servo_controller.hpp"
#include <cmath>
#include <cstdio>

// Các thông số vật lý cơ bản của một chân (đơn vị: mm)
constexpr float COXA_LENGTH = 25.0f;
constexpr float FEMUR_LENGTH = 50.0f;
constexpr float TIBIA_LENGTH = 75.0f;

// Servo offsets and directions
constexpr float COXA_OFFSET = 90.0f;   // Center position offset
constexpr float FEMUR_OFFSET = 90.0f;  
constexpr float TIBIA_OFFSET = 90.0f;
constexpr int COXA_DIR = 1;    // 1 or -1 to reverse direction
constexpr int FEMUR_DIR = 1;
constexpr int TIBIA_DIR = -1;  // Reversed for natural motion

Leg::Leg(int leg_id, int coxa_channel, int femur_channel, int tibia_channel)
    : leg_id(leg_id),
      coxa_channel(coxa_channel),
      femur_channel(femur_channel),
      tibia_channel(tibia_channel),
      target_x(0), target_y(0), target_z(0),
      coxa_angle(0), femur_angle(0), tibia_angle(0)
{}

void Leg::setTargetPosition(float x, float y, float z) {
    target_x = x;
    target_y = y;
    target_z = z;
    computeIK();
}

void Leg::computeIK() {
    // Góc Coxa (xoay chân qua trái/phải)
    coxa_angle = atan2(target_y, target_x) * 180.0f / M_PI;

    // Chiều dài chiếu từ Coxa đến điểm chân cần đến (sau khi loại bỏ xoay ngang)
    float horizontal_dist = sqrt(target_x * target_x + target_y * target_y) - COXA_LENGTH;
    float vertical_dist = -target_z;  // Z hướng xuống dưới

    float dist = sqrt(horizontal_dist * horizontal_dist + vertical_dist * vertical_dist);

    // Sử dụng định lý cosine để tính các góc
    float a1 = atan2(vertical_dist, horizontal_dist);
    float a2 = acos((FEMUR_LENGTH * FEMUR_LENGTH + dist * dist - TIBIA_LENGTH * TIBIA_LENGTH) /
                    (2 * FEMUR_LENGTH * dist));

    femur_angle = (a1 + a2) * 180.0f / M_PI;

    tibia_angle = acos((FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - dist * dist) /
                       (2 * FEMUR_LENGTH * TIBIA_LENGTH)) * 180.0f / M_PI;

    // Đổi tibia_angle về dạng servo (có thể cần - hoặc offset)
    tibia_angle = 180.0f - tibia_angle;
}

void Leg::update() {
    // Áp dụng offset và hệ số điều chỉnh chiều
    float coxa_pwm = COXA_OFFSET + (COXA_DIR * coxa_angle);
    float femur_pwm = FEMUR_OFFSET + (FEMUR_DIR * femur_angle);
    float tibia_pwm = TIBIA_OFFSET + (TIBIA_DIR * tibia_angle);

    // Giới hạn góc trong khoảng hợp lệ
    coxa_pwm = std::max(0.0f, std::min(180.0f, coxa_pwm));
    femur_pwm = std::max(0.0f, std::min(180.0f, femur_pwm));
    tibia_pwm = std::max(0.0f, std::min(180.0f, tibia_pwm));

    // Cập nhật góc cho servo
    ServoController::setAngle(coxa_channel, coxa_pwm);
    ServoController::setAngle(femur_channel, femur_pwm);
    ServoController::setAngle(tibia_channel, tibia_pwm);

    printf("Leg %d angles: Coxa=%.2f, Femur=%.2f, Tibia=%.2f\n",
           leg_id, coxa_angle, femur_angle, tibia_angle);
}

std::array<float, 3> Leg::getServoAngles() const {
    return {coxa_angle, femur_angle, tibia_angle};
}
