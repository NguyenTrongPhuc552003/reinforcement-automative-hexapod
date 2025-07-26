#ifndef PCA9685_HPP
#define PCA9685_HPP

#include <cstdint>

class PCA9685
{
public:
    static const uint8_t ADDR_1 = 0x40;
    static const uint8_t ADDR_2 = 0x41;
    static const int I2C_BUS = 2;

    PCA9685(uint8_t address);
    ~PCA9685();

    bool init();
    void setFreq(float freq);
    void setPWM(uint8_t channel, uint16_t on, uint16_t off);
    void setServoAngle(uint8_t channel, float angle);

private:
    uint8_t addr;
    int fd;

    bool writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);
};

#endif
