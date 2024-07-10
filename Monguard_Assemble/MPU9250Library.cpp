#include "MPU9250Library.h"

MPU9250Library::MPU9250Library(uint8_t address, float threshold) 
    : mpu(), lastX(0), threshold(threshold), thresholdExceeded(false) {}

bool MPU9250Library::begin() {
    Wire.begin();
    MPU9250Setting setting;
    if (!mpu.setup(0x68, setting, Wire)) {
        return false;
    }
    return true;
}

void MPU9250Library::update() {
    if (mpu.update()) {
        float currentX = mpu.getAccX();
        float deltaX = abs(currentX - lastX);
        thresholdExceeded = (deltaX > threshold);
        lastX = currentX;
    }
}

bool MPU9250Library::isThresholdExceeded() {
    return thresholdExceeded;
}
