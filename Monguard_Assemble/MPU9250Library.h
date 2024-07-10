#ifndef MPU9250LIBRARY_H
#define MPU9250LIBRARY_H

#include <Arduino.h>  
#include <Wire.h>
#include <MPU9250.h>
#include <math.h>  // mpu9250 값 계산을 위해 추가

class MPU9250Library {
public:
    MPU9250Library(uint8_t address = 0x68, float threshold = 0.5);

    bool begin();
    void update();
    bool isThresholdExceeded();

private:
    MPU9250 mpu;
    float lastX;
    float threshold;
    bool thresholdExceeded;
};

#endif
