#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>

// class default I2C adress is 0x68

class Accelgyro
{
    private:
    
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;

    MPU6050 accgyr;

    public:
    Accelgyro();
    Accelgyro(int16_t axoff, int16_t ayoff, int16_t azoff, int16_t gxoff, int16_t gyoff, int16_t gzoff);
    void update();
    bool isConnected() {return accgyr.testConnection();}
    int16_t getAccX() {return ax;}
    int16_t getAccY() {return ay;}
    int16_t getAccZ() {return az;}
    int16_t getRotX() {return gx;}
    int16_t getRotY() {return gy;}
    int16_t getRotZ() {return gz;}
}



#endif