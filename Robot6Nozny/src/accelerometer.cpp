#include <Arduino.h>
#include "accelerometer.h"

Accelgyro::Accelgyro(int16_t axoff, int16_t ayoff, int16_t azoff, int16_t gxoff, int16_t gyoff, int16_t gzoff)
{
    accgyr.initialize();

    accgyr.setXAccelOffset(axoff);
    accgyr.setYAccelOffset(ayoff);
    accgyr.setZAccelOffset(azoff);
    accgyr.setXGyroOffset(gxoff);
    accgyr.setYGyroOffset(gyoff);
    accgyr.setZGyroOffset(gzoff);
}

void Accelgyro::update()
{
    accgyr.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}
