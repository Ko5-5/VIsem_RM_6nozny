#include <Arduino.h>
#include "accelerometer.h"

Accelgyro::Accelgyro(int16_t _axoff, int16_t _ayoff, int16_t _azoff, int16_t _gxoff, int16_t _gyoff, int16_t _gzoff)
    :axoff(_axoff),
    ayoff(_ayoff),
    azoff(_azoff),
    gxoff(_gxoff),
    gyoff(_gyoff),
    gzoff(_gzoff)
{}

void Accelgyro::init(){
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
