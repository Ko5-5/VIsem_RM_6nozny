#include <Arduino.h>
#include "BluetoothSerial.h"
#include "robot_leg.h"
#include "accelerometer.h"

// required if using I2CDEV_ARDUINO_WIRE is used in I2Cdev.h
#if (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE) && !defined (PARTICLE)
  #include "Wire.h"
#endif

#define OUTPUT_READABLE_ACCELGYRO   // using readable output
// #define OUTPUT_BINARY_ACCELGYRO    // using unreadable output, easy to parse


BluetoothSerial SerialBT;
ESP32PWM pwm;

void setup() {
  SerialBT.begin("ESP32test");
  delay(1000);
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
}

void loop() {

}
