#include <Arduino.h>
#include <pgmspace.h>
#include "BluetoothSerial.h"
#include "robot_leg.h"
#include "accelerometer.h"

// required if using I2CDEV_ARDUINO_WIRE is used in I2Cdev.h
#if (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE) && !defined (PARTICLE)
  #include "Wire.h"
#endif

#define OUTPUT_READABLE_ACCELGYRO   // using readable output
// #define OUTPUT_BINARY_ACCELGYRO    // using unreadable output, easy to parse

//BluetoothSerial SerialBT;
ESP32PWM pwm;
Accelgyro acgr1;


void setup() {
  acgr1 = Accelgyro(-76, -2359, 1688, 220, 76, -85);
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Initializing I2C devices...");
  Serial.println("Testing connections...");
  Serial.println(acgr1.isConnected() ? "MPU6050 connection succesful" : "MPU6050 connection failed");
  acgr1.init();
  //SerialBT.begin("ESP32test");
  delay(1000);
  // Allow allocation of all timers
  //ESP32PWM::allocateTimer(0);
	//ESP32PWM::allocateTimer(1);
	//ESP32PWM::allocateTimer(2);
	//ESP32PWM::allocateTimer(3);
}

void loop() 
{
  acgr1.update();
  Serial.print("A::\tX:");
  Serial.print(acgr1.getAccX());
  Serial.print("\tY:");
  Serial.print(acgr1.getAccY());
  Serial.print("\tZ:");
  Serial.print(acgr1.getAccZ());
  Serial.print("\t\tG::\tX:");
  Serial.print(acgr1.getRotX());
  Serial.print("\tY:");
  Serial.print(acgr1.getRotY());
  Serial.print("\tZ:");
  Serial.print(acgr1.getRotZ());
  Serial.print("\n");
  delay(500);
}