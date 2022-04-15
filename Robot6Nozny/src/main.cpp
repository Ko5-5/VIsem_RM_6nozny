#include <Arduino.h>
#include "BluetoothSerial.h"
#include "robot_leg.h"

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