// Biblioteki
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Pliki nagłówkowe
#include "pinout.h"
#include "robot_leg.h"
#include "prox_sensor.h"

/*
RobotLeg leg1(PWM_1, PWM_2);
RobotLeg leg2(PWM_1, PWM_2);
RobotLeg leg3(PWM_1, PWM_2);
RobotLeg leg4(PWM_1, PWM_2);
RobotLeg leg5(PWM_1, PWM_2);
RobotLeg leg6(PWM_1, PWM_2);
*/

#define PERIPHERAL_NAME "Test Peripheral"
#define SERVICE_UUID "d049fa8a-d907-11ec-9d64-0242ac120002"
#define CHARACTERISTIC_INPUT_UUID "9c6f413e-d90c-11ec-9d64-0242ac120002"

BLECharacteristic proxSensorReadingCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor proxSensorReadingDescriptor(BLEUUID((uint16_t)0x2902));

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 500;

bool deviceConnected = false;

double odleglosc;
uint8_t sensorFlag = 0;

BLECharacteristic *pOutputChar;

// Setup callbacks onConnect and onDisconnect
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("BLE Client Connected");
  };
  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("BLE Client Disconnected");
  }
};

class InputReceivedCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharWriteState)
  {
    uint8_t *inputValue = pCharWriteState->getData();

    switch (*inputValue)
    {
    case 0x00:
      sensorFlag = 0;
      break;
    case 0x01:
      sensorFlag = 1;
      break;
    }
  }
};

void setup()
{
  // Start serial communication
  Serial.begin(9600);

  // Create the BLE Device
  BLEDevice::init(PERIPHERAL_NAME);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *proxService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pInputChar = proxService->createCharacteristic(
      CHARACTERISTIC_INPUT_UUID,
      BLECharacteristic::PROPERTY_WRITE_NR |
          BLECharacteristic::PROPERTY_WRITE);

  pInputChar->setCallbacks(new InputReceivedCallbacks());

  proxService->addCharacteristic(&proxSensorReadingCharacteristics);
  proxSensorReadingDescriptor.setValue("Proximity Sensor Reading");
  proxSensorReadingCharacteristics.addDescriptor(&proxSensorReadingDescriptor);

  // Start the service
  proxService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  setupProxFront();
  setupProxBack();
  delay(1000);
}

void loop()
{
  if (deviceConnected)
  {
    if ((millis() - lastTime) > timerDelay)
    {
      if (sensorFlag)
        odleglosc = zmierzOdlegloscFront();
      else
        odleglosc = zmierzOdlegloscBack();
      static char proxReading[6];
      dtostrf(odleglosc, 3, 2, proxReading);
      proxSensorReadingCharacteristics.setValue(proxReading);
      proxSensorReadingCharacteristics.notify();
      Serial.print("Odleglosc ");
      if (sensorFlag)
        Serial.print("front: ");
      else
        Serial.print("back: ");
      Serial.println(odleglosc);
      lastTime = millis();
    }
  }
}