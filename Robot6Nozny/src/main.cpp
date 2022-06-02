// Biblioteki
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Pliki nagłówkowe
#include "pinout.h"
#include "robot_leg.h"
#include "prox_sensor.h"
#include "accelerometer.h"
#include "ble_setup.h"

// ---- RTOS tasks
TaskHandle_t MeasureTask;
TaskHandle_t BLETask;
void measureTaskLoop(void *parameter);
void bleTaskLoop(void *parameter);
static SemaphoreHandle_t mutex_sensor;
// ----

// ---- Robot legs
RobotLeg leg1(PWM_1, PWM_2);
/*
RobotLeg leg2(PWM_1, PWM_2);
RobotLeg leg3(PWM_1, PWM_2);
RobotLeg leg4(PWM_1, PWM_2);
RobotLeg leg5(PWM_1, PWM_2);
RobotLeg leg6(PWM_1, PWM_2);
*/
// ----
// ---- MPU6050 Accelerometer and gyroscoper
Accelgyro acgr1;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
float euler[3];
volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}
// ----
// ---- PROXY SENSORS
struct proxValues
{
  uint8_t front;
  uint8_t back;
};
volatile proxValues prox;
// ----
// ---- BLE setup
BLECharacteristic proxSensorReadingCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor proxSensorReadingDescriptor(BLEUUID((uint16_t)0x2902));
BLECharacteristic *pOutputChar;
unsigned long lastTime = 0;
unsigned long timerDelay = 500;
bool deviceConnected = false;
double odleglosc;
static uint8_t sensorFlag;
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
    case 0x02:
    {
      xSemaphoreTake(mutex_sensor, portMAX_DELAY);
      sensorFlag = 0;
      xSemaphoreGive(mutex_sensor);
      break;
    }
    case 0x01:
    {
      xSemaphoreTake(mutex_sensor, portMAX_DELAY);
      sensorFlag = 1;
      xSemaphoreGive(mutex_sensor);
      break;
    }
    }
  }
};
// ----
void setup()
{
  Serial.begin(115200);
  // ---- Robots legs init
  leg1.init();
  // ----
  // ---- Proximity sensor init
  setupProxFront();
  setupProxBack();
  sensorFlag = 0;
  // ----
  // ---- BLE server init
  // Start serial communication
  Serial.begin(9600);
  // Create the BLE Device
  BLEDevice::init(PERIPHERAL_NAME);
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *proxService = pServer->createService(SERVICE_UUID);
  // Create BLE characteristic
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
  // ----
  // ---- Accelerometer and gyroscope init
  acgr1 = Accelgyro(-76, -2359, 1688, 220, 76, -85);
  Wire.begin();
  Serial.println(acgr1.isConnected() ? "MPU6050 connection succesful" : "MPU6050 connection failed");
  acgr1.init();
  devStatus = acgr1.getMPU().dmpInitialize();
  if (devStatus == 0)
  {
    acgr1.getMPU().CalibrateAccel(6);
    acgr1.getMPU().CalibrateGyro(6);
    acgr1.getMPU().PrintActiveOffsets();
    acgr1.getMPU().setDMPEnabled(true);
    dmpReady = true;
    packetSize = acgr1.getMPU().dmpGetFIFOPacketSize();
  }
  else
  {
    Serial.print("DMP Initialization failed");
    Serial.print(devStatus);
  }
  // ----
  // ---- RTOS tasks init
  mutex_sensor = xSemaphoreCreateMutex();
  xSemaphoreTake(mutex_sensor, portMAX_DELAY);
  xTaskCreatePinnedToCore(measureTaskLoop, "MeasureTask", 1000, NULL, 1, &MeasureTask, 0);
  xTaskCreatePinnedToCore(bleTaskLoop, "BLETask", 10000, NULL, 1, &BLETask, 0);
  xSemaphoreGive(mutex_sensor);
  //  ----
  delay(1000);
}

void measureTaskLoop(void *parameter)
{
  for (;;)
  {
    // ACCELEROMETER
    /*
    acgr1.update();
    if (!dmpReady)
      return;

    if (acgr1.getMPU().dmpGetCurrentFIFOPacket(fifoBuffer))
    {
      acgr1.getMPU().dmpGetQuaternion(&q, fifoBuffer);
      acgr1.getMPU().dmpGetEuler(euler, &q);
      Serial.print("Euler angles:\n");
      Serial.print("X: ");
      Serial.print(euler[0] + 180 / M_PI);
      Serial.print("\tY: ");
      Serial.print(euler[1] + 180 / M_PI);
      Serial.print("\tZ: \n");
      Serial.print(euler[2] + 180 / M_PI);
    }*/
    // PROXY SENSORS
    prox.front = zmierzOdlegloscFront();
    prox.back = zmierzOdlegloscBack();
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}
void bleTaskLoop(void *parameter)
{
  for (;;)
  {
    if (deviceConnected)
    {
      // if ((millis() - lastTime) > timerDelay)
      //{
      static char proxReading[6];
      xSemaphoreTake(mutex_sensor, portMAX_DELAY);
      uint8_t sensor = sensorFlag;
      xSemaphoreGive(mutex_sensor);
      if (sensor)
        dtostrf(prox.front, 3, 2, proxReading);
      else
        dtostrf(prox.back, 3, 2, proxReading);

      proxSensorReadingCharacteristics.setValue(proxReading);
      proxSensorReadingCharacteristics.notify();
      Serial.print("Odleglosc ");
      if (sensor)
      {
        Serial.print("front: ");
        Serial.println(prox.front);
      }
      else
      {
        Serial.print("back: ");
        Serial.println(prox.back);
      }
      //}
      // lastTime = millis();
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void loop()
{
  Serial.print("Sensor flag: ");
  xSemaphoreTake(mutex_sensor, portMAX_DELAY);
  Serial.println(sensorFlag);
  xSemaphoreGive(mutex_sensor);
  vTaskDelay(500 / portTICK_PERIOD_MS);
  // LEGS
  // leg1.moveUp();
  // delay(2000);
  // leg1.moveDown();
  // delay(2000);
}