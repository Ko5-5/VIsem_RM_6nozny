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
#include "expander.h"

// ---- RTOS tasks
TaskHandle_t MeasureTask;
TaskHandle_t BLETask;
void measureTaskLoop(void *parameter);
void bleTaskLoop(void *parameter);
static SemaphoreHandle_t mutex_sensor;
static SemaphoreHandle_t mutex_prox;
static SemaphoreHandle_t mutex_leg;
static SemaphoreHandle_t mutex_exp;
// ----
// ---- Robot legs
RobotLeg leg1(PWM_1, PWM_2, 100);
RobotLeg leg2(PWM_3, PWM_4);
RobotLeg leg3(PWM_5, PWM_6, 80);
RobotLeg leg4(PWM_7, PWM_8, 80);
RobotLeg leg5(PWM_9, PWM_10);
RobotLeg leg6(PWM_11, PWM_12, 100);
static uint8_t legFlag;
// ----
// ---- Exapander - leg limit switches
Expander exp_legs;
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
// BLECharacteristic expReadingCharacteristics("ae05843c-919b-4d86-b37c-b31c7eb43320", BLECharacteristic::PROPERTY_NOTIFY);
// BLEDescriptor expReadingDescriptor(BLEUUID((uint16_t)0x1902));
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

    if (*inputValue < (uint8_t)0x10)
    {
      if (xSemaphoreTake(mutex_sensor, portMAX_DELAY) == pdTRUE)
      {
        sensorFlag = *inputValue == 0x00 ? 0 : 1;
      }
      xSemaphoreGive(mutex_sensor);
    }
    else
    {
      if (xSemaphoreTake(mutex_leg, portMAX_DELAY) == pdTRUE)
      {
        legFlag = *inputValue;
        xSemaphoreGive(mutex_leg);
      }
    }
  }
};
// ----
void setup()
{
  Serial.begin(115200);
  // ---- Proximity sensor init
  setupProxFront();
  setupProxBack();
  sensorFlag = 0;
  // ---- Expander
  exp_legs.setup();
  exp_legs.updateButtons();
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
  BLEService *proxService = pServer->createService(SERVICE_PROX_UUID);
  // BLEService *expService = pServer->createService(SERVICE_EXP_UUID);
  //  Create BLE characteristic
  BLECharacteristic *pInputChar = proxService->createCharacteristic(
      CHARACTERISTIC_INPUT_UUID,
      BLECharacteristic::PROPERTY_WRITE_NR |
          BLECharacteristic::PROPERTY_WRITE);

  pInputChar->setCallbacks(new InputReceivedCallbacks());
  proxService->addCharacteristic(&proxSensorReadingCharacteristics);
  proxSensorReadingDescriptor.setValue("Proximity Sensor Reading");
  proxSensorReadingCharacteristics.addDescriptor(&proxSensorReadingDescriptor);
  // expService->addCharacteristic(&expReadingCharacteristics);
  // expReadingDescriptor.setValue("Expander Reading");
  // expReadingCharacteristics.addDescriptor(&expReadingDescriptor);
  //  Start the service
  proxService->start();
  // expService->start();
  //  Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_PROX_UUID);
  // pAdvertising->addServiceUUID(SERVICE_EXP_UUID);
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
  mutex_prox = xSemaphoreCreateMutex();
  mutex_leg = xSemaphoreCreateMutex();
  mutex_exp = xSemaphoreCreateMutex();
  xSemaphoreTake(mutex_sensor, portMAX_DELAY);
  xSemaphoreTake(mutex_prox, portMAX_DELAY);
  xSemaphoreTake(mutex_exp, portMAX_DELAY);
  xTaskCreatePinnedToCore(measureTaskLoop, "MeasureTask", 1000, NULL, 1, &MeasureTask, 0);
  xTaskCreatePinnedToCore(bleTaskLoop, "BLETask", 10000, NULL, 1, &BLETask, 0);
  xSemaphoreGive(mutex_sensor);
  xSemaphoreGive(mutex_prox);
  xSemaphoreGive(mutex_exp);
  //   ----
  //  ---- Robots legs init
  leg1.init();
  leg2.halfInit();
  leg3.init();
  leg4.init();
  leg5.halfInit();
  leg6.init();
  legFlag = 0;
  //  ----
  vTaskDelay(1000 / portTICK_PERIOD_MS);
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
    if (xSemaphoreTake(mutex_prox, portMAX_DELAY) == pdTRUE)
    {
      prox.front = zmierzOdlegloscFront();
      prox.back = zmierzOdlegloscBack();
      xSemaphoreGive(mutex_prox);
    }
    // LEG TOUCH

    if (xSemaphoreTake(mutex_exp, portMAX_DELAY) == pdTRUE)
    {
      exp_legs.updateButtons();
      Serial.print("Buttons: ");
      Serial.print(exp_legs.buttons[0]);
      Serial.print(" : ");
      Serial.print(exp_legs.buttons[1]);
      Serial.print(" : ");
      Serial.print(exp_legs.buttons[2]);
      Serial.print(" : ");
      Serial.print(exp_legs.buttons[3]);
      Serial.print(" : ");
      Serial.print(exp_legs.buttons[4]);
      Serial.print(" : ");
      Serial.println(exp_legs.buttons[5]);
      xSemaphoreGive(mutex_exp);
    }
    // ----
    vTaskDelay(200 / portTICK_PERIOD_MS);
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
      // static char expReading[6];
      static uint8_t sensor = 0;
      if (xSemaphoreTake(mutex_sensor, portMAX_DELAY) == pdTRUE)
      {
        sensor = sensorFlag;
        xSemaphoreGive(mutex_sensor);
      }
      if (sensor)
        dtostrf(prox.front, 3, 2, proxReading);
      else
        dtostrf(prox.back, 3, 2, proxReading);
      /*if (xSemaphoreTake(mutex_exp, portMAX_DELAY) == pdTRUE)
      {
        expReading[0] = exp_legs.buttons[0];
        expReading[1] = exp_legs.buttons[1];
        expReading[2] = exp_legs.buttons[2];
        expReading[3] = exp_legs.buttons[3];
        expReading[4] = exp_legs.buttons[4];
        expReading[5] = exp_legs.buttons[5];
        xSemaphoreGive(mutex_exp);
      }*/
      proxSensorReadingCharacteristics.setValue(proxReading);
      // expReadingCharacteristics.setValue(expReading);
      proxSensorReadingCharacteristics.notify();
      // expReadingCharacteristics.notify();

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
  /*if (xSemaphoreTake(mutex_sensor, portMAX_DELAY) == pdTRUE)
  {
    Serial.print("Sensor flag: ");
    Serial.println(sensorFlag);
    xSemaphoreGive(mutex_sensor);
  }*/

  if (xSemaphoreTake(mutex_leg, portMAX_DELAY) == pdTRUE)
  {
    switch (legFlag)
    {
    case 0x10:
    {
      Serial.println("Servo calib 1");
      leg1.moveCenter();
      legFlag = 0;
      break;
    }
    case 0x11:
    {
      Serial.println("Servo calib 1");
      leg1.moveCenter();
      leg1.moveDown();
      legFlag = 0;
      break;
    }
    case 0x12:
    {
      Serial.println("Servo calib 1");
      leg1.moveCenter();
      leg1.moveUp();
      legFlag = 0;
      break;
    }
    case 0x13:
    {
      Serial.println("Servo calib 1");
      leg1.moveFront();
      legFlag = 0;
      break;
    }
    case 0x14:
    {
      Serial.println("Servo calib 1");
      leg1.moveBack();
      legFlag = 0;
      break;
    }
    case 0x20:
    {
      Serial.println("Servo calib 2");
      leg2.moveCenter();
      legFlag = 0;
      break;
    }
    case 0x21:
    {
      Serial.println("Servo calib 2");
      leg2.moveCenter();
      leg2.moveDown();
      legFlag = 0;
      break;
    }
    case 0x22:
    {
      Serial.println("Servo calib 2");
      leg2.moveCenter();
      leg2.moveUp();
      legFlag = 0;
      break;
    }
    case 0x23:
    {
      Serial.println("Servo calib 2");
      leg2.moveFront();
      legFlag = 0;
      break;
    }
    case 0x24:
    {
      Serial.println("Servo calib 2");
      leg2.moveBack();
      legFlag = 0;
      break;
    }
    case 0x30:
    {
      Serial.println("Servo calib 3");
      leg3.moveCenter();
      legFlag = 0;
      break;
    }
    case 0x31:
    {
      Serial.println("Servo calib 3");
      leg3.moveCenter();
      leg3.moveDown();
      legFlag = 0;
      break;
    }
    case 0x32:
    {
      Serial.println("Servo calib 3");
      leg3.moveCenter();
      leg3.moveUp();
      legFlag = 0;
      break;
    }
    case 0x33:
    {
      Serial.println("Servo calib 3");
      leg3.moveFront();
      legFlag = 0;
      break;
    }
    case 0x34:
    {
      Serial.println("Servo calib 3");
      leg3.moveBack();
      legFlag = 0;
      break;
    }
    case 0x40:
    {
      Serial.println("Servo calib 4");
      leg4.moveCenter();
      legFlag = 0;
      break;
    }
    case 0x41:
    {
      Serial.println("Servo calib 4");
      leg4.moveCenter();
      leg4.moveDown();
      legFlag = 0;
      break;
    }
    case 0x42:
    {
      Serial.println("Servo calib 4");
      leg4.moveCenter();
      leg4.moveUp();
      legFlag = 0;
      break;
    }
    case 0x43:
    {
      Serial.println("Servo calib 4");
      leg4.moveFront();
      legFlag = 0;
      break;
    }
    case 0x44:
    {
      Serial.println("Servo calib 4");
      leg4.moveBack();
      legFlag = 0;
      break;
    }
    case 0x50:
    {
      Serial.println("Servo calib 5");
      leg5.moveCenter();
      legFlag = 0;
      break;
    }
    case 0x51:
    {
      Serial.println("Servo calib 5");
      leg5.moveCenter();
      leg5.moveDown();
      legFlag = 0;
      break;
    }
    case 0x52:
    {
      Serial.println("Servo calib 5");
      leg5.moveCenter();
      leg5.moveUp();
      legFlag = 0;
      break;
    }
    case 0x53:
    {
      Serial.println("Servo calib 5");
      leg5.moveFront();
      legFlag = 0;
      break;
    }
    case 0x54:
    {
      Serial.println("Servo calib 5");
      leg5.moveBack();
      legFlag = 0;
      break;
    }
    case 0x60:
    {
      Serial.println("Servo calib 6");
      leg6.moveCenter();
      legFlag = 0;
      break;
    }
    case 0x61:
    {
      Serial.println("Servo calib 6");
      leg6.moveCenter();
      leg6.moveDown();
      legFlag = 0;
      break;
    }
    case 0x62:
    {
      Serial.println("Servo calib 6");
      leg6.moveCenter();
      leg6.moveUp();
      legFlag = 0;
      break;
    }
    case 0x63:
    {
      Serial.println("Servo calib 6");
      leg6.moveFront();
      legFlag = 0;
      break;
    }
    case 0x64:
    {
      Serial.println("Servo calib 6");
      leg6.moveBack();
      legFlag = 0;
      break;
    }
    case 0x80:
    {
      Serial.println("Set 1 Up");
      leg1.moveUp();
      leg3.moveUp();
      leg5.moveUp();
      legFlag = 0;
      break;
    }
    case 0x81:
    {
      Serial.println("Set 1 Down");
      leg1.moveDown();
      leg3.moveDown();
      leg5.moveDown();
      legFlag = 0;
      break;
    }
    case 0x82:
    {
      Serial.println("Set 2 Up");
      leg2.moveUp();
      leg4.moveUp();
      leg6.moveUp();
      legFlag = 0;
      break;
    }
    case 0x83:
    {
      Serial.println("Set 2 Down");
      leg2.moveDown();
      leg4.moveDown();
      leg6.moveDown();
      legFlag = 0;
      break;
    }
    case 0x84:
    {
      Serial.println("Set 1 Front");
      leg1.moveFront();
      leg3.moveFront();
      leg5.moveFront();
      legFlag = 0;
      break;
    }
    case 0x85:
    {
      Serial.println("Set 1 Center");
      leg1.moveCenter();
      leg3.moveCenter();
      leg5.moveCenter();
      legFlag = 0;
      break;
    }
    case 0x86:
    {
      Serial.println("Set 1 Back");
      leg1.moveBack();
      leg3.moveBack();
      leg5.moveBack();
      legFlag = 0;
      break;
    }
    case 0x87:
    {
      Serial.println("Set 2 Front");
      leg2.moveBack();
      leg4.moveBack();
      leg6.moveBack();
      legFlag = 0;
      break;
    }
    case 0x88:
    {
      Serial.println("Set 2 Center");
      leg2.moveCenter();
      leg4.moveCenter();
      leg6.moveCenter();
      legFlag = 0;
      break;
    }
    case 0x89:
    {
      Serial.println("Set 2 Back");
      leg2.moveFront();
      leg4.moveFront();
      leg6.moveFront();
      legFlag = 0;
      break;
    }
    case 0x90:
    {
      Serial.println("Center all");
      leg1.moveCenter();
      // leg2.moveCenter();
      leg3.moveCenter();
      leg4.moveCenter();
      // leg5.moveCenter();
      leg6.moveCenter();
      break;
    }
    case 0x91:
    {
      if (xSemaphoreTake(mutex_prox, portMAX_DELAY) == pdTRUE)
      {
        if (prox.back > 10)
        {
          xSemaphoreGive(mutex_prox);
          Serial.println("A step forward");
          leg1.moveUp();
          leg3.moveUp();
          leg5.moveUp();
          vTaskDelay(300 / portTICK_PERIOD_MS);
          leg1.moveFront();
          leg3.moveFront();
          leg5.moveFront();
          vTaskDelay(300 / portTICK_PERIOD_MS);
          leg1.moveDown();
          leg3.moveDown();
          leg5.moveDown();
          vTaskDelay(300 / portTICK_PERIOD_MS);
          leg2.moveUp();
          leg4.moveUp();
          leg6.moveUp();
          vTaskDelay(300 / portTICK_PERIOD_MS);
          leg2.moveBack();
          leg4.moveBack();
          leg6.moveBack();
          vTaskDelay(300 / portTICK_PERIOD_MS);
          leg2.moveDown();
          leg4.moveDown();
          leg6.moveDown();
          vTaskDelay(300 / portTICK_PERIOD_MS);
          leg2.moveUp();
          leg5.moveUp();
          vTaskDelay(300 / portTICK_PERIOD_MS);
          leg1.moveCenter();
          leg3.moveCenter();
          leg4.moveCenter();
          leg6.moveCenter();
          vTaskDelay(400 / portTICK_PERIOD_MS);
          leg2.moveDown();
          leg5.moveDown();
        }
        else
          xSemaphoreGive(mutex_prox);
      }
      legFlag = 0;
      break;
    }
    case 0x92:
    {
      if (xSemaphoreTake(mutex_prox, portMAX_DELAY) == pdTRUE)
      {
        if (prox.front > 10)
        {
          xSemaphoreGive(mutex_prox);
          Serial.println("A step backward");
          leg1.moveUp();
          leg3.moveUp();
          leg5.moveUp();
          vTaskDelay(300 / portTICK_PERIOD_MS);
          leg1.moveBack();
          leg3.moveBack();
          leg5.moveBack();
          vTaskDelay(300 / portTICK_PERIOD_MS);
          leg1.moveDown();
          leg3.moveDown();
          leg5.moveDown();
          vTaskDelay(300 / portTICK_PERIOD_MS);
          leg2.moveUp();
          leg4.moveUp();
          leg6.moveUp();
          vTaskDelay(300 / portTICK_PERIOD_MS);
          leg2.moveFront();
          leg4.moveFront();
          leg6.moveFront();
          vTaskDelay(300 / portTICK_PERIOD_MS);
          leg2.moveDown();
          leg4.moveDown();
          leg6.moveDown();
          vTaskDelay(300 / portTICK_PERIOD_MS);
          leg2.moveUp();
          leg5.moveUp();
          vTaskDelay(300 / portTICK_PERIOD_MS);
          leg1.moveCenter();
          leg3.moveCenter();
          leg4.moveCenter();
          leg6.moveCenter();
          vTaskDelay(400 / portTICK_PERIOD_MS);
          leg2.moveDown();
          leg5.moveDown();
        }
        else
          xSemaphoreGive(mutex_prox);
      }
      legFlag = 0;
      break;
    }
    case 0x93:
    {
      Serial.println("A turn right");
      leg1.moveUp();
      leg3.moveUp();
      leg5.moveUp();
      vTaskDelay(300 / portTICK_PERIOD_MS);
      leg1.moveFront();
      leg3.moveFront();
      leg5.moveFront();
      vTaskDelay(300 / portTICK_PERIOD_MS);
      leg1.moveDown();
      leg3.moveDown();
      leg5.moveDown();
      vTaskDelay(300 / portTICK_PERIOD_MS);
      leg2.moveUp();
      leg4.moveUp();
      leg6.moveUp();
      vTaskDelay(300 / portTICK_PERIOD_MS);
      leg2.moveFront();
      leg4.moveFront();
      leg6.moveFront();
      vTaskDelay(300 / portTICK_PERIOD_MS);
      leg2.moveDown();
      leg4.moveDown();
      leg6.moveDown();
      vTaskDelay(300 / portTICK_PERIOD_MS);
      leg2.moveUp();
      leg5.moveUp();
      vTaskDelay(300 / portTICK_PERIOD_MS);
      leg1.moveCenter();
      leg3.moveCenter();
      leg4.moveCenter();
      leg6.moveCenter();
      vTaskDelay(400 / portTICK_PERIOD_MS);
      leg2.moveDown();
      leg5.moveDown();
      legFlag = 0;
      break;
    }
    case 0x94:
    {
      Serial.println("A turn left");
      leg1.moveUp();
      leg3.moveUp();
      leg5.moveUp();
      vTaskDelay(300 / portTICK_PERIOD_MS);
      leg1.moveBack();
      leg3.moveBack();
      leg5.moveBack();
      vTaskDelay(300 / portTICK_PERIOD_MS);
      leg1.moveDown();
      leg3.moveDown();
      leg5.moveDown();
      vTaskDelay(300 / portTICK_PERIOD_MS);
      leg2.moveUp();
      leg4.moveUp();
      leg6.moveUp();
      vTaskDelay(300 / portTICK_PERIOD_MS);
      leg2.moveBack();
      leg4.moveBack();
      leg6.moveBack();
      vTaskDelay(300 / portTICK_PERIOD_MS);
      leg2.moveDown();
      leg4.moveDown();
      leg6.moveDown();
      vTaskDelay(300 / portTICK_PERIOD_MS);
      leg2.moveUp();
      leg5.moveUp();
      vTaskDelay(300 / portTICK_PERIOD_MS);
      leg1.moveCenter();
      leg3.moveCenter();
      leg4.moveCenter();
      leg6.moveCenter();
      vTaskDelay(400 / portTICK_PERIOD_MS);
      leg2.moveDown();
      leg5.moveDown();
      legFlag = 0;
      break;
    }
    case 0x99:
    {
      Serial.println("Stand up");
      leg1.moveCenter();
      leg3.moveCenter();
      leg4.moveCenter();
      leg6.moveCenter();
      vTaskDelay(1);
      leg1.moveDown();
      leg2.moveDown();
      leg3.moveDown();
      leg4.moveDown();
      leg5.moveDown();
      leg6.moveDown();
      legFlag = 0;
      break;
    }
    }
    xSemaphoreGive(mutex_leg);
  }

  vTaskDelay(500 / portTICK_PERIOD_MS);
}