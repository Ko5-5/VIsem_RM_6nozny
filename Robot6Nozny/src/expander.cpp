#include "expander.h"

void Expander::configurePort(uint8_t port, uint8_t custom)
{
  if (custom == INPUT)
  {
    writeBlockData(port, 0xFF);
  }
  else if (custom == OUTPUT)
  {
    writeBlockData(port, 0x00);
  }
  else
  {
    writeBlockData(port, custom);
  }
}

void Expander::writeBlockData(uint8_t cmd, uint8_t data)
{
  Wire.beginTransmission(MCPAddress);
  Wire.write(cmd);
  Wire.write(data);
  Wire.endTransmission();
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

uint8_t Expander::valueFromPin(uint8_t pin, uint8_t statusGP)
{
  return (statusGP & (0x0001 << pin)) == 0 ? 0 : 1;
}

uint8_t Expander::readPin(uint8_t pin, uint8_t gp)
{
  uint8_t statusGP = 0;
  Wire.beginTransmission(MCPAddress);
  Wire.write(gp);
  Wire.endTransmission();
  Wire.requestFrom(MCPAddress, 1); // ler do chip  1 byte
  statusGP = Wire.read();

  return valueFromPin(pin, statusGP);
}

void Expander::setup()
{
  Serial.begin(9600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  uint8_t connect = Wire.begin(I2C_SDA, I2C_SCL, 200000); // ESP32
  Wire.setClock(200000);                                  // frequencia
  Serial.println(connect ? "MCP23016 connection succesful" : "MCP23016 connection failed");
  // configurePort(IODIR0, INPUT);
  // configurePort(IODIR1, INPUT);
  writeBlockData(IODIR0, 0xFF);
  writeBlockData(IODIR1, 0xFF);
}

void Expander::updateButtons()
{
  buttons[0] = readPin(5, GP1);
  buttons[1] = readPin(2, GP1);
  buttons[2] = readPin(1, GP1);
  buttons[3] = readPin(1, GP0);
  buttons[4] = readPin(3, GP0);
  buttons[5] = readPin(4, GP0);
}