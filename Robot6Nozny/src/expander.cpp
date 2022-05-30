#include "expander.h"



Expander::void setupExp()   {
  Serial.begin(9600);
 
  delay(1000);
  Wire.begin(19,23); //ESP32

 
Wire.setClock(200000); //frequencia
 
  //configura o GPIO0 como OUTPUT (todos os pinos)
  configurePort(IODIR0, OUTPUT);
  //configura o GPIO1 como INPUT o GP1.0 e como OUTPUT os outros GP1
  configurePort(IODIR1, 0x01);
  //seta todos os pinos do GPIO0 como LOW
  writeBlockData(GP0, B00000000);
  //seta todos os pinos do GPIO1 como LOW
  writeBlockData(GP1, B00000000);
}
 
 
Expander::void loopExp() {
   //verifica e o botão GP foi pressionado
   checkButton(GP1);
    
} 

Expander::void configurePort(uint8_t port, uint8_t custom)
{
  if(custom == INPUT)
  {
    writeBlockData(port, 0xFF);
  }
  else if(custom == OUTPUT)
  {
    writeBlockData(port, 0x00);
  }
  else
  {
    writeBlockData(port, custom);
  }
}


Expander::void writeBlockData(uint8_t cmd, uint8_t data)
{
  Wire.beginTransmission(MCPAddress);
  Wire.write(cmd);
  Wire.write(data);
  Wire.endTransmission();
  delay(10);
}


Expander::void checkButton(uint8_t GP)
{
  //faz a leitura do pino 0 no GP fornecido
   uint8_t btn = readPin(0,GP);
   //se botão pressionado, seta para HIGH as portas GP0
   if(btn)
   {
      writeBlockData(GP0, B11111111);
   }
   //caso contrario deixa todas em estado LOW
   else{
        writeBlockData(GP0, B00000000);
   }
}


Expander::uint8_t readPin(uint8_t pin, uint8_t gp)
{
    uint8_t statusGP = 0;
    Wire.beginTransmission(MCPAddress);
    Wire.write(gp); 
    Wire.endTransmission();
    Wire.requestFrom(MCPAddress, 1); // ler do chip  1 byte
    statusGP = Wire.read(); 
 
    return valueFromPin(pin, statusGP);
}
 

Expander::uint8_t valueFromPin(uint8_t pin, uint8_t statusGP)
{
  return (statusGP &( 0x0001 << pin)) == 0 ? 0 : 1;
}



