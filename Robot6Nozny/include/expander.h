#ifndef EXPANDER_H
#define EXPANDER_H

#include <Wire.h>

#define GP0 0x00     // DATA PORT REGISTER 0
#define GP1 0x01     // DATA PORT REGISTER 1
#define OLAT0 0x02   // OUTPUT LATCH REGISTER 0
#define OLAT1 0x03   // OUTPUT LATCH REGISTER 1
#define IPOL0 0x04   // INPUT POLARITY PORT REGISTER 0
#define IPOL1 0x05   // INPUT POLARITY PORT REGISTER 1
#define IODIR0 0x06  // I/O DIRECTION REGISTER 0
#define IODIR1 0x07  // I/O DIRECTION REGISTER 1
#define INTCAP0 0x08 // INTERRUPT CAPTURE REGISTER 0
#define INTCAP1 0x09 // INTERRUPT CAPTURE REGISTER 1
#define IOCON0 0x0A  // I/O EXPANDER CONTROL REGISTER 0
#define IOCON1 0x0B  // I/O EXPANDER CONTROL REGISTER 1

#define MCPAddress 0x20

class Expander
{
public:
    void setupExp();
    void loopExp();

    // configura o GPIO (GP0 ou GP1)
    // como parametro passamos:
    // port: GP0 ou GP1
    // custom: INPUT para todos as portas do GP trabalharem como entrada
    //         OUTPUT para todos as portas do GP trabalharem como saida
    //         custom um valor de 0-255 indicando o modo das portas (1=INPUT, 0=OUTPUT)
    //         ex: 0x01 ou B00000001 ou  1 : indica que apenas o GPX.0 trabalhará como entrada, o restando como saida

    void configurePort(uint8_t port, uint8_t custom);

    // envia dados para o MCP23016 através do barramento i2c
    // cmd: COMANDO (registrador)
    // data: dados (0-255)

    void writeBlockData(uint8_t cmd, uint8_t data);

    // verifica se o botão foi pressionado
    // parametro GP: GP0 ou GP1

    void checkButton(uint8_t GP);

    // faz a leitura de um pino específico
    // pin: pino desejado (0-7)
    // gp: GP0 ou GP1
    // retorno: 0 ou 1

    uint8_t readPin(uint8_t pin, uint8_t gp);

    // retorna o valor do bit na posição desejada
    // pin: posição do bit (0-7)
    // statusGP: valor lido do GP (0-255)

    uint8_t valueFromPin(uint8_t pin, uint8_t statusGP);
};

#endif