#include "ble_setup.h"

uint8_t commandHandler(uint8_t *inputValue)
{
    switch (*inputValue)
    {
    case 0x10:
    {
        return 1;
        break;
    }
    case 0x11:
    {
        return 11;
        break;
    }
    case 0x12:
    {
        return 12;
        break;
    }
    case 0x13:
    {
        return 13;
        break;
    }
    case 0x14:
    {
        return 14;
        break;
    }
    case 0x20:
    {
        return 20;
        break;
    }
    case 0x21:
    {
        return 21;
        break;
    }
    case 0x22:
    {
        return 22;
        break;
    }
    case 0x23:
    {
        return 23;
        break;
    }
    case 0x24:
    {
        return 24;
        break;
    }
    case 0x30:
    {
        return 30;
        break;
    }
    case 0x31:
    {
        return 31;
        break;
    }
    case 0x32:
    {
        return 32;
        break;
    }
    case 0x33:
    {
        return 33;
        break;
    }
    case 0x34:
    {
        return 34;
        break;
    }
    case 0x40:
    {
        return 40;
        break;
    }
    case 0x41:
    {
        return 41;
        break;
    }
    case 0x42:
    {
        return 42;
        break;
    }
    case 0x43:
    {
        return 43;
        break;
    }
    case 0x44:
    {
        return 44;
        break;
    }
    case 0x50:
    {
        return 50;
        break;
    }
    case 0x51:
    {
        return 51;
        break;
    }
    case 0x52:
    {
        return 52;
        break;
    }
    case 0x53:
    {
        return 53;
        break;
    }
    case 0x54:
    {
        return 54;
        break;
    }
    case 0x60:
    {
        return 60;
        break;
    }
    case 0x61:
    {
        return 61;
        break;
    }
    case 0x62:
    {
        return 62;
        break;
    }
    case 0x63:
    {
        return 63;
        break;
    }
    case 0x64:
    {
        return 64;
        break;
    }
    case 0x99:
    {
        return 99;
        break;
    }
    }
}