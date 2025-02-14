#ifndef _MY_EEPROM_H
#define _MY_EEPROM_H

#include <Arduino.h>
#include <EEPROM.h>
#include "calibrate.h"

#define C_0A_CAL_ADDR   0    // Direccion en la eeprom, donde voy a guardar los valores de calibracion
#define C_1A_CAL_ADDR   sizeof(double)

void myEeprom_writeDouble(uint16_t addr,  double value)
{
    byte* p = (byte*)(void*)&value; // Convertir el puntero a byte
    for (int i = 0; i < sizeof(double); i++) {
        EEPROM.write(addr + i, *(p + i)); // Escribir byte a byte
    }
}
  
double myEeprom_readDouble(uint16_t addr)
{
    double value = 0;
    byte* p = (byte*)(void*)&value; // Convertir el puntero a byte
    for (int i = 0; i < sizeof(double); i++) {
        *(p + i) = EEPROM.read(addr + i); // Leer byte a byte
    }
    return value;
}
#endif