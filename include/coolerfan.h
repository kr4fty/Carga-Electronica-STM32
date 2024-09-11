/*
 * Por el uso de un transistor driver para activar al mosfet, que es quien
 * enciende el cooler, este utiliza logica inversa. Activo cooler con un LOW y 
 * apago con un HIGH.
 */

#ifndef _COOLER_H
#define _COOLER_H

#include <Arduino.h>
#include "config.h"

void coolerFan_init()
{
    pinMode(COOLER_FAN_PIN, OUTPUT);
    
    digitalWrite(COOLER_FAN_PIN, HIGH);
}

void coolerFan_powerOn()
{
    digitalWrite(COOLER_FAN_PIN, LOW);
}

void coolerFan_powerOff()
{
    digitalWrite(COOLER_FAN_PIN, HIGH);
}

#endif