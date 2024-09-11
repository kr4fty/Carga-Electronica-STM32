#ifndef _CLOCK_H
#define _CLOCK_H

#include <Arduino.h>

unsigned long previousMillis = 0;   // Almacena el último tiempo en que se ejecutó la acción
unsigned long currentMillis ;       // Obtiene el tiempo actual

uint8_t seconds = 0;
uint8_t minutes = 0;
uint8_t hours = 0;

void clock_update()
{
    seconds++;
    if (seconds >= 60) {
        seconds = 0;
        minutes++;
        if (minutes >= 60) {
            minutes = 0;
            hours++;
            if (hours >= 24) {
                hours = 0; // Reiniciar a 0 después de 24 horas
            }
        }
    }
}

uint8_t clock_get_seconds()
{
    return seconds;
}

uint8_t clock_get_minutes()
{
    return minutes;
}
uint8_t clock_get_hours()
{
    return hours;
}

void clock_resetClock()
{
    seconds = 0;
    minutes = 0;
    hours = 0;
}
#endif