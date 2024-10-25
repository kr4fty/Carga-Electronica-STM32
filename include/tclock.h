#ifndef _CLOCK_H
#define _CLOCK_H

#include <Arduino.h>

#define NO_LIMIT 0              // Limite de tiempo del proceso intereminado

unsigned long previousMillis=0; // Almacena el último tiempo en que se ejecutó la acción
unsigned long currentMillis;    // Obtiene el tiempo actual
long totalTime=0;               // Tiempo total en funcionamiento, en segundos
unsigned long timeDuration=NO_LIMIT;   // Tiempo que estara el proceso encendido

// Definición de la estructura para almacenar horas, minutos y segundos
typedef struct {
    int horas = 0;
    int minutos = 0;
    int segundos = 0;
} Tiempo;

Tiempo tiempo;

void clock_update()
{
    totalTime++;
    tiempo.segundos++;
    if (tiempo.segundos >= 60) {
        tiempo.segundos = 0;
        tiempo.minutos++;
        if (tiempo.minutos >= 60) {
            tiempo.minutos = 0;
            tiempo.horas++;
            if (tiempo.horas >= 24) {
                tiempo.horas = 0; // Reiniciar a 0 después de 24 horas
            }
        }
    }
}

void clock_decrement_time()
{
    totalTime--;
    tiempo.segundos--;
    if (tiempo.segundos < 0) {
        tiempo.segundos = 59;
        tiempo.minutos--;
        if (tiempo.minutos < 0) {
            tiempo.minutos = 59;
            tiempo.horas--;
            if (tiempo.horas < 0) {
                tiempo.horas = 23; // Reiniciar a 0 después de 24 horas
            }
        }
    }
}

uint8_t clock_get_seconds()
{
    return tiempo.segundos;
}

uint8_t clock_get_minutes()
{
    return tiempo.minutos;
}

uint8_t clock_get_hours()
{
    return tiempo.horas;
}

unsigned long clock_get_total_time(){
    return totalTime;
}

Tiempo clock_totalTime_to_standar_format(int totalTime)
{
    Tiempo tiempo; // Crear una instancia de la estructura Tiempo
    tiempo.horas = totalTime / 3600; // Calcula las horas
    tiempo.minutos = (totalTime % 3600) / 60; // Calcula los minutos
    tiempo.segundos = totalTime % 60; // Calcula los segundos

    return tiempo; // Retorna la estructura
}

unsigned long clock_standar_format_to_totalTime(Tiempo clock)
{
    unsigned long totalT = clock.horas*60*60 + clock.minutos*60 + clock.segundos;

    return totalT;
}

void clock_resetClock(unsigned long newPeriod=0)
{
    if(!newPeriod){
        tiempo.segundos = 0;
        tiempo.minutos = 0;
        tiempo.horas = 0;
        totalTime = 0;
    }
    else{
        tiempo = clock_totalTime_to_standar_format(newPeriod);
        totalTime = newPeriod;
    }
}

#endif