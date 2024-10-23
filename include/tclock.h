#ifndef _CLOCK_H
#define _CLOCK_H

#include <Arduino.h>

#define NO_LIMIT 0              // Limite de tiempo del proceso intereminado

unsigned long previousMillis=0; // Almacena el último tiempo en que se ejecutó la acción
unsigned long currentMillis;    // Obtiene el tiempo actual
long totalTime=0;      // Tiempo total en segundos
unsigned long timeDuration=NO_LIMIT;   // Tiempo que estara el proceso encendido

int8_t seconds = 0;
int8_t minutes = 0;
int8_t hours = 0;

// Definición de la estructura para almacenar horas, minutos y segundos
typedef struct {
    int horas = 0;
    int minutos = 0;
    int segundos = 0;
} Tiempo;

void clock_update()
{
    totalTime++;
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
void clock_decrement_time()
{
    totalTime--;
    seconds--;
    if (seconds < 0) {
        seconds = 59;
        minutes--;
        if (minutes < 0) {
            minutes = 59;
            hours--;
            if (hours < 0) {
                hours = 23; // Reiniciar a 0 después de 24 horas
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

unsigned long clock_standar_format_to_totalTime(Tiempo time)
{
    unsigned long totalT = time.horas*60*60 + time.minutos*60 + time.segundos;

    return totalT;
}

void clock_resetClock(unsigned long newPeriod=0)
{
    if(!newPeriod){
        seconds = 0;
        minutes = 0;
        hours = 0;
        totalTime = 0;
    }
    else{
        Tiempo time = clock_totalTime_to_standar_format(newPeriod);
        seconds = time.segundos;
        minutes = time.minutos;
        hours = time.horas;
        totalTime = newPeriod;
    }
}
#endif