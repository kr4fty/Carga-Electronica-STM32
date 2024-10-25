#ifndef CONTROL_H
#define CONTROL_H

#include "pwm.h"
#include "notifications.h"
#include "sound.h"
#include "tclock.h"

extern double Setpoint;         // Setpoint del sistema
extern float totalmAh;          // mAh totales consumidos
extern float totalWh;           // Wh totales consumidos
extern bool batteryConnected;   // True: tengo bateria conectada
extern bool forceRePrint;       // True: limpia y fuerza la reimpresion en pantalla
extern bool isPowerOn;          // True: proceso funcionando

void control_resetCounters()
{
    totalmAh = 0;
    totalWh = 0;
    if(timeDuration == NO_LIMIT){
        totalTime = 0;
    }
    else{
        totalTime = timeDuration;
    }
}

void control_stopOutputsAndReset()
{
    pwm_setDuty1(0);
    pwm_setDuty2(0);
    Setpoint = 0; // Reinicia el setpoint
}

void control_forceReprintDisplay()
{
    batteryConnected = true;
    forceRePrint = true;
}

void control_resetAllForNewMode()
{
    // Apago si estaba en funcionamiento
    isPowerOn = false;
    // Apago Led indicador de estado encendido
    digitalWrite(LED, HIGH); 
    // Apago las salidas PWM
    control_stopOutputsAndReset();
    // Reinicio los contadores
    control_resetCounters();
    // Reinicio el tiempo a valores por defecto, de acuerdo el modo seleccionado
    clock_resetClock(timeDuration);
}

void control_powerOff()
{
    uint8_t notificationPriority = 2;
    notification_remove(3);
    notification_add("   POWER OFF  ", notificationPriority);
    
    tone(BUZZER_PIN, 436, 100);
    tone(BUZZER_PIN, 200, 150);

    control_stopOutputsAndReset();
}

#endif // CONTROL_H