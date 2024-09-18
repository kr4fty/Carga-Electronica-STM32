#ifndef _NOTIFICATIONS_H
#define _NOTIFICATIONS_H

#include <Arduino.h>
#include "display.h"

#define NO_TIME_LIMIT 0 // Expresa que seguira por tiempo indefinido
#define MESSAGE_DIAPLAY_TIME 2000 // por default 2 seg

typedef struct {
    char message[50];
    uint8_t priority=0;
    unsigned long timestamp; // Tiempo en que se creó la notificación
} Notification;

Notification notification;

bool newNotification=false, newMiniNotificacion=false; // True: si hay una notificacion que muestrar

// Inserta una nueva notificacion con prioridad mas alta
void notification_add(const char *message, uint8_t priority, unsigned long duration=MESSAGE_DIAPLAY_TIME, uint8_t color=COLOR_BW)
{
    if(priority >= notification.priority) // comprueba si la nueva es de mayor prioridad, si es que hay una anterior en pantalla
    {
        if(newMiniNotificacion){
            newMiniNotificacion = false; // si habia una de menor prioridad activa GRANDE antes, la desactivo
        }
        newNotification = true; // SI, hay nueva notificacion mas prioritaria a imprimir

        sprintf(notification.message,"%s",message);
        notification.priority = priority;

        if(duration>0){
            notification.timestamp = millis() + duration;
        }
        else{ // Si recibo una duracion 0 significa que la noficicacion seguira estando hasta que venga un estado de igual prioridad que la modifique
            notification.timestamp = 0;
        }
        
        // Imprimo inmediatamente la nueva notificacion
        lcd_printNotification(notification.message, color);
    }
}

// Usando las mini notificaciones puedo mostrar la temperatura
void notification_addMini(const char *message, uint8_t priority, unsigned long duration=MESSAGE_DIAPLAY_TIME, uint8_t color=COLOR_BW)
{
    if(priority >= notification.priority) // comprueba si la nueva es de mayor prioridad, si es que hay una anterior en pantalla
    {
        if(newNotification){
            newNotification = false; // si habia una de menor prioridad activa GRANDE antes, la desactivo
        }
        newMiniNotificacion = true; // SI, hay nueva notificacion mas prioritaria a imprimir

        sprintf(notification.message,"%s",message);
        notification.priority = priority;

        if(duration>0){
            notification.timestamp = millis() + duration;
        }
        else{ // Si recibo una duracion 0 significa que la noficicacion seguira estando hasta que venga un estado de igual prioridad que la modifique
            notification.timestamp = 0;
        }
        
        // Imprimo inmediatamente la nueva notificacion
        lcd_printNotification(notification.message, color);
    }
}

 // Elimina una notificacion con prioridad Priority
void notification_remove(uint8_t priority)
{
    if(notification.priority == priority){
        notification.priority = 0;
        newNotification = false;
    }
}

 // Elimina una mini notificacion con prioridad Priority
void notification_removeMini(uint8_t priority)
{
    if(notification.priority == priority){
        notification.priority = 0;
        newMiniNotificacion = false;
    }
}

// Devuelve true si la notificacion ya expiro
bool notification_hasExpired()
 {
    if(millis()>notification.timestamp && notification.timestamp>NO_TIME_LIMIT){
        newNotification = false;
        notification.priority = 0;
        return true;
    }

    return false;
 }

#endif