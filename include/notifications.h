#ifndef _NOTIFICATIONS_H
#define _NOTIFICATIONS_H

#include <Arduino.h>
#include "display.h"

#define NO_TIME_LIMIT 0 // Expresa que seguira por tiempo indefinido

typedef struct {
    char message[50];
    uint8_t priority=0;
    unsigned long timestamp; // Tiempo en que se creó la notificación
} Notification;

Notification notification;

int notificationCount = 0;
bool newNotification=false, newMiniNotificacion=false; // True: si hay una notificacion que se muestra
unsigned long messageDisplayTime = 2000; // por default 2 seg


void notification_add(const char *message, uint8_t priority, unsigned long duration=messageDisplayTime, uint8_t color=COLOR_BW)// Inserta una nueva notificacion con prioridad mas alta
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

void notification_addMini(const char *message, uint8_t priority, unsigned long duration=messageDisplayTime, uint8_t color=COLOR_BW)// Usando las mini notificaciones puedo mostrar la temperatura
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

void notification_remove(uint8_t priority) // Elimina una notificacion con prioridad Priority
{
    if(notification.priority == priority){
        notification.priority = 0;
        newNotification = false;
    }
}

void notification_removeMini(uint8_t priority) // Elimina una notificacion con prioridad Priority
{
    if(notification.priority == priority){
        notification.priority = 0;
        newMiniNotificacion = false;
    }
}

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