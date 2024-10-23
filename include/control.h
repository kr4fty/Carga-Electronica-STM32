#ifndef CONTROL_H
#define CONTROL_H

#include "pwm.h"
#include "notifications.h"
#include "sound.h"

extern double Setpoint;

void control_stopOutputsAndReset(){
    pwm_setDuty1(0);
    pwm_setDuty2(0);
    Setpoint = 0; // Reinicia el setpoint
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