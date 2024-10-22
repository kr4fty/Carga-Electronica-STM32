#ifndef CONTROL_H
#define CONTROL_H

#include "pwm.h"

extern double Setpoint;

void control_stopOutputsAndReset(){
    pwm_setDuty1(0);
    pwm_setDuty2(0);
    Setpoint = 0; // Reinicia el setpoint
}

#endif // CONTROL_H