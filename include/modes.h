/*
 * modes.h
 *          Contiene las funciones especificas para cada modo de operacion
 * 
 * Autor: Tapia Velasquez Favio
 */

#ifndef _MODES_H
#define _MODES_H

#include "display.h"

// Modos de control
#define NO_MODE           0 // No hay modo seleccionado
#define C_CONST_MODE      1 // Modo Corriente Constante
#define P_CONST_MODE      2 // Modo Potencia Constante
#define LIMITED_TIME_MODE 3 // Modo Corriente Constante con un límite de tiempo

extern float powerSetpoint; // Variable global que contine el setPoint de Potencia 
uint8_t controlMode=C_CONST_MODE; // Modo de control, corriente cte. por defecto

#define C_1A            1.0 // 1A
#define P_1W            1.0 // 1W


// Función para actualizar el setpoint en modo corriente
float modes_updateCurrentSetpoint(long encoderValue) {
    float ampereSetpoint = encoderValue / 100.0; // Potencias desde 0 a 10A con una resolución de 0.01A

    return ampereSetpoint;
}

// Función para actualizar el setpoint en modo potencia
float modes_updatePowerSetpoint(double vIn, long encoderValue) {
    powerSetpoint = encoderValue / 10.0; // Potencias desde 0 a 100W con una resolución de 0.1W

    float ampereSetpoint = powerSetpoint / vIn; // Cálculo de la corriente necesaria

    return ampereSetpoint;
}

// Función principal que maneja el encoder
float modes_handleEncoderChange(double vIn, long encoderValue, uint8_t mode) {
    if (mode == C_CONST_MODE) {
        return modes_updateCurrentSetpoint(encoderValue);
    } else if(mode == P_CONST_MODE){
        return modes_updatePowerSetpoint(vIn, encoderValue);
    } else {
        return 1; // porque me tira error el compilador para el caso que no es ninguno de los dos anteriores
    }
}

#endif