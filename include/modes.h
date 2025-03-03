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
#define R_CONST_MODE      3 // Modo Resistencia Constante

extern float powerSetpoint; // Variable global que contiene el setPoint de Potencia
extern float resistanceSetpoint; // Variable global que contiene el setPoint de Resistencia
uint8_t controlMode=C_CONST_MODE; // Modo de control, corriente cte. por defecto

#define C_1A            1.0 // 1A
#define P_1W            1.0 // 1W
#define R_10R          10.0 // 10R
#define V_31V           3.1 // 3.1V
#define V_0V            0.0 // 0.0V

// Función para actualizar el setpoint en modo Corriente Constante
float modes_updateCurrentSetpoint(long encoderValue)
{
    float ampereSetpoint = encoderValue / 100.0; // Potencias desde 0 a 10A con una resolución de 0.01A

    return ampereSetpoint;
}

// Función para actualizar el setpoint en modo Potencia Constante
float modes_updatePowerSetpoint(double vIn, long encoderValue)
{
    powerSetpoint = encoderValue / 10.0; // Potencias desde 0 a 100W con una resolución de 0.1W

    float ampereSetpoint = powerSetpoint / vIn; // Cálculo de la corriente necesaria

    return ampereSetpoint;
}

// Función para actualizar el setpoint en modo Resistencia Constante
float modes_updateResistenceSetpoint(double vIn, long encoderValue)
{
    resistanceSetpoint = encoderValue / 10.0; // Resistencias desde 0 a 100R con una resolución de 0.1R

    float ampereSetpoint = vIn/resistanceSetpoint; // Cálculo de la corriente necesaria

    return ampereSetpoint;
}

// Función principal que maneja el encoder
float modes_handleEncoderChange(double vIn, long encoderValue, uint8_t mode) {
    switch (mode)
    {
        case C_CONST_MODE:
            return modes_updateCurrentSetpoint(encoderValue);
            break;
        case P_CONST_MODE:
            return modes_updatePowerSetpoint(vIn, encoderValue);
            break;
        case R_CONST_MODE:
            return modes_updateResistenceSetpoint(vIn, encoderValue);
            break;
        default:
            return 1; // porque me tira error el compilador para el caso que no es ninguno de los dos anteriores
            break;
    }
}

#endif