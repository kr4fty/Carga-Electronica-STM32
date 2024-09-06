#ifndef _CONVERSION_H
#define _CONVERSION_H

#include <Arduino.h>

// Coeficientes de la regresión polinómica
const double a = 0.000276531754856837;  // Ajusta según tus cálculos
const double b = -1.43948056287478;     // Ajusta según tus cálculos
const double c = 2413.16196611298;      // Ajusta según tus cálculos

// Función para convertir Duty Cycle a ADC
double dutycycleToADC(int dutyCycle) {
    return a * pow(dutyCycle, 2) + b * dutyCycle + c;
}

// Función para convertir ADC a Duty Cycle
int adcToDutycycle(double adcValue) {
    // Coeficientes de la ecuación cuadrática
    double A = a;
    double B = b;
    double C = c - adcValue;

    // Calcula el discriminante
    double discriminant = B * B - 4 * A * C;

    // Calcula las dos soluciones
    double dutyCycle1 = (-B + sqrt(discriminant)) / (2 * A);
    double dutyCycle2 = (-B - sqrt(discriminant)) / (2 * A);

    // Devuelve la solución válida (debe estar en el rango 0-4095)
    if (dutyCycle1 >= 0 && dutyCycle1 <= 4095) {
        return static_cast<int>(dutyCycle1);
    } else if (dutyCycle2 >= 0 && dutyCycle2 <= 4095) {
        return static_cast<int>(dutyCycle2);
    } else {
        return -1; // Indica un error si ninguna solución es válida
    }
}

const double aa = 5.94507403588975E-6;
const double bb = -0.031275076187976;
const double cc = 41.1938473619435;

// Función para convertir Duty Cycle a Amperes
double dutyCycleToAmpere(int dutyCycle)
{
    return aa * pow(dutyCycle, 2) + bb * dutyCycle + cc;
}

// Función para convertir Amperes a Dutycycle
int ampereToDutycycle(double ampereValue) {
    // Coeficientes de la ecuación cuadrática
    double A = aa;
    double B = bb;
    double C = cc - ampereValue;

    // Calcula el discriminante
    double discriminant = B * B - 4 * A * C;

    // Calcula las dos soluciones
    double dutyCycle1 = (-B + sqrt(discriminant)) / (2 * A);
    double dutyCycle2 = (-B - sqrt(discriminant)) / (2 * A);

    // Devuelve la solución válida (debe estar en el rango 0-4095)
    if (dutyCycle1 >= 0 && dutyCycle1 <= 4095) {
        return static_cast<int>(dutyCycle1);
    } else if (dutyCycle2 >= 0 && dutyCycle2 <= 4095) {
        return static_cast<int>(dutyCycle2);
    } else {
        return -1; // Indica un error si ninguna solución es válida
    }
}

#endif