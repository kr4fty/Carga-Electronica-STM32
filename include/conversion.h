#ifndef _CONVERSION_H
#define _CONVERSION_H

#include <Arduino.h>
#include "calibrate.h"

#define MOSFET1 1
#define MOSFET2 2

/*********************** ADC en funcion del DutyCycle ************************/

// Coeficientes de la regresión polinómica, para MOSFET1
#define AM1 0.000203984593837537
#define BM1 -1.08140665266108
#define CM1 1987.46580882355
// MOSFET2
#define AM2 0.000281527347781217
#define BM2 -1.54884932920536
#define CM2 2681.0007739938

// Función para convertir Duty Cycle a ADC
double dutycycleToADC(int dutyCycle, uint8_t unity=MOSFET1)
{
    double a, b, c;
    switch (unity)
    {
    case MOSFET1:
        a = AM1; b = BM1; c = CM1;
        break;
    case MOSFET2:
        a = AM2; b = BM2; c = CM2;
        break;
    default:
        break;
    }
    return a * pow(dutyCycle, 2) + b * dutyCycle + c;
}

// Función para convertir valores de ADC a DutyCycle
int adcToDutycycle(double adcValue, uint8_t unity=MOSFET1)
{
    // Coeficientes de la ecuación cuadrática
    double A, B, C;
    switch (unity)
    {
    case MOSFET1:
        A = AM1; B = BM1; C = CM1-adcValue;
        break;
    case MOSFET2:
        A = AM2; B = BM2; C = CM2-adcValue;
        break;
    default:
        break;
    }

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

/******************** Corriente en función del DutyCycle *********************/
#define AA1 4.15812324929972E-06
#define BB1 -0.0220782703081233
#define CC1 29.3736470588236
// MOSFET2
#define AA2 5.29375644994838E-06
#define BB2 -0.0290162306501547
#define CC2 39.7774687822496


// Función para convertir valores de DutyCycle a Amperes
double dutyCycleToAmpere(int dutyCycle, uint8_t unity=MOSFET1)
{
    double a, b, c;
    switch (unity)
    {
    case MOSFET1:
        a = AA1; b = BB1; c = CC1;
        break;
    case MOSFET2:
        a = AA2; b = BB2; c = CC2;
        break;
    default:
        break;
    }
    return a * pow(dutyCycle, 2) + b * dutyCycle + c;
}

// Función para convertir Amperes a Dutycycle
int ampereToDutycycle(double ampereValue, uint8_t unity=MOSFET1)
{
    // Coeficientes de la ecuación cuadrática
    double A, B, C;
    switch (unity)
    {
    case MOSFET1:
        A = AA1; B = BB1; C = CC1-ampereValue;
        break;
    case MOSFET2:
        A = AA2; B = BB2; C = CC2-ampereValue;
        break;
    default:
        break;
    }

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

// Función para convertir Amperes a valores de ADC
//      I = (Iadc - iAdcOffset) / (AdcRaw_1A - iAdcOffset);
double ampereToAdc(double ampValue)
{
    double iAdc = ampValue*Adc1aDiff + iAdcOffset;

    return iAdc;
}

#endif