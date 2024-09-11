#ifndef _CONVERSION_H
#define _CONVERSION_H

#include <Arduino.h>

#define MOSFET1 1
#define MOSFET2 2

/*********************** ADC en funcion del DutyCycle ************************/

// Coeficientes de la regresión polinómica, para MOSFET1
#define AM1  0.000318318466981727
#define BM1  -1.69726098379941
#define CM1  2804.90220497491
// MOSFET2
#define AM2  0.000228750264512528
#define BM2  -1.24017153035163
#define CM2  2222.8681053449

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

// Función para convertir ADC a Duty Cycle
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

/******************** Corriente en funcion del DutyCycle *********************/
#define AA1  6.06822970192502E-6
#define BB1  -0.0323636188175456
#define CC1  43.2261758151534
// MOSFET2
#define AA2  4.59759310013991E-06
#define BB2  -0.0250369488847147
#define CC2  34.1090211439599


// Función para convertir Duty Cycle a Amperes
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

#endif