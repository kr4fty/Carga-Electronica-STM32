#ifndef _CONST_H
#define _CONST_H

// Medicion de Tension
#define R1            100.0
#define R2             20.0
#define GAIN_V (R2/(R1+R2))
#define V3_3            3.3

// Medicion de Corriente
#define R3              2.2
#define R4             10.0
#define VD              0.7
#define GAIN_I (R4/(R3+R4))
#define SENSIBILITY   0.185 // Modelo ACS712 5A
#define ADCOFFSET    541.61 // ADC medido en vacio
#define ADCRAW_1A (592.01-ADCOFFSET) // ADC medido con una corriente de 1A

#define ALPHA_V         0.1 // Factor de suavizado (ajustar según sea necesario) 0<x<1
#define ALPHA_I      0.0001 // numero mas cercano a 0 para mas filtrado                             
#define MU             0.01 // Tasa de aprendizaje, Filtro Adaptativo           0<x<1
#define _X_V              3 // Cte. filtro de Media Ponderada, X generalmente
#define _X_I              3 // corresponde al numero de muestras

// PID
// https://pidtuner.com/
//  Proportional Gain= 4
//  Integral Time= 0.35
//  Derivative Time= 0.05
//  Integral Gain= 11.428571428571429
//  Derivative Gain= 0.2

#define KP                4
#define KI         11.42857
#define KD              0.2

#endif