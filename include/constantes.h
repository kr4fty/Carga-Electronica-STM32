#ifndef _CONST_H
#define _CONST_H

// Tiempo
#define TIME_1SEG      1000 // Intervalo de 1 segundo

// Medicion de Tension
#define R1            100.0
#define R2             20.0
#define GAIN_V (R2/(R1+R2))
#define ADCRAW_1V      50.4
#define V3_3            3.3
// Tension
#define VBATT_MIN      0.26 // Vbatt min. para indicar si hay fuente conectada

// Medicion de Corriente
#define R3              2.2
#define R4             10.0
#define VD              0.7
#define GAIN_I (R4/(R3+R4))
#define SENSIBILITY   0.185 // Modelo ACS712 5A
#define IADCOFFSET    541.8 //541.61 // ADC medido en vacio
#define ADCRAW_1A     598.7 // ADC medido con una corriente de 1A
#define C_1A            1.0 // 1A
                         
#define MU            0.001 // Tasa de aprendizaje, Filtro Adaptativo con 0<x<1

// MOSFET
#define ADC_VGS_THRESHOLE  2650 // Tension minima de conduccion para este IRFZ44

// PID
//      https://pidtuner.com/
//          Proportional Gain= 35
//          Integral Time= 1
//          Derivative Time= 0.1
//          Integral Gain= 35
//          Derivative Gain= 3.5
// Constantes en modo Conservativo
#define KP_CNSTIVE     15.0
#define KI_CNSTIVE     15.0
#define KD_CNSTIVE      1.5
// Constantes en modo Agresivo
#define KP_AGG         35.0
#define KI_AGG         35.0
#define KD_AGG          3.5

#define PID_WINDOW_SIZE  10 // 10 mili segundos

#endif