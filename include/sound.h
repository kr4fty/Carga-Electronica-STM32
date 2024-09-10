#ifndef _SOUND_H
#define _SOUND_H
#include <Arduino.h>

// GPIO PA1
void tone_init() {
    // Habilitar el reloj para TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Habilitar el reloj para el GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // Configurar PA1 como salida alternativa (modo push-pull)
    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1); // Limpiar configuraciones
    GPIOA->CRL |= (GPIO_CRL_MODE1_1 | GPIO_CRL_CNF1_1); // Salida a 2 MHz, modo alternativo

    // Configurar el modo PWM en el canal 2
    TIM2->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2); // Modo PWM1 para el canal 2
    TIM2->CCMR1 &= ~TIM_CCMR1_OC2PE; // Deshabilitar la pre-carga
}

inline void tone(uint8_t _pin, unsigned int frequency, unsigned long duration)
{
    // Configurar el prescaler y el valor de auto-reload
    TIM2->PSC = 72 - 1; // Prescaler 
    TIM2->ARR = (1000000 / frequency) - 1; // Configurar el valor de auto-reload

    // Establecer el valor de comparación para un ciclo de trabajo del 50%
    TIM2->CCR2 = TIM2->ARR / 2; // 50% de ciclo de trabajo

    // Habilitar la salida del canal 2
    TIM2->CCER |= TIM_CCER_CC2E; // Habilitar la salida de PWM en el canal 2

    // Habilitar el contador
    TIM2->CR1 |= TIM_CR1_CEN; // Habilitar el contador

    // Generar la señal PWM durante la duración especificada
    uint32_t endT = millis() + duration;
    while (millis() < endT);

    // Detener el tono
    // Deshabilitar la salida del canal 2
    TIM2->CCER &= ~TIM_CCER_CC2E; // Deshabilitar la salida del canal 2
    // Detener el temporizador
    TIM2->CR1 &= ~TIM_CR1_CEN; // Deshabilitar el temporizador
    
}

#endif