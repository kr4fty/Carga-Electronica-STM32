/*
 *  PMW.H
 *        Al utilizar el GPIO PB1, este pertenece al canal 4 del TIM3
 *        Como se desea utilizar un rango del ciclo de trabajo desde 0 a 4095,
 *        12 bits de rosolucion del duty, entonces la frecuencia de salida del
 *        pwm sera:
 *                  Fpwm= Fclk/((ARR+1)*(PSC+1)) ~= 8.789KHz
 * 
 *        con ARR= 2¹²-1 = 4095, periodo
 *            PSC= 1, prescaler
 * 
 *        https://deepbluembedded.com/stm32-pwm-example-timer-pwm-mode-tutorial/
 */

#ifndef _PWM_H
#define _PWM_H
#include "config.h"

// Inicialización de PWM para PB1 y PB0
void pwm_init(void) {
    // 1. Habilitar el reloj para GPIOB y TIM3
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // Habilitar reloj para GPIOB
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Habilitar reloj para TIM3

    // 2. Configurar PB1 como salida alternativa (canal 4)
    GPIOB->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1); // Limpiar configuraciones
    GPIOB->CRL |= (GPIO_CRL_MODE1_1 | GPIO_CRL_CNF1_1); // Salida alternativa, modo 2 MHz, push-pull

    // 3. Configurar PB0 como salida alternativa (canal 3)
    GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0); // Limpiar configuraciones
    GPIOB->CRL |= (GPIO_CRL_MODE0_1 | GPIO_CRL_CNF0_1); // Salida alternativa, modo 2 MHz, push-pull

    // 4. Configurar el TIM3 para PWM
    TIM3->PSC = 1; // Prescaler (1 significa que la frecuencia del timer es 36 MHz)
    TIM3->ARR = PWM_RESOLUTION; // Periodo para 12 bits

    // 5. Configurar el canal 4 (PB1)
    TIM3->CCR4 = PWM_INITIAL_DUTY; // Ciclo de trabajo inicial para PB1
    TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // Modo PWM1 para el canal 4
    TIM3->CCMR2 |= TIM_CCMR2_OC4PE; // Habilitar preload
    TIM3->CCER |= TIM_CCER_CC4E; // Habilitar salida del canal 4

    // 6. Configurar el canal 3 (PB0)
    TIM3->CCR3 = PWM_INITIAL_DUTY; // Ciclo de trabajo inicial para PB0
    TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // Modo PWM1 para el canal 3
    TIM3->CCMR2 |= TIM_CCMR2_OC3PE; // Habilitar preload
    TIM3->CCER |= TIM_CCER_CC3E; // Habilitar salida del canal 3

    // 7. Habilitar el temporizador
    TIM3->CR1 |= TIM_CR1_CEN; // Habilitar el temporizador
}

// Ajustar el ciclo de trabajo para PB1 (canal 4)
void pwm_setDuty1(uint32_t duty) {
    TIM3->CCR4 = duty; // Ajustar el ciclo de trabajo para PB1
}

// Ajustar el ciclo de trabajo para PB0 (canal 3)
void pwm_setDuty2(uint32_t duty) {
    TIM3->CCR3 = duty; // Ajustar el ciclo de trabajo para PB0
}

void pwm_stop() {
    // 1. Deshabilitar la salida del canal 4
    TIM3->CCER &= ~TIM_CCER_CC4E; // Deshabilitar la salida del canal 4

    // 2. Detener el temporizador
    TIM3->CR1 &= ~TIM_CR1_CEN; // Deshabilitar el temporizador

    // 3. Opcional: Restablecer el valor de CCR4 a 0 (o cualquier otro valor deseado)
    TIM3->CCR4 = PWM_INITIAL_DUTY; // Establecer el ciclo de trabajo a 0
}

void pwm_restart(uint32_t duty) {
    // 1. Habilitar el temporizador
    TIM3->CR1 |= TIM_CR1_CEN; // Habilitar el temporizador

    // 2. Habilitar la salida del canal 4
    TIM3->CCER |= TIM_CCER_CC4E; // Habilitar la salida del canal 4

    // 3. Configurar el ciclo de trabajo (opcional, si deseas establecer un nuevo valor)
    TIM3->CCR4 = duty; // Restablecer el ciclo de trabajo inicial (ajustar según sea necesario)
}

#endif