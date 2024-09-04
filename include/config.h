/*
 *                      BLUE PILL PINOUT
 *
 *                  ,----------------------,
 *                  |o|PB12 | USB |   GND|o|          
 *   ENCODER_PIN_A  |o|PB13 '-----'   GND|o|  
 *   ENCODER_PIN_B  |o|PB14          3.3v|o|  
 *      BUTTON_PIN  |o|PB15          NRST|o|
 *                  |o|PA8           PB11|o|
 *                  |o|PA9           PB10|o|  
 *                  |o|PA10           PB1|o|  PWM_PIN [PWM_PIN1]
 *                  |o|PA11           PB0|o|  [PWM_PIN2]
 *                  |o|PA12           PA7|o|  
 *                  |o|PA15    /\     PA6|o|  
 *                  |o|PB3    /  \    PA5|o|  IBATT_SENSE_PIN
 * LCD_BKLIGHT_PIN  |o|PB4    \  /    PA4|o|  VBATT_SENSE_PIN
 *     LCD_RST_PIN  |o|PB5     \/     PA3|o|  FET_TEMP_SENSE_PIN
 *      LCD_CS_PIN  |o|PB6            PA2|o|
 *      LCD_DC_PIN  |o|PB7  =======   PA1|o|  BUZZER_PIN
 *     LCD_DIN_PIN  |o|PB8  =======   PA0|o|  COOLER_FAN_PIN
 *    LCD_SCLK_PIN  |o|PB9           PC15|o|
 *                  |o|5V            PC14|o|
 *                  |o|GND  . . . .  PC13|o|
 *                  |o|3.3V =======  VBAT|o|
 *                  '_'_____|_|_|_|______'_'
 *                          | | | |
 * 
 * 
 * 
 */



#ifndef _CONFIG_H
#define _CONFIG_H

// LCD
#define LCD_SCLK_PIN        PB9
#define LCD_DIN_PIN         PB8
#define LCD_DC_PIN          PB7
#define LCD_CS_PIN          PB6
#define LCD_RST_PIN         PB5
#define LCD_BKLIGHT_PIN     PB4

// ENCODER and BUTTTON
#define ENCODER_PIN_A      PB13
#define ENCODER_PIN_B      PB14
#define BUTTON_PIN         PB15
#define ENCODER_VCC_PIN      -1 // Uso VCC de la placa
#define ENCODER_STEPS         4
#define ENCODER_CENTRAL_PIN_TO_VCC false // se usa pin central conectado a GND

// BUZZER
#define BUZZER_PIN          PA1 // Canal 2 de TIM2

// COOLER FAN
#define COOLER_FAN_PIN      PA0

//MOSFET TEMPERATURE
#define FET_TEMP_SENSE_PIN  PA3
#define FET_MAX_TEMP        100 //ºC maximos
#define FET_MIN_TEMP         30 //ºC minimo para encender el cooler

// PWM
#define PWM_PIN             PB1 // Canal 4 de TIM3
//#define FREQUENCY         10000 // 10KHz
#define PWM_INITIAL_DUTY      0 // 0%
#define PWM_RESOLUTION     4095 // Resolución de 12 bits

// VBATT IBATT
#define VBATT_SENSE_PIN     PA4
#define IBATT_SENSE_PIN     PA5

//PID
#define RESOLUTION PWM_RESOLUTION
#endif