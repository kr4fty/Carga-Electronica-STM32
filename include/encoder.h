#ifndef _ENCODER_H
#define _ENCODER_H
#include <AiEsp32RotaryEncoder.h>
#include "config.h"
#include "constantes.h"

AiEsp32RotaryEncoder encoder = AiEsp32RotaryEncoder(ENCODER_PIN_A,
                                                    ENCODER_PIN_B,
                                                    BUTTON_PIN,
                                                    ENCODER_VCC_PIN,
                                                    ENCODER_STEPS,
                                                    ENCODER_CENTRAL_PIN_TO_VCC);

void readEncoderISR()
{
	encoder.readEncoder_ISR();
}

void encoder_init()
{
    encoder.begin();
    encoder.setup(readEncoderISR);
    bool circleValues = false;
    encoder.setBoundaries(0, 1000, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
    //encoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
    encoder.setAcceleration(150); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
    encoder.setEncoderValue(0);
}

void encoder_setBasicParameters(long minValue, long maxValue, bool circleValues=false , long newValue=0, long accel=150)
{
    encoder.setBoundaries(minValue, maxValue, circleValues);
    encoder.setEncoderValue(newValue);
    encoder.setAcceleration(accel);
}

enum ClickType {
    NO_CLICK,       // No hay pulsacion
    SHORT_CLICK,    // Pulsacion corta
    LONG_CLICK,     // Pulsacion larga
    DOUBLE_CLICK    // Doble pulsacion
};

bool wasButtonDown    = false; // se presiono el boton
bool checkDoubleClick = false; // si detecta una pulsacion rapida es posible que venga una doble pulsacion
bool longClickDetected;        // True: se forzo pulzacion larga, ignorar click al soltar el boton
uint8_t clickCounter=0; // 
//paramaters for button
unsigned long shortPressAfterMiliseconds = 250; // Tiempo de espera para una pulsación corta
unsigned long longPressAfterMiliseconds = 1000; // ¿Cuánto tiempo se considera una pulsación larga?
unsigned long lastTimeButtonDown = millis();           // tiempo en el que duro la pulsacion
unsigned long lastTimeButtonClick;
unsigned long maxTimeBetween2Events = 300;

bool isButtonDown()
{
    return digitalReadFast(digitalPinToPinName(BUTTON_PIN))? false : true;
}

bool isButtonUp()
{
    return digitalReadFast(digitalPinToPinName(BUTTON_PIN))? true : false;
}

// Devuelve el tipo de pulsacion detectatado. Caso contrario, un NO_CLICK
uint8_t isButtonClicked()
{
    unsigned long timeDiff = millis() - lastTimeButtonDown;
    uint8_t key = NO_CLICK;
    bool currentButtonState = digitalReadFast(digitalPinToPinName(BUTTON_PIN));
    
    // Boton presionado, button down.
    if (!currentButtonState){
        // Lo hacemos por unica vez en cada ciclo click, button down a button up
        if(!wasButtonDown) {
            wasButtonDown = true;   //  No se ingresara aqui hasta el proximo inicio de click
            lastTimeButtonDown = millis();  // Iniciar el temporizador
            longClickDetected = false;
            //Serial.printf("button down\n");
        }
        // Si el boton permanece presionado, chequeo por pulzacion larga
        else{
            // Considerar como pulsación larga si se mantuvo presionado más de 'longPressAfterMiliseconds'
            if(timeDiff > longPressAfterMiliseconds){
                if(!longClickDetected){ // el boton sigue presionado?
                    key = LONG_CLICK;
                    longClickDetected = true;
                    //Serial.printf("Pulsacion Larga, Duracion: %d, Key:%d\n", timeDiff, key);
                } 
                // se ha detectado una pulsacion larga, por lo que no tiene sentido ahora preguntar por pulsacion corta
                return key;
            }
        }
    }
    // Boton liberado, se hizo click
    else{
        if(longClickDetected){ // Si hubo una pulsacion larga, reseteo valores para ignorar el click al soltar el boton
            longClickDetected = false;
            wasButtonDown = false;
        }
        else if (wasButtonDown){
            //Serial.printf("button up\n");
            clickCounter++;
            //Serial.printf("Clicks = %d Time: %d ", clickCounter, timeDiff);
            wasButtonDown = false;

            if (timeDiff > shortPressAfterMiliseconds){ // Se detecto pulsacion corta
                //Serial.printf("\n");
            } else { // Se detecto pulsacion corta y rapida. Posible intento de doble click?
                checkDoubleClick = true;
                //Serial.printf("Fast click\n");
            }
        }
    }
    // Pasado un tiempo maxTimeBetween2Events, veo si fue un click corto o un doble click rapido
    if(timeDiff>maxTimeBetween2Events && clickCounter){
        if(clickCounter==1){ // Solo se detecto una pulsacion, entonces es una pulsacion corta
            key = SHORT_CLICK;
            //Serial.printf("Pulsacion Corta\n");

        }
        else if(clickCounter>1){ // Se detecto varias pulsaciones, entonces es un doble click (podria haber 3 o mas :P)
            if(checkDoubleClick){
                key = DOUBLE_CLICK;
                //Serial.printf("Doble Pulsacion\n");
            }
        }
        
        clickCounter = 0;
        checkDoubleClick = false;
    }

    return key;
}


#endif