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
static unsigned long    lastTimePressed = 0;

void encoder_init()
{
    encoder.begin();
    encoder.setup(readEncoderISR);
    bool circleValues = false;
    encoder.setBoundaries(VGS_THRESHOLE, 4095, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
    //encoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
    encoder.setAcceleration(250); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
    encoder.setEncoderValue(0);
}

bool isButtonDown()
{
    return digitalRead(BUTTON_PIN) ? false : true;
}

#endif