#ifndef _CALIBRATE_H
#define _CALIBRATE_H

#include <Arduino.h>
#include "pwm.h"
#include "encoder.h"
#include "display.h"
#include "myEeprom.h"

extern double vBattRawOld, iBattRawOld;
extern double iAdcOffset;  // Lectura del ADC medida en vació (0 A)
extern double AdcRaw_1A;   // Lectura del ADC midiendo 1A
extern double Adc1aDiff;   // Contiene la diferencia en valores de ADC, entre 1A y 0A

// Mostrar los parámetros de referencia
void calibration_show()
{
    lcd_printCalibrationParameters(iAdcOffset, AdcRaw_1A);
    while(!isButtonClicked());
}

// Se encarga de de realizar las muestras para obtener los parámetros de referencia
void calibration_calibrate()
{
    lcd_printCalibration();

    iBattRawOld = analogRead(IBATT_SENSE_PIN);
    for(int i=0; i<5000; i++){
        iAdcOffset = analogRead(IBATT_SENSE_PIN);
        iAdcOffset = iBattRawOld + MU * (iAdcOffset - iBattRawOld);
        iBattRawOld = iAdcOffset;
        if(!(i%50)){
            lcd_printIraw(iAdcOffset);
        }
        delay(1);
    }
    encoder_setBasicParameters(0, 4095, false, 2400);
    long encoderValue = encoder.readEncoder();

    pwm_setDuty1(encoderValue);
    pwm_setDuty2(0);

    while(!isButtonClicked()){
        if (encoder.encoderChanged()){
            encoderValue = encoder.readEncoder();
            pwm_setDuty1(encoderValue);
        }

        AdcRaw_1A = analogRead(IBATT_SENSE_PIN);
        AdcRaw_1A = iBattRawOld + MU * (AdcRaw_1A - iBattRawOld);
        iBattRawOld = AdcRaw_1A;
        lcd_printIraw(AdcRaw_1A, COLOR_WB);
        delay(1);
    }
    pwm_setDuty1(0);
    Adc1aDiff = AdcRaw_1A - iAdcOffset;

    calibration_show();

    // Guardo en EEPROM
    myEeprom_writeDouble(C_0A_CAL_ADDR,  iAdcOffset);
    myEeprom_writeDouble(C_1A_CAL_ADDR,  AdcRaw_1A);
}

// Leo los parámetro desde la EEPROM
void calibration_readParameters()
{
    // Recupero Valores desde la EEPROM
    iAdcOffset = myEeprom_readDouble(C_0A_CAL_ADDR);
    AdcRaw_1A = myEeprom_readDouble(C_1A_CAL_ADDR);
    Adc1aDiff = AdcRaw_1A - iAdcOffset;

    // Si hay algún valor medio raro, utilizo los de "constantes.h"
    if(((iAdcOffset<(512*.95)) || (iAdcOffset>1023)) || ((AdcRaw_1A<(512)) || (AdcRaw_1A>1023)) || (iAdcOffset == AdcRaw_1A)){
        iAdcOffset = IADCOFFSET;
        AdcRaw_1A = ADCRAW_1A;
        Adc1aDiff = ADC_1A_DIFF;
    }
}

// Reinicia los parámetro a valores por defecto
void calibration_resetParameters()
{
    iAdcOffset = IADCOFFSET;
    AdcRaw_1A = ADCRAW_1A;
    Adc1aDiff = ADC_1A_DIFF;
}

#endif