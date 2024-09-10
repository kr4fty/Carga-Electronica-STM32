#include <Arduino.h>
#include <PID_v1.h>
#include "config.h"
#include "encoder.h"
#include "display.h"
#include "pwm.h"
#include "sound.h"
#include "tclock.h"
#include "constantes.h"
#include "coolerfan.h"
#include "conversion.h"

int dutyCycle = 0;

double Setpoint, Input, Output; // Parametro PID
//Specify the links and initial tuning parameters
double Kp=KP, Ki=KI, Kd=KD;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
unsigned long windowStartTime;

double vBattRawOld, iBattRawOld;
uint16_t iAdcOffset;  // Lectura del ADC medida en vacio (0 A)
float AdcRaw_1A;      // Lectura del ADC midiendo 1A

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  
  lcd_init();
  encoder_init();
  pwm_init();
  coolerFan_init();
  tone_init();

  myPID.SetOutputLimits(VGS_THRESHOLE, 4095);
  myPID.SetSampleTime(PID_WINDOW_SIZE);  //sets the period, in Millisecond

  // Lectura inicial de los sensores
  vBattRawOld = analogRead(VBATT_SENSE_PIN)-.01; // le resto un pequeño valor para que imprima la tension cuando
  iBattRawOld = analogRead(IBATT_SENSE_PIN);

  // Entrando a modo Calibracion. Medimos una corriente de 1A para luego aplicar relacion
  if(isButtonDown())
  {
    lcd_printCalibration();
    delay(2000);
    while(!encoder.isEncoderButtonClicked()){
      AdcRaw_1A = analogRead(IBATT_SENSE_PIN);
      AdcRaw_1A = iBattRawOld + MU * (AdcRaw_1A - iBattRawOld);
      iBattRawOld = AdcRaw_1A;
      delay(5);
    }
    AdcRaw_1A -= 512;
  }
  else{
    iAdcOffset = IADCOFFSET;
    AdcRaw_1A = ADCRAW_1A;
  }

  windowStartTime = millis();

  encoder.setEncoderValue(adcToDutycycle(ADCRAW_1A)); // Seteo por defecto a 1A
  dutyCycle = encoder.readEncoder();
  Setpoint = dutycycleToADC(dutyCycle);

  tone(BUZZER_PIN, 434, 100);
  lcd_printBaseFrame();
}

bool isPowerOn = false; // TRUE: en funcionamiento, False: no ejecutandose
double vBattRaw, iBattRaw; // Valores Leidos en los ADC
float iIn, vIn, wIn, totalmAh, totalWh; // Valores actuales de la V, I y Wh
bool wasVUpdated, wasIUpdated, wasXhUpdated=true; //True: Si los valores cambiaron, para re imprimir
uint16_t mosfetTempRaw, oldMofetTempRaw;
float mosfetTemp; 
long timeToUpdateDisplay=millis()+DISPLAY_UPDATE_WINDOW;
unsigned long powerStateMessageTime, showMessageDuringThisTime = 2000; // 2 seg
bool printStatusMessage= true;
bool isItOverheating=false;
bool isPrintTime=true;
bool isTheSetpointUpdated; // Para tener prioridad al mostrar nuevo setpoint
bool printTinySetpoint=true;
unsigned long timeToPrintNewSetpoint, windowNewSetpoint=1000;
double Output2; // Contiene el duty del segundo MOSFET. Varia con el setpoint

void loop() {
  /***************************************************************************/
  /*                            SETEO DE CORRIENTE                           */
  /***************************************************************************/
  if (encoder.encoderChanged())
  {
    dutyCycle = encoder.readEncoder();
    Output2 = VGS_THRESHOLE + (dutyCycle-VGS_THRESHOLE)/2;
    if(isPowerOn){
      lcd_printNewSetpoint(dutyCycleToAmpere(dutyCycle));
      isTheSetpointUpdated = true;
    }
    else{
      printTinySetpoint = true;
    }

    Setpoint = dutycycleToADC(dutyCycle);
    
    tone(BUZZER_PIN, 600, 10);

    timeToPrintNewSetpoint = millis() + windowNewSetpoint;
  }
  if(isTheSetpointUpdated && (millis()>timeToPrintNewSetpoint)){
      isTheSetpointUpdated = false;
      printStatusMessage= true;

      tone(BUZZER_PIN, 7000, 20);
      delay(75);
      tone(BUZZER_PIN, 7000, 80);
  }

  if (encoder.isEncoderButtonClicked()){
    isPowerOn = not isPowerOn;

    // ENCENDIDO
    if(isPowerOn){
      lcd_printPowerOnMessage();
      tone(BUZZER_PIN, 4000, 50);
      delay(20);
      tone(BUZZER_PIN, 4500, 50);
      delay(20);
      tone(BUZZER_PIN, 5000, 50);
      
      myPID.SetMode(AUTOMATIC); // Encendemos el PID

      coolerFan_powerOn();      
    }
    // APAGADO
    else{
      lcd_printPowerOffMessage();
      
      tone(BUZZER_PIN, 436, 100);
      tone(BUZZER_PIN, 200, 150);

      pwm_setDuty1(0);
      pwm_setDuty2(0);

      myPID.SetMode(MANUAL);  // Apagamos el PID

      coolerFan_powerOff();
    }
    printStatusMessage = true;
    powerStateMessageTime = millis() + showMessageDuringThisTime;

    digitalWrite(LED, digitalRead(LED)?LOW:HIGH);
  }  
  // FIN CONFIGURACION DE CORRIENTE DE CARGA

  /***************************************************************************/
  /*           CHEQUEO Y CONTROL DE LA TEMPERATURA EN EL MOSFET              */
  /***************************************************************************/
  mosfetTempRaw = analogRead(FET_TEMP_SENSE_PIN);
  // Calculo la temperatura en el mosfet
  //mosfetTemp = mosfetTempRaw;
  mosfetTemp = 23;
  
  // Si supera cierta Temp maxima enciendo el Cooler
  if(mosfetTemp>FET_MIN_TEMP){
    // Activo el cooler
    //coolerFan_powerOn();
  }else{
    // Desactivo el cooler
    //coolerFan_powerOff();
  }

  // Si es mayor Apago la carga
  if(mosfetTemp>FET_MAX_TEMP){
    // Detenemos la salida PWM
    //pwm_stop();

    isItOverheating = true;

    // Emito un Sonido de alerta
    tone(BUZZER_PIN, 2000, 50);
    delay(100);
    tone(BUZZER_PIN, 2000, 50);
    delay(100);
    tone(BUZZER_PIN, 2000, 50);

  }
  // FIN TEMPERATURA EN EL MOSFET

  /***************************************************************************/
  /*                                  PID                                    */
  /***************************************************************************/
  vBattRaw = analogRead(VBATT_SENSE_PIN);
  // Filtro de Wiener, Adaptativo
  vBattRaw = vBattRawOld + MU * (vBattRaw - vBattRawOld);  
  // Calulo de vIn
  vIn = vBattRaw/ADCRAW_1V;

  if(vBattRaw!=vBattRawOld){// si cambio V entonces actualizo valor en el LCD
    wasVUpdated = true;
    vBattRawOld = vBattRaw;
  }
  
  iBattRaw = (double)analogRead(IBATT_SENSE_PIN);
  // Filtro de Wiener, Adaptativo
  iBattRaw = iBattRawOld + MU * (iBattRaw - iBattRawOld);
  // Calculo iIn
  iIn = (iBattRaw - iAdcOffset) / (ADCRAW_1A - iAdcOffset);

  if(iBattRaw!=iBattRawOld){ // si cambio I entonces actualizo valor en el LCD
    if(isPowerOn)
      wasIUpdated = true;
    else
      iAdcOffset = iBattRaw; // Corriente medida, valores de ADC, a 0A
    iBattRawOld = iBattRaw;
  }
  
  // CALCULO PID
  Input = iBattRaw;
  if(myPID.Compute()&&isPowerOn)
  {
    if(iIn<1){ // si es menor a 1A solo trabaja un MOSFET
      pwm_setDuty1(Output);
      pwm_setDuty2(0);
    }
    else{
      pwm_setDuty1(Output);
      pwm_setDuty2(Output2); // Valor fijo a la mitad del setpoint
    }
    // Calculo wIn
    wIn = vIn * iIn;
    // Wh
    totalWh += (wIn / 360000.0);  // PID_WINDOW_SIZE / 3600000.0 = 1/360000.0
    // Ah
    totalmAh += (iIn / 360.0);    // PID_WINDOW_SIZE / 3600000.0 * 1000 = 1/360.0

    wasXhUpdated = true;
  } 
  // FIN PID

  /***************************************************************************/
  /*                     Actuzalizo Tiempo Transcurrido                      */
  /***************************************************************************/
  currentMillis = millis();  // Obtiene el tiempo actual

  // Comprueba si ha pasado el intervalo de 1 segundo
  if (currentMillis - previousMillis >= TIME_1SEG) {
    previousMillis = currentMillis;  // Guarda el tiempo actual
    if(isPowerOn){ // Actuzalizar el Tiempo solo si esta en funcionamiento
      clock_update();
      isPrintTime = true;
    }
  }
  // FIN RELOJ

  /***************************************************************************/
  /*                            Refresco la Pantalla                         */
  /***************************************************************************/
  if(millis()>timeToUpdateDisplay && !isTheSetpointUpdated){
    // Se muestra el nuevo estado del dispositivo, solo por 2seg
    if(printStatusMessage){
      if(millis()>powerStateMessageTime){
        printStatusMessage = false;
        // Reimprimo toda la pantalla
        lcd_printBaseFrame();
        wasXhUpdated = true;
        wasVUpdated = true;
        wasIUpdated = true;
        isPrintTime = true;
      }
    }
    else if(isItOverheating){
      // Imprimir mensaje por Interrupcion de sobre-temperatura
      lcd_printOverTemperatureMessage();
    }else{
      // actualizo el valor de la temperatura en el MOSFET
      if(mosfetTempRaw!=oldMofetTempRaw){
        lcd_printTemperature(mosfetTemp);

        oldMofetTempRaw = mosfetTempRaw;
      }
      // Imprimir tiempo transcurrido
      if(isPrintTime){
        lcd_printTime(clock_get_hours(), clock_get_minutes(), clock_get_seconds());
        isPrintTime = false;
      }
    }

    // Imprimo los Watts-Hora y Ampere-Hora
    if(wasXhUpdated){      
      lcd_printWattHour(totalWh);
      lcd_printAmpHour(totalmAh);

      wasXhUpdated = false;
    }

    // Imprimo Tension y Corriente
    if(wasVUpdated){
      // Print Vin
      lcd_printVin(vIn);
      wasVUpdated = false;
    }
    if(isPowerOn){ // Solo muestro la corriente cuando esta encendido. Caso contrario, el Setpoint
      if(wasIUpdated){
        // Print Iin
        lcd_printIin(iIn);
        wasIUpdated = false;
      }
    }
    else{
      lcd_printTinyNewSetpoint(dutyCycleToAmpere(dutyCycle));
      printTinySetpoint = false;
    }

    lcd_display();
    
    timeToUpdateDisplay = millis()+DISPLAY_UPDATE_WINDOW;
  }  
  // FIN UPDATE DISPLAY
}