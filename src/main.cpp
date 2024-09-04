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

int dutyCycle = 0;

double Setpoint, Input, Output; // Parametro PID
//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 200;
unsigned long windowStartTime;

double vBattRawOld, iBattRawOld;
uint16_t ADCOffset; // Lectura del ADC medida en vacio (0 A)
float AdcRaw_1A;    // Lectura del ADC midiendo 1A

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  
  lcd_init();
  encoder_init();
  pwm_init();
  coolerFan_init();
  tone_init();
  
  //initialize the variables we're linked to
  Setpoint = 0;
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, RESOLUTION);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  //sets the period, in Millisecond
  myPID.SetSampleTime(100);

  windowStartTime = millis();

  tone(BUZZER_PIN, 434, 100);
  lcd_printBaseFrame();

  // Lectura inicial de los sensores
  vBattRawOld = analogRead(VBATT_SENSE_PIN)-.01; // le resto un pequeño valor para que imprima la tension cuando
  iBattRawOld = analogRead(IBATT_SENSE_PIN);

  //Entrando a modo Calibracion. ADC medido con una corriente de 1A
  if(isButtonDown())
  {
    lcd_printCalibration();
    delay(2000);
    while(!encoder.isEncoderButtonClicked()){
      AdcRaw_1A = analogRead(IBATT_SENSE_PIN);
      //AdcRaw_1A = (double)((_X_I*iBattRawOld + AdcRaw_1A)/(_X_I+1));
      AdcRaw_1A = ALPHA_I*AdcRaw_1A + (1-ALPHA_I)*iBattRawOld;
      iBattRawOld = AdcRaw_1A;
      delay(5);
    }
    AdcRaw_1A -= 512;
  }
  else{
    ADCOffset = ADCOFFSET;
    AdcRaw_1A = ADCRAW_1A;
  }
}

bool isPowerOn = false;
double vBattRaw, iBattRaw;
float iIn, vIn, iOut, vOut;
uint16_t mosfetTempRaw, oldMofetTempRaw;
float mosfetTemp;
long timeToUpdateDisplay=millis()+DISPLAY_UPDATE_WINDOW;
unsigned long powerStateMessageTime, showMessageDuringThisTime = 2000; // 2 seg
bool printStatusMessage= false;
bool isItOverheating=false;
bool isPrintTime;
bool isThereNewSetpointValue; // Para tener prioridad al mostrar nuevo setpoint
unsigned long timeToPrintNewSetpoint, windowNewSetpoint=1000;

void loop() {
  /***************************************************************************/
  /*                            SETEO DE CORRIENTE                           */
  /***************************************************************************/
  if (encoder.encoderChanged())
  {
    Setpoint = encoder.readEncoder();

    /*lcd.setTextSize(1);
    lcd.setCursor(LCDWIDTH/2-4, LCDHEIGHT/2-4); // para pruebas
    lcd.print(dutyCycle);                       // para pruebas
    updateDisplay = true;                       // para pruebas*/
    dutyCycle = Setpoint;                       // para pruebas
    pwm_setDuty(dutyCycle);                     // para pruebas

    lcd_printNewSetpoint(Setpoint);
    tone(BUZZER_PIN, 600, 10);
    isThereNewSetpointValue = true;

    timeToPrintNewSetpoint = millis() + windowNewSetpoint;
  }
  if(isThereNewSetpointValue && (millis()>timeToPrintNewSetpoint)){
      isThereNewSetpointValue = false;
      lcd_printBaseFrame();
      if(isPowerOn){
        tone(BUZZER_PIN, 7000, 20);
        delay(75);
        tone(BUZZER_PIN, 7000, 80);
      }
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

      coolerFan_powerOn();      
    }
    // APAGADO
    else{
      lcd_printPowerOffMessage();
      //pwm_setDuty(0);
      tone(BUZZER_PIN, 436, 100);
      tone(BUZZER_PIN, 200, 150);

      coolerFan_powerOff();
    }
    printStatusMessage = true;
    powerStateMessageTime = millis() + showMessageDuringThisTime;

    digitalWrite(LED, digitalRead(LED)?LOW:HIGH);
    //digitalWrite(COOLER_FAN_PIN, digitalRead(COOLER_FAN_PIN)?LOW:HIGH);
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
  //vBattRaw = (double)((_X_V*vBattRawOld + vBattRaw)/(_X_V+1));
  // Filtro EMA
  //vBattRaw = ALPHA_V*vBattRaw + (1-ALPHA_V)*vBattRawOld;
  // Filtro de Wiener, Adaptativo
  vBattRaw = vBattRawOld + MU * (vBattRaw - vBattRawOld);

  vBattRawOld = vBattRaw;

  
  iBattRaw = (double)analogRead(IBATT_SENSE_PIN);
  //iBattRaw = (double)((_X_I*iBattRawOld + iBattRaw)/(_X_I+1));
  // Filtro EMA
  //iBattRaw = ALPHA_I*iBattRaw + (1-ALPHA_I)*iBattRawOld;
  // Filtro de Wiener, Adaptativo
  iBattRaw = iBattRawOld + MU * (iBattRaw - iBattRawOld);
  iBattRawOld = iBattRaw;
  

  // Conversion ADC a valores de Vin o Iin
  //vIn = random(1000)/100.0;
  //iIn = random(500)/100.0;

  // CALCULO PID
  Input = iBattRaw;
  myPID.Compute();
  //dutyCycle = Output;
  if (millis() > windowStartTime+WindowSize)
  {
    windowStartTime += WindowSize;
    // Actuo en base al Output computado
    //pwm_setDuty(dutyCycle);
  } 
  // FIN PID

  /***************************************************************************/
  /*                     Actuzalizo Tiempo Transcurrido                      */
  /***************************************************************************/
  currentMillis = millis();  // Obtiene el tiempo actual

  // Comprueba si ha pasado el intervalo de 1 segundo
  if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;  // Guarda el tiempo actual
      isPrintTime = true;
      clock_update();
  }
  // FIN RELOJ

  /***************************************************************************/
  /*                            Refresco la Pantalla                         */
  /***************************************************************************/
  if(millis()>timeToUpdateDisplay && !isThereNewSetpointValue){
    // Se muestra el nuevo estado del dispositivo, solo por 2seg
    if(printStatusMessage){
      if(millis()>powerStateMessageTime){
        printStatusMessage = false;
        lcd_printBaseFrame();
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
    if( (vBattRawOld != vBattRaw) || (iBattRawOld != iBattRaw) ){
      lcd_printWattHour(vIn*iIn);
      lcd_printAmpHour(iIn);
    }
    
    // Imprimo Tension y Corriente
    //if(vBattRawOld != vBattRaw)
    {
      // Calculo Vin
      vOut = (double)(vBattRaw/1024.0)*V3_3;
      vIn = vOut/GAIN_V;
      // Print Vin
      lcd_printVin(vIn);
      //vBattRawOld = vBattRaw;
    }
    //if(iBattRawOld != iBattRaw)
    {
      // Calculo iIn
      //iOut = ((iBattRaw-ADCOffset)/1024.0) * ((V3_3 + VD)/GAIN_I);
      //iIn = (iOut / SENSIBILITY);
      iIn = (iBattRaw - ADCOffset) / ADCRAW_1A;

      // Print Iin
      lcd_printIin(iIn);
      //iBattRawOld = iBattRaw;
    }

    
    lcd_display();
    
    timeToUpdateDisplay = millis()+DISPLAY_UPDATE_WINDOW;
  }  
  // FIN UPDATE DISPLAY
}