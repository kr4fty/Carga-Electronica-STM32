#include <Arduino.h>
#include <PID_v1.h>
#include "config.h"
#include "encoder.h"
#include "display.h"
#include "pwm.h"
#include "sound.h"
#include "tclock.h"

int dutyCycle = 0;

double Setpoint, Input, Output; // Parametro PID
//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 200;
unsigned long windowStartTime;

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  
  lcd_init();
  encoder_init();
  pwm_init();
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
}

bool isPowerOn = false;
double vBattRaw, iBattRaw;
double vBattRawOld, iBattRawOld;
float iIn, vIn;
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
    pwm_setDuty(dutyCycle);                     // para pruebas
    dutyCycle = encoder.readEncoder();          // para pruebas

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
    }
    // APAGADO
    else{
      lcd_printPowerOffMessage();
      //pwm_setDuty(0);
      tone(BUZZER_PIN, 436, 100);
      tone(BUZZER_PIN, 200, 150);
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
  mosfetTemp = random(100);
  
  // Si supera cierta Temp maxima enciendo el Cooler
  if(mosfetTemp>FET_MIN_TEMP){
    // Activo el cooler
    digitalWrite(COOLER_FAN_PIN, HIGH);
  }else{
    // Desactivo el cooler
    digitalWrite(COOLER_FAN_PIN, LOW);
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
  vBattRaw = (double)((_X_V*vBattRawOld + vBattRaw)/(_X_V+1));
  
  
  iBattRaw = analogRead(IBATT_SENSE_PIN);
  iBattRaw = (double)((_X_I*iBattRawOld + iBattRaw)/(_X_I+1));
  

  // Conversion ADC a valores de Vin o Iin
  vIn = random(2000)/100.0;
  iIn = random(500)/100.0;

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
    if(vBattRawOld != vBattRaw){
      // Print Vin
      lcd_printVin(vIn);
      vBattRawOld = vBattRaw;
    }
    if(iBattRawOld != iBattRaw){
      // Print Iin
      lcd_printIin(iIn);
      iBattRawOld = iBattRaw;
    }

    
    lcd_display();
    
    timeToUpdateDisplay = millis()+DISPLAY_UPDATE_WINDOW;
  }  
  // FIN UPDATE DISPLAY
}