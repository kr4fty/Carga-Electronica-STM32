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
#include "notifications.h"

long encoderValue = 0;

double Setpoint, Input, Output; // Parametro PID
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, KP_AGG, KI_AGG, KD_AGG, DIRECT);
unsigned long windowStartTime;

double vBattRawOld, iBattRawOld;

#ifdef DEBUG
unsigned long nextTime, startTime;
double    Time;
#define WINDOW_CAPTURE  40
#define WINDOW_10SEG 10000
#endif

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  #endif
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  
  lcd_init();
  encoder_init();
  pwm_init();
  coolerFan_init();
  tone_init();

  myPID.SetOutputLimits(ADC_VGS_THRESHOLE, 4095);
  myPID.SetSampleTime(PID_WINDOW_SIZE);  //sets the period, in Millisecond
  myPID.SetMode(AUTOMATIC); // Encendemos el PID

  // Lectura inicial de los sensores
  vBattRawOld = analogRead(VBATT_SENSE_PIN);
  iBattRawOld = analogRead(IBATT_SENSE_PIN);

  // Entrando a modo Calibracion. Medimos una corriente de 1A para luego aplicar relacion
  if(isButtonDown())
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
    encoder.setBoundaries(0, 4095, false);
    encoder.setEncoderValue(ampereToDutycycle(C_1A));
    encoderValue = encoder.readEncoder();
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
    encoder.setBoundaries(0, 1000, false);
    shortClick = false;
    longClick = false;
  }
  else{
    iAdcOffset = IADCOFFSET;
    AdcRaw_1A = ADCRAW_1A;
    Adc1aDiff = ADC_1A_DIFF;
  }

  windowStartTime = millis();

  encoder.setEncoderValue(C_1A*100); // Seteo por defecto a 1A
  Setpoint = 0;

  tone(BUZZER_PIN, 434, 100);
  lcd_printBaseFrame();
}

bool isPowerOn = false, isPowerOnOld; // TRUE: en funcionamiento, False: no ejecutandose
double vRaw; // Valor crudo de la tencion de entrada, sin aplicar filtro, para detectar mas rapidamente la desconexion de la bateria
double vBattRaw, iBattRaw; // Valores Leidos en los ADC
bool batteryConnected=true;  // True si se detecto tension de sensado de bateria
float iIn, vIn, wIn, totalmAh, totalWh; // Valores actuales de la V, I y Wh
bool wasVUpdated=true, wasIUpdated=true, wasXhUpdated=true, wasTempUpdated=true; //True: Si los valores cambiaron, para re imprimir
uint16_t mosfetTempRaw, oldMofetTempRaw;
float mosfetTemp; 
long timeToUpdateDisplay=millis()+DISPLAY_UPDATE_WINDOW;
uint8_t notificationPriority; // Contiene la prioridad de notificaciones
uint16_t time_mseg; // Tiempo en mili segundos
bool forceRePrint; // Para volver a imprimir toda la pantalla
bool isItOverheating=false;
bool isPrintTime=true;
bool isTheSetpointUpdated; // Para tener prioridad al mostrar nuevo setpoint
bool printTinySetpoint=true;
unsigned long timeToPrintNewSetpoint, windowNewSetpoint=1000;
double Output2; // Contiene el duty del segundo MOSFET. Varia con el setpoint
float ampereSetpoint=C_1A; // Contiene el valor del Setpoint expresado en ampere

void loop() {
  /***************************************************************************/
  /*                    MEDION DE TENSION Y CORRIENTE                        */
  /***************************************************************************/
  vRaw = analogRead(VBATT_SENSE_PIN);
  // Filtro de Wiener, Adaptativo
  vBattRaw = vBattRawOld + MU * (vRaw - vBattRawOld);  
  // Calulo de vIn
  //vIn = vBattRaw/ADCRAW_1V; // falta de linealidad
  vIn = 0.01935*vBattRaw + 0.25; // Funcion obtenida por regresion lineal

  if(vBattRaw!=vBattRawOld){// si cambio V entonces actualizo valor en el LCD
    wasVUpdated = true;
    vBattRawOld = vBattRaw;
  }
  
  iBattRaw = (double)analogRead(IBATT_SENSE_PIN);
  // Filtro de Wiener, Adaptativo
  iBattRaw = iBattRawOld + MU * (iBattRaw - iBattRawOld);
  // Calculo iIn
  iIn = (iBattRaw - iAdcOffset) / Adc1aDiff;

  if(iBattRaw!=iBattRawOld){ // si cambio I entonces actualizo valor en el LCD
    if(isPowerOn){
      wasIUpdated = true;
    }
    else{
      iAdcOffset = iBattRaw; // Corriente medida, valores de ADC, a 0A
    }
    iBattRawOld = iBattRaw;
  }
  // FIN Mediones

  /***************************************************************************/
  /*                 ¿SE CONECTO LA FUENTE A LA ENTRADA?                     */
  /***************************************************************************/
  // Bateria/Fuente conectada y funcionando?
  if(vRaw > VBATT_MIN){
    // Bateria conectada
    if(!batteryConnected){
      // Solo mostramos una vez el mensaje
      notificationPriority = 3;
      time_mseg = 500;
      notification_add("BATT CONNECTED", notificationPriority, time_mseg, COLOR_BW);

      batteryConnected = true;
    }
  }
  else{ // NO SE DETECTO TENSION DE ENTRADA!!!
    if(batteryConnected){
      // Se muestra solo una vez y queda fijo hasta que no se cambie el estado
      notificationPriority = 3;
      notification_add("  NO BATTERY  ", notificationPriority, NO_TIME_LIMIT, COLOR_WB);

      // Emito un Sonido de alerta
      tone(BUZZER_PIN, 2000, 50);
      delay(50);
      tone(BUZZER_PIN, 2000, 50);
      delay(50);
      tone(BUZZER_PIN, 2000, 50);

      batteryConnected = false;
      if(isPowerOn){
        pwm_setDuty1(0);
        pwm_setDuty2(0);
        Setpoint = 0; // Lo seteo al valor de ADC para 0A
        //myPID.SetMode(MANUAL);  // Apagamos el PID
      }
    }
  }
  // FIN DETECCION DE TENSION DE ENTRADA

  /***************************************************************************/
  /*                            SETEO DE CORRIENTE                           */
  /***************************************************************************/
  if (encoder.encoderChanged())
  {
    encoderValue = encoder.readEncoder();
    ampereSetpoint = encoderValue/100.0;
    Output2 = ampereToDutycycle(ampereSetpoint*.5, MOSFET2);
    if(isPowerOn){ // Si esta encendido lo musetro en una ventana temporal
      lcd_printNewSetpoint(ampereSetpoint);
      isTheSetpointUpdated = true;
    }
    else{ // Si esta APAGADO lo muestro en lugar de la corriente medida
      printTinySetpoint = true;
    }

    if(isPowerOn){
      Setpoint = ampereToAdc(ampereSetpoint);
    }
    
    tone(BUZZER_PIN, 600, 10);

    timeToPrintNewSetpoint = millis() + windowNewSetpoint;
  }
  // Tiempo de muestra de la Ventana temporal que imprime el nuevo SETPOINT
  if(isTheSetpointUpdated && (millis()>timeToPrintNewSetpoint)){
      isTheSetpointUpdated = false;
      forceRePrint = true;
      batteryConnected = true;  // si estando encendido se desconecta y luego modifico el setpoint, no se reimprime la notificacion

      tone(BUZZER_PIN, 7000, 20);
      delay(75);
      tone(BUZZER_PIN, 7000, 80);
  }
  // FIN CONFIGURACION DE CORRIENTE DE CARGA

  /***************************************************************************/
  /*                           ENCENDIDO Y APAGADO                           */
  /***************************************************************************/

  // Deteccion de pulsacion de boton
  if(isButtonClicked()){
    //  HUBO UNA PULSACION LARGA
    if(longClick){ // Reiniciamos los contadores
      longClick = false;

      notificationPriority = 1;
      notification_add("RESET COUNTERS", notificationPriority);

      clock_resetClock();
      totalmAh = 0;
      totalWh = 0;

      tone(BUZZER_PIN, 1000,300);
    }

    // HUBO UN CLICK EN EL BOTON
    if (shortClick){
      shortClick = false;
      isPowerOn = not isPowerOn; // Cambia el estado anterior, de ENCENDIDO -> APAGADO y viceversa

      // ENCENDIDO
      if(isPowerOn){
        notificationPriority = 2;
        notification_add("   POWER ON   ", notificationPriority);
        
        tone(BUZZER_PIN, 4000, 50);
        delay(20);
        tone(BUZZER_PIN, 4500, 50);
        delay(20);
        tone(BUZZER_PIN, 5000, 50);
        
        Setpoint =ampereToAdc(ampereSetpoint);
        /*if(ampereSetpoint<1){
          myPID.SetTunings(KP_AGG, KI_AGG, KD_AGG);
        }
        else{
          myPID.SetTunings(KP_CNSTIVE, KI_CNSTIVE, KD_CNSTIVE);
        }*/
        //myPID.SetMode(AUTOMATIC); // Encendemos el PID

        coolerFan_powerOn();   
        
        #ifdef DEBUG
        startTime = millis();
        nextTime = startTime + WINDOW_CAPTURE;
        #endif
      }
      // APAGADO
      else{
        notificationPriority = 2;
        notification_remove(3);
        notification_add("   POWER OFF  ", notificationPriority);
        
        tone(BUZZER_PIN, 436, 100);
        tone(BUZZER_PIN, 200, 150);

        pwm_setDuty1(0);
        pwm_setDuty2(0);

        Setpoint = 0;
        //myPID.SetMode(MANUAL);  // Apagamos el PID

        coolerFan_powerOff();
      }

      digitalWrite(LED, isPowerOn?LOW:HIGH);
    }
  }
  // FIN BOTON

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

    if(!isItOverheating){ // Notificacion por exceso de temperatura, solo una vez
      
      if(isPowerOn){
        pwm_setDuty1(0);
        pwm_setDuty2(0);
        Setpoint = iAdcOffset; // Lo seteo al valor de ADC para 0A
        //myPID.SetMode(MANUAL);  // Apagamos el PID
      }

      isItOverheating = true;
      notificationPriority = 4;
      notification_addMini("TEMP MAX", notificationPriority, NO_TIME_LIMIT, COLOR_WB);

      // Emito un Sonido de alerta
      tone(BUZZER_PIN, 2000, 50);
      delay(100);
      tone(BUZZER_PIN, 2000, 50);
      delay(100);
      tone(BUZZER_PIN, 2000, 50);
    }

  }
  else{
    if(isItOverheating){
      isItOverheating = false;
      notificationPriority = 4;
      notification_removeMini(notificationPriority);
    }
  }
  if(mosfetTempRaw!=oldMofetTempRaw){
    wasTempUpdated = true;
    oldMofetTempRaw = mosfetTempRaw;
  }
  // FIN TEMPERATURA EN EL MOSFET

  /***************************************************************************/
  /*                                  PID                                    */
  /***************************************************************************/
  if(batteryConnected){
    // CALCULO PID
    Input = iBattRaw;
    if(myPID.Compute() && isPowerOn)
    {
      if(ampereSetpoint<1){ // si es menor a 1A solo trabaja un MOSFET
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
  }
  // FIN PID

  /***************************************************************************/
  /*                     Actuzalizo Tiempo Transcurrido                      */
  /***************************************************************************/
  currentMillis = millis();  // Obtiene el tiempo actual

  // Comprueba si ha pasado el intervalo de 1 segundo
  if (isPowerOn && batteryConnected && (currentMillis - previousMillis)>=TIME_1SEG) {
    previousMillis = currentMillis;  // Guarda el tiempo actual
    // Actuzalizar el Tiempo solo si esta en funcionamiento
    clock_update();
    isPrintTime = true;
  }
  // FIN RELOJ

  /***************************************************************************/
  /*                            Refresco la Pantalla                         */
  /***************************************************************************/

  if(millis()>timeToUpdateDisplay && !isTheSetpointUpdated){
    if(newNotification){
      if(notification_hasExpired()){
        // Fuerzo reimprimir toda la pantalla
        forceRePrint = true;

        /* Se coloca aquí porque, al reconectar la batería tras una desconexión,
         * se genera un sobreimpulso que puede dañar la fuente/batería. Por 
         * ello, espero a que desaparezca la notificación de 'conexión de 
         * batería' antes de reactivar la carga."
         */
        if(isPowerOn&&batteryConnected){
          Setpoint = ampereToAdc(ampereSetpoint);
        }
        batteryConnected = true;  // si estando encendido se desconecta y luego apago la carga, no se reimprime la pantalla
      }
    }
    else {
      // actualizo el valor de la temperatura en el MOSFET
      if(wasTempUpdated){
        lcd_printTemperature(mosfetTemp);
        wasTempUpdated = false;
      }
      // Imprimir tiempo transcurrido
      // Usando las mini notificaciones puedo mostrar la temperatura
      if(!newMiniNotificacion && isPrintTime){
        if(isPowerOn){
          lcd_printTime(clock_get_hours(), clock_get_minutes(), clock_get_seconds(), COLOR_WB);
        }
        else{
          lcd_printTime(clock_get_hours(), clock_get_minutes(), clock_get_seconds());
        }
        isPrintTime = false;
      }
    }

    if(forceRePrint){
      // Reimprimir toda la pantalla
      forceRePrint = false;
      lcd_printBaseFrame();
      wasXhUpdated = true;
      wasVUpdated = true;
      if(isPowerOn){
        wasIUpdated = true;
      }
      else{
        printTinySetpoint = true;
      }
      isPrintTime = true;
      wasTempUpdated = true;
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
    else {
      if(printTinySetpoint)
        lcd_printTinyNewSetpoint(ampereSetpoint);
      printTinySetpoint = false;
    }

    lcd_display();
    
    timeToUpdateDisplay = millis()+DISPLAY_UPDATE_WINDOW;
  }  
  // FIN UPDATE DISPLAY

  #ifdef DEBUG
  /***************************************************************************/
  /*                   Envio Datos por el puerto Serie                       */
  /***************************************************************************/
  if(millis()>nextTime && isPowerOn && millis()<(startTime+WINDOW_10SEG)){
   
    Time = (millis()-startTime);
    // Para graficar y obtener datos utilizando serial_port_plotter
    //sprintf(buff, "$%d %ld %d;", (int)encoderValue, Time, (int)iBattRaw);
    Serial.printf("$%d %ld %d;", encoderValue, (unsigned long)Time, (int)iBattRaw);
    nextTime = millis() + WINDOW_CAPTURE;
  }
  if(isPowerOn && millis()>(startTime+WINDOW_10SEG)){
    isPowerOn = false;
    notificationPriority = 2;
    notification_remove(3);
    notification_add("   POWER OFF  ", notificationPriority);

    tone(BUZZER_PIN, 436, 100);
    tone(BUZZER_PIN, 200, 150);

    pwm_setDuty1(0);
    pwm_setDuty2(0);

    Setpoint = 0;
  }
  // END Serial port
  #endif
}