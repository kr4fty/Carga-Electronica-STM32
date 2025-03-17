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
#include "myMenu.h"
#include "calibrate.h"
#include "modes.h"
#include "control.h"

long encoderValue = 0;// Valor leído desde el encoder
float ampereSetpoint;// Contiene el valor del Setpoint expresado en Amperes
float powerSetpoint;// Contiene el valor de Setpoint expresado en Watts
float resistanceSetpoint; // Contiene el valor del Setpoint expresado en Ohmios
float vLimit=0;     // Valor de la tensión mínima de corte
double vBattRawOld; // Valor de lectura anterior del pin ADC que mide V
double iBattRawOld; // Valor de lectura anterior del pin ADC que mide I
double iAdcOffset;  // Lectura del ADC medida en vació (0 A)
double AdcRaw_1A;   // Lectura del ADC midiendo 1A
double Adc1aDiff;   // Contiene la diferencia en valores de ADC, entre 1A y 0A
float iIn, vIn, vInOld, wIn, totalmAh, totalWh; // Valores actuales de la V, I y Wh
uint8_t key; // 0: no click, 1: corta, 2: larga, 3: doble pulsación

double Setpoint, Input, Output; // Parámetro PID
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, KP_AGG, KI_AGG, KD_AGG, DIRECT);
unsigned long windowStartTime;

#ifdef DEBUG
unsigned long nextTime, startTime, actualTime;
#define WINDOW_CAPTURE  40
#define WINDOW_10SEG 10000
#endif

void setup() {
    #ifdef DEBUG
    Serial.begin(115200);
    delay(5000);
    Serial.println("Iniciando");
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

    // Entrando a modo Calibración. Medimos una corriente de 1A para luego aplicar relación
    if(isButtonDown())
    {
        calibration_calibrate();
        if(!iAdcOffset || !AdcRaw_1A){
            calibration_resetParameters();
        }
    }
    else{
        // Recupero los parámetros de calibración de la Corriente desde la EEPROM
        calibration_readParameters();
    }

    vLimit = V_0V;              // 0 Volt
    controlMode; // Por default esta en C_CONST_MODE
    
    ampereSetpoint = C_1A;      // 1 Ampere
    powerSetpoint = P_1W;       // 1 Watt
    resistanceSetpoint = R_10R; // 10 Ohm

    //lcd_printBaseFrame(controlMode);
    lcd_printSelectBaseFrame();

    windowStartTime = millis();

    encoder_setBasicParameters(0, 299, true, 1, 2); // Configuro para selección de parámetro a modificar

    lcd_printSelectedParam(1, controlMode, COLOR_WB); // resalto
            
    encoderValue = encoder.readEncoder();

    Setpoint = 0;

    // Inicio Proceso de control
    tone(BUZZER_PIN, 2600, 250);
}

bool isPowerOn = false; // TRUE: en funcionamiento
long encoderValueChanged; // True: el valor del encoder cambio
int8_t encoderDirection;
double vRaw; // Valor crudo de la tensión de entrada, sin aplicar filtro, para detectar mas rápidamente la desconexión de la batería
double vBattRaw, iBattRaw; // Valores Leidos en los ADC
bool batteryConnected=true;  // True si se detecto tensión de sensado de batería
bool wasVUpdated=true, wasIUpdated=true, wasXhUpdated=true, wasTempUpdated=true; //True: Si los valores cambiaron, para re imprimir
bool wasCtrlModeUpdated=true;
uint16_t mosfetTempRaw, oldMosfetTempRaw;
float mosfetTemp; 
long timeToUpdateDisplay=millis()+DISPLAY_UPDATE_WINDOW;
uint8_t notificationPriority; // Contiene la prioridad de notificaciones
uint16_t time_mseg; // Tiempo en mili segundos
bool forceRePrint; // Para volver a imprimir toda la pantalla
bool isItOverheating=false;
bool isPrintTime=true;
bool isTheSetpointUpdated; // Para tener prioridad al mostrar nuevo setpoint
bool printTinySetpoint=true;
unsigned long timeToPrintNewSetpoint, windowNewSetpoint=2000;
double Output2; // Contiene el duty del segundo MOSFET. Varia con el setpoint
uint8_t oldControlMode; // Contiene el estado anterior del modo de control
long oldEncoderValue; // Para restaurar el valor del encoder al salir del menú
unsigned long timeLoadTestStart;
float vInNominal, vMaxForImax, iMax;
uint8_t paramNumber=1, oldParamNumber=0; // Parámetro seleccionado actual y anterior
long eventStartTime, eventEndTime; // Registran el tiempo de inicio y final del evento
long pulseCount; // Contador de ticks en cada evento (giro del encoder)
bool newValue; // True: si hubo un cambio, admitido, en el encoder
bool modifyParameter; // True: se entro en modo edición del parámetro seleccionado
bool actualColor; // Color actual del parámetro
bool firstTime=true; // True: para ejecutar acciones solo en el arranque
uint8_t colorCount; // Contador para indicar el cambio del color, modo edición

void loop() {
    /***************************************************************************/
    /*                    MEDICIÓN DE TENSIÓN Y CORRIENTE                      */
    /***************************************************************************/
    vRaw = analogRead(VBATT_SENSE_PIN);
    // Filtro de Wiener, Adaptativo
    vBattRaw = vBattRawOld + MU * (vRaw - vBattRawOld);  
    // Calculo de vIn
    vIn = 0.01935*vBattRaw + 0.25; // Función obtenida por regresión lineal

    if(vBattRaw != vBattRawOld){// si cambio V entonces actualizo valor en el LCD
        wasVUpdated = true;
        vBattRawOld = vBattRaw;
    }
    
    iBattRaw = (double)analogRead(IBATT_SENSE_PIN);
    // Filtro de Wiener, Adaptativo
    iBattRaw = iBattRawOld + MU * (iBattRaw - iBattRawOld);
    // Calculo iIn
    iIn = (iBattRaw - iAdcOffset) / Adc1aDiff;

    if(iBattRaw != iBattRawOld){ // si cambio I entonces actualizo valor en el LCD
        if(isPowerOn){
            wasIUpdated = true;
        }
        else{
            iAdcOffset = iBattRaw; // Corriente medida, valores de ADC, a 0A

            // Guardo la tensión nominal en vació
            vInNominal = vIn;
        }
        iBattRawOld = iBattRaw;
    }

    // Calculo wIn
    wIn = vIn * iIn;

    // Si vario la Vin entonces recalculo el setpoint para el modo P_CONST_MODE
    if(controlMode==P_CONST_MODE || controlMode==R_CONST_MODE){
        if(vIn != vInOld && isPowerOn){
            // Actualizo valores
            switch (controlMode)
            {
                case P_CONST_MODE:
                    ampereSetpoint =  powerSetpoint/vIn; // Cálculo de la corriente necesaria
                    break;
                case R_CONST_MODE:
                    ampereSetpoint = vIn/resistanceSetpoint; // Cálculo de la corriente necesaria
                break;
                default:
                    break;
            }

            Output2 = ampereToDutycycle(ampereSetpoint*.5, MOSFET2);
            
            Setpoint = ampereToAdc(ampereSetpoint);
            vInOld = vIn;
        }
    }
    // FIN Mediciones

    /***************************************************************************/
    /*                 ¿SE CONECTO LA FUENTE A LA ENTRADA?                     */
    /***************************************************************************/
    // Batería/Fuente conectada y funcionando? Detección persistente
    if(vRaw > VBATT_MIN){
        // Batería conectada
        if(!batteryConnected){
            // Solo se muestra si no se esta el Menu de configuración activo
            if(!showMenu && isPowerOn){
                // Solo mostramos una vez el mensaje
                notificationPriority = 3;
                time_mseg = 500;
                notification_add("BATT CONNECTED", notificationPriority, time_mseg, COLOR_BW);

                batteryConnected = true;
            }
        }
    }
    else{ // NO SE DETECTO TENSIÓN DE ENTRADA!!!
        if(batteryConnected){
            // Solo se muestra si no se esta el Menu de configuración activo
            if(!showMenu && isPowerOn){
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
            }
            if(isPowerOn){
                control_stopOutputsAndReset();
                //myPID.SetMode(MANUAL);  // Apagamos el PID
            }
        }
    }
    // FIN DETECCIÓN DE TENSIÓN DE ENTRADA

    /******************************* Encoder *********************************/
    encoderValueChanged = encoder.encoderChanged();
    encoderDirection = encoder.getDirection();
    
    if(encoderValueChanged){        
        if(showMenu || modifyParameter){
            encoderValue = encoder.readEncoder();
            
            eventEndTime = millis();
            newValue = true;
        }
        else{
            if(!eventStartTime){
                eventStartTime = millis();
                pulseCount = 0;
            }
            
            pulseCount += encoderDirection;
            eventEndTime = millis();
        }
    }
    if(eventStartTime && !modifyParameter && (millis()-eventEndTime)>40){
        // Estando apagado, se produjo un evento y ya termino
        eventStartTime = 0;
        newValue = true; // Evento finalizado, hay nuevo valor
    }
    if(eventStartTime && modifyParameter && (millis()-eventEndTime)>2000){
        // Se termino el tiempo de espera para la edición del parámetro
        eventStartTime = 0;
        modifyParameter = false; // Ya no estoy en modo edición

        forceRePrint = true; // Fuerzo re imprimir la pantalla

        // Emito un Sonido de alerta
        tone(BUZZER_PIN, 7000, 20);
        delay(75);
        tone(BUZZER_PIN, 7000, 80);

        // Recupero configuración previa del contador del encoder
        if(firstTime){
            encoder_setBasicParameters(0, 299, true, paramNumber, 2);
        }
        else{
            encoder_setBasicParameters(0, 199, true, 1, 2);
        }
    }
    //

    /********************* Selección de Parámetro a modificar ****************/
    if(newValue && !showMenu && !modifyParameter){
        newValue = false;
        // Modificación del parámetro seleccionado
        if(abs(pulseCount)>2){ // Entrando a modo EDICIÓN
            if(encoderDirection<0){
                // Emito un Sonido de alerta
                tone(BUZZER_PIN, 2500, 50);
                tone(BUZZER_PIN, 0, 50);
                tone(BUZZER_PIN, 2500, 50);

                modifyParameter = true; // Modo edición activado

                // Re configurar los parámetros del encoder de acuerdo al modo actual de funcionamiento
                switch (paramNumber)
                {
                    case 0: // Modificación de Vlimit
                        encoder_setBasicParameters(0, 2000, false, vLimit*100, 150);
                        break;
                    
                    case 1: // Modificación de C, W o R
                        switch (controlMode){
                            case C_CONST_MODE:
                                encoder_setBasicParameters(0, 1000, false, ampereSetpoint*100, 100);
                                break;
                            case P_CONST_MODE:
                                encoder_setBasicParameters(0, 1000, false, powerSetpoint*10, 100);
                                break;
                            case R_CONST_MODE:
                                encoder_setBasicParameters(0, 1000, false, resistanceSetpoint*10, 100);
                                break;
                            default:
                                break;
                        }
                        break;
                    case 2: // Modo de operación
                        encoder_setBasicParameters(1, 3, true, 0, 0);
                        wasCtrlModeUpdated = true;
                        break;
                    case 3:
                        break;
                    case 4:
                        break;
                    default:
                        break;
                }
                // Como contiene un valor viejo entonces los actualizo
                encoderValue = encoder.readEncoder(); 

                // limpio 
                lcd_printSelectedParam(paramNumber, controlMode); // limpio

                eventStartTime = millis();
                eventEndTime = millis();
            }
            // 
            if(modifyParameter){
                if(isPowerOn){
                    // Si esta encendido el proceso entonces fuerzo a mostrar 
                    // la ventana del setpoint grande la primera vez
                    newValue = true;
                }
            }
        }
        else{ // Entrando en modo SELECCIÓN
            oldParamNumber = paramNumber;
            if(firstTime){
                paramNumber = encoder.readEncoder()%3;
            }
            else{
                paramNumber = encoder.readEncoder()%2;
            }

            lcd_printSelectedParam(oldParamNumber, controlMode); // limpio selección anterior
            lcd_printSelectedParam(paramNumber, controlMode, COLOR_WB); // resalto el actual

            tone(BUZZER_PIN, 600, 10);
        }
    }
    //

    /******************************** Menú ***********************************/
    if(showMenu && encoderValueChanged){
        // Resalto el nuevo Item seleccionado mediante el encoder
        menu.highlightMenuItem(encoderValue);
    }
    //

    /*************************************************************************/
    /*                      MODIFICACIÓN DE PARÁMETROS                       */
    /*************************************************************************/
    if(!isLoadTestRunning){ // No se selecciono la Prueba de Carga
        if(newValue){
            newValue = false;
            if (!showMenu){
                switch (paramNumber)
                {
                    case 0: // Limite de Tensión
                        vLimit = encoderValue/100.0;

                        if(isPowerOn){
                            lcd_printNewSetpoint(vLimit, V_LIMIT);
                            isTheSetpointUpdated = true;
                        }
                        else{
                            printTinySetpoint = true;
                        }
                        break;
                    case 1: // Limite principal 
                        // Calculo el Setpoint necesario en valores de Ampere normalizados
                        ampereSetpoint = modes_handleEncoderChange(vIn, encoderValue, controlMode);
                        
                        // Se actualizo el Setpoint, por lo que se deberá actualizar la pantalla
                        if(isPowerOn){ // Si esta encendido se mostrara en una ventana temporal
                            switch (controlMode){
                                case C_CONST_MODE:
                                    lcd_printNewSetpoint(ampereSetpoint, controlMode);
                                    break;
                                case P_CONST_MODE:
                                    lcd_printNewSetpoint(powerSetpoint, controlMode);
                                    break;
                                case R_CONST_MODE:
                                    lcd_printNewSetpoint(resistanceSetpoint, controlMode);
                                    break;
                                default:
                                    break;
                            }
                            
                            isTheSetpointUpdated = true;
                        }
                        else{ // Si esta APAGADO se mostrara en lugar de la corriente medida
                            printTinySetpoint = true;
                        }

                        Output2 = ampereToDutycycle(ampereSetpoint*.5, MOSFET2);
                        if(isPowerOn){
                            // Calculo el Setpoint necesario en valores que interpreta el PID
                            Setpoint = ampereToAdc(ampereSetpoint);
                        }
                        break;
                    case 2: // Modo de Operación
                        controlMode = encoderValue;
                        wasCtrlModeUpdated = true;
                        break;
                    case 3:
                        break;
                    case 4:
                        break;
                    default:
                        break;
                }
                
                //tone(BUZZER_PIN, 600, 10);

                timeToPrintNewSetpoint = millis() + windowNewSetpoint;
            }
        }

        // Tiempo de muestra de la Ventana temporal que imprime el nuevo SETPOINT
        if(isTheSetpointUpdated && (millis()>timeToPrintNewSetpoint)){
                isTheSetpointUpdated = false;
                // si estando encendido se desconecta y luego modifico el setpoint, no se reimprime la notificación
                control_forceReprintDisplay();

                tone(BUZZER_PIN, 7000, 20);
                delay(75);
                tone(BUZZER_PIN, 7000, 80);
        }
        // FIN CONFIGURACIÓN DE CORRIENTE DE CARGA
    }
    /***************************************************************************/
    /*                              PRUEBA DE CARGA                            */
    /***************************************************************************/
    else{
        // Cada determinado tiempo incremento ampereSetpoint
        if(isPowerOn && isLoadTestRunning){
            if(millis() > (timeLoadTestStart+250)){
                ampereSetpoint += 0.01;
                Output2 = ampereToDutycycle(ampereSetpoint*.5, MOSFET2);                
                Setpoint = ampereToAdc(ampereSetpoint);
                
                timeLoadTestStart = millis();
            }
        }

        // Verifico que la tensión no haya caído por debajo de 85% de la nominal
        if(vIn < (0.85*vInNominal)){// Cayo por debajo, apago el control
            // Apago las salidas PWM
            control_powerOff();
            // Presento informe de salida
            lcd_printShowLoadTestResults(vMaxForImax, iMax);
            // Espero por un click en el botón
            while(!isButtonClicked());
            // Re imprimo todo como si se encendiese por primera vez
            isPowerOn = false;
            control_forceReprintDisplay();
            // Limpio contadores y reloj
            control_resetAllForNewMode();
            // Deshabilito la prueba
            isLoadTestRunning = false;
        }
        else{ // Todo normal por aquí
            vMaxForImax = vIn;
            iMax = iIn;
        }
        
    }

    /***************************************************************************/
    /*                           ENCENDIDO Y APAGADO                           */
    /***************************************************************************/
    // Verifico los Limites para que el sistema permanezca encendido
    // Auto-apagado
    if(vIn <= vLimit && vRaw > VBATT_MIN && isPowerOn){// espero 250ms en el arranque
        // Apago 
        isPowerOn = false;
        // Apago la salida PWM y envio notificación en pantalla
        control_powerOff();
        digitalWrite(LED, HIGH);
    }

    // Detección de pulsación de botón  
    key = isButtonClicked(); // 0: no click, 1: short click, 2: long click, 3: double click
    if(key){
        if(!showMenu){ // Modo normal
            //  HUBO UNA PULSACIÓN LARGA
            if(key == LONG_CLICK){ // Reiniciamos los contadores

                notificationPriority = 1;
                notification_add("RESET COUNTERS", notificationPriority);

                clock_resetClock(timeDuration);
                
                control_resetCounters();

                tone(BUZZER_PIN, 1000,300);
            }

            // HUBO UN CLICK EN EL BOTÓN
            else if (key == SHORT_CLICK){
                // Una vez que se ejecuto el proceso de control ya no se 
                // mostraran algunas opciones que se ejecutan solo al arranque
                if(firstTime){
                    firstTime = false;
                }
                isPowerOn = not isPowerOn; // Cambia el estado anterior, de ENCENDIDO -> APAGADO y viceversa

                // ENCENDIDO
                if(isPowerOn){
                    forceRePrint = true;

                    notificationPriority = 2;
                    notification_add("   POWER ON   ", notificationPriority);
                    
                    tone(BUZZER_PIN, 4000, 50);
                    delay(20);
                    tone(BUZZER_PIN, 4500, 50);
                    delay(20);
                    tone(BUZZER_PIN, 5000, 50);
                    
                    Setpoint = ampereToAdc(ampereSetpoint);

                    coolerFan_powerOn();   
                    
                    #ifdef DEBUG
                    startTime = millis();
                    nextTime = startTime + WINDOW_CAPTURE;
                    #endif
                }
                // APAGADO
                else{
                    control_powerOff();
                    //myPID.SetMode(MANUAL);  // Apagamos el PID

                    coolerFan_powerOff();
                }

                digitalWrite(LED, isPowerOn?LOW:HIGH);
            }
            else if(key == DOUBLE_CLICK){
                menu_init();
                showMenu = true;
                oldEncoderValue = encoderValue;
            }
        } // FIN Modo normal
        else{ // Se ingresa al Menú
            menu.executeMenuAction(); // Ejecuta la acción asociada al ítem del menú seleccionado.

            if(!showMenu){      // Restauro los valores
                control_forceReprintDisplay();

                if(oldControlMode != controlMode){ // Nuevo modo seteado
                    // Inicializo el setpoint por defecto para cada modo
                    vLimit = V_0V;              // 0 Volt
                    ampereSetpoint = C_1A;      // 1 Ampere
                    powerSetpoint = P_1W;       // 1 Watt
                    resistanceSetpoint = R_10R; // 10 Ohm

                    isPowerOn = false; // Forzar apagado

                    oldControlMode = controlMode;
                }
                
                switch (controlMode){
                    case C_CONST_MODE:
                        encoder_setBasicParameters(0, 1000, false, ampereSetpoint*100, 100);
                        break;
                    case P_CONST_MODE:
                        encoder_setBasicParameters(0, 1000, false, powerSetpoint*10, 100);
                        break;
                    case R_CONST_MODE:
                        encoder_setBasicParameters(0, 1000, false, resistanceSetpoint*10, 100);
                        break;
                    default:
                        break;
                }
                
                // Vuelvo a calcular el Setpoint para el modo seleccionado
                encoderValue = encoder.readEncoder();
                if(!isLoadTestRunning){
                    ampereSetpoint = modes_handleEncoderChange(vIn, encoderValue, controlMode);
                }
                else{
                    ampereSetpoint = 0;
                }
            }
        }
    }
    // FIN BOTÓN

    /***************************************************************************/
    /*           CHEQUEO Y CONTROL DE LA TEMPERATURA EN EL MOSFET              */
    /***************************************************************************/
    mosfetTempRaw = analogRead(FET_TEMP_SENSE_PIN);
    // Calculo la temperatura en el mosfet
    //mosfetTemp = mosfetTempRaw;
    mosfetTemp = 23;
    
    // Si supera cierta Temp máxima enciendo el Cooler
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

        if(!isItOverheating){ // Notificación por exceso de temperatura, solo una vez
            
            if(isPowerOn){
                control_stopOutputsAndReset();
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
    if(mosfetTempRaw!=oldMosfetTempRaw){
        wasTempUpdated = true;
        oldMosfetTempRaw = mosfetTempRaw;
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
            // Wh
            totalWh += (wIn / 360000.0);  // PID_WINDOW_SIZE / 3600000.0 = 1/360000.0
            // Ah
            totalmAh += (iIn / 360.0);    // PID_WINDOW_SIZE / 3600000.0 * 1000 = 1/360.0

            wasXhUpdated = true;
        }
    }
    // FIN PID

    /***************************************************************************/
    /*                      Actualizo Tiempo Transcurrido                      */
    /***************************************************************************/
    currentMillis = millis();  // Obtiene el tiempo actual

    // Comprueba si ha pasado el intervalo de 1 segundo
    if (isPowerOn && batteryConnected && (currentMillis - previousMillis)>=TIME_1SEG && !isLoadTestRunning) {
        previousMillis = currentMillis;  // Guarda el tiempo actual
        
        if(timeDuration == NO_LIMIT){
            // Incremento el tiempo
            clock_update();
        }
        else if(totalTime > 0){
            // Decremento el tiempo
            clock_decrement_time();
        }
        else{ // Si se supero el periodo de tiempo seleccionado, apago
            isPowerOn = false;
            // Apago la salida PWM y envio notificación en pantalla
            control_powerOff();
            // Reseteo el reloj, por si quiero volver a prender la carga
            clock_resetClock(timeDuration);
            // Apago Led indicador de estado
            digitalWrite(LED, HIGH);
        }
        isPrintTime = true;
    }
    // FIN RELOJ

    /***************************************************************************/
    /*                            Refresco la Pantalla                         */
    /***************************************************************************/
    if(!showMenu){
        // Se refresca cada 20mseg
        if(millis()>timeToUpdateDisplay && !isTheSetpointUpdated){
            // Manejo del parpadeo, en el valor del parámetro, en modo edición
            colorCount ++;
            if(colorCount == 20){// Parpadeo cada 20*20mS=400mseg                
                colorCount = 0;
                if(modifyParameter){
                    actualColor? actualColor=COLOR_WB: actualColor=COLOR_BW;
                }
                else{
                    actualColor = COLOR_BW;
                }
            }

            if(newNotification){
                if(notification_hasExpired()){
                    // Fuerzo reimprimir toda la pantalla
                    control_forceReprintDisplay();

                    /* Se coloca aquí porque, al reconectar la batería tras una desconexión,
                    * se genera un sobre-impulso que puede dañar la fuente/batería. Por 
                    * ello, espero a que desaparezca la notificación de 'conexión de 
                    * batería' antes de reactivar la carga."
                    */
                    if(isPowerOn&&batteryConnected){
                        Setpoint = ampereToAdc(ampereSetpoint);
                    }
                }
            }
            else {
                // actualizo el valor de la temperatura en el MOSFET
                if(wasTempUpdated && !firstTime){
                    lcd_printTemperature(mosfetTemp);
                    wasTempUpdated = false;
                }
                // Imprimir tiempo transcurrido
                // Usando las mini notificaciones puedo mostrar la temperatura
                if(!newMiniNotification && isPrintTime){
                    if(isPowerOn){
                        lcd_printTime(clock_get_hours(), clock_get_minutes(), clock_get_seconds(), COLOR_WB);
                    }
                    else{
                        lcd_printTime(clock_get_hours(), clock_get_minutes(), clock_get_seconds());
                    }
                    isPrintTime = false;
                }
                // Solo para modo Prueba de carga imprimo AmpereSetpoint en lugar del reloj
                if(isLoadTestRunning){
                    lcd_printAmpereSetpoint(ampereSetpoint);
                }
            }

            if(forceRePrint){   // Reimprimir toda la pantalla
                if(firstTime){
                    lcd_printSelectBaseFrame(controlMode);
                }
                else{
                    lcd_printBaseFrame(controlMode);
                    paramNumber = 1;
                }
                // Resaltar Parámetro a modificar
                lcd_printSelectedParam(paramNumber, controlMode, COLOR_WB); 

                wasXhUpdated = true;
                wasVUpdated = true;
                if(isPowerOn){
                    wasIUpdated = true;
                }
                else{
                    printTinySetpoint = true;
                }
                if(!firstTime){
                    isPrintTime = true;
                    wasTempUpdated = true;
                }
                else{
                    wasCtrlModeUpdated = true;
                }

                forceRePrint = false;
            }

            // Imprimo los Watts-Hora y Ampere-Hora, o Modo de Operación 
            if(firstTime){
                if(wasCtrlModeUpdated){
                    if(paramNumber == 2){
                        lcd_printOpMode(controlMode, actualColor);
                    }
                    else{
                        lcd_printOpMode(controlMode, COLOR_BW);
                    }
                    if(!modifyParameter)
                        wasCtrlModeUpdated = false;
                }
            }
            else{
                if(wasXhUpdated){      
                    lcd_printWattHour(totalWh);
                    lcd_printAmpHour(totalmAh);

                    wasXhUpdated = false;
                }
            }

            // Imprimo Tensión medida
            if(isPowerOn){
                if(wasVUpdated){
                    // Print Vin
                    lcd_printVin(vIn);
                    wasVUpdated = false;
                }
            }
            else {
                //lcd_printVin(vLimit, COLOR_WB);                
                if(modifyParameter){
                    if(paramNumber==0){
                        lcd_printVin(vLimit, actualColor);
                    }
                }
                else{
                    if(wasVUpdated){
                        lcd_printVin(vLimit);
                        wasVUpdated = false;
                    }
                }
            }
            // Imprimo Corriente medida
            if(isPowerOn)
            { // Solo muestro la corriente cuando esta encendido. Caso contrario, el Setpoint
                if(wasIUpdated){
                    switch (controlMode){
                        case C_CONST_MODE:
                            // Print Iin
                            lcd_printIin(iIn);
                            break;
                        case P_CONST_MODE:
                            // Print Win
                            lcd_printPin(wIn);
                            break;
                        case R_CONST_MODE:
                            // Print Rout
                            lcd_printRout(vIn/iIn);
                            break;
                        default:
                            break;
                    }
                    wasIUpdated = false;
                }
            }
            else{
                if(modifyParameter && paramNumber==1){
                    // mientras no este en funcionamiento la carga, se mostrara el setpoint
                    switch (controlMode){
                        case C_CONST_MODE:
                            lcd_printTinyNewSetpoint(ampereSetpoint, controlMode, actualColor);
                            break;
                        case P_CONST_MODE:
                            lcd_printTinyNewSetpoint(powerSetpoint, controlMode, actualColor);
                            break;
                        case R_CONST_MODE:
                            lcd_printTinyNewSetpoint(resistanceSetpoint, controlMode, actualColor);
                            break;
                        default:
                        break;
                    }
                }
                else if(printTinySetpoint){
                    // mientras no este en funcionamiento la carga, se mostrara el setpoint
                    switch (controlMode){
                        case C_CONST_MODE:
                            lcd_printTinyNewSetpoint(ampereSetpoint, controlMode);
                            break;
                        case P_CONST_MODE:
                            lcd_printTinyNewSetpoint(powerSetpoint, controlMode);
                            break;
                        case R_CONST_MODE:
                            lcd_printTinyNewSetpoint(resistanceSetpoint, controlMode);
                            break;
                        default:
                        break;
                    }
                    
                    printTinySetpoint = false;
                }
            }

            lcd_display();
            
            timeToUpdateDisplay = millis()+DISPLAY_UPDATE_WINDOW; // Cada 20ms
        }
    }
    // FIN UPDATE DISPLAY

    #ifdef DEBUG
    /***************************************************************************/
    /*                   Envio Datos por el puerto Serie                       */
    /***************************************************************************/
    if(millis()>nextTime && isPowerOn && millis()<(startTime+WINDOW_10SEG)){
     
        actualTime = (millis()-startTime);
        // Para gráficar y obtener datos utilizando serial_port_plotter
        //sprintf(buff, "$%d %ld %d;", (int)encoderValue, actualTime, (int)iBattRaw);
        Serial.printf("$%d %ld %d;", encoderValue, (unsigned long)actualTime, (int)iBattRaw);
        nextTime = millis() + WINDOW_CAPTURE;
    }
    if(isPowerOn && millis()>(startTime+WINDOW_10SEG)){
        isPowerOn = false;
        notificationPriority = 2;
        notification_remove(3);
        notification_add("   POWER OFF  ", notificationPriority);

        tone(BUZZER_PIN, 436, 100);
        tone(BUZZER_PIN, 200, 150);

        control_stopOutputsAndReset();
    }
    // END Serial port
    #endif
}