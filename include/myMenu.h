#ifndef _MY_MENU_H
#define _MY_MENU_H

#include "menu.h"
#include "calibrate.h"
#include "tclock.h"
#include "control.h"
#include "encoder.h"

// Inicializa la librería de menús
MenuLibraryWithSubmenus menu;

bool showMenu;              //True: si se muestra el menu y no las mediciones
extern uint8_t controlMode; // Modo de control, corriente cte. por defecto (C_CONST_MODE es 1)
extern bool isPowerOn;      // True: proceso funcionando
extern float vLimit;        // Contiene la tensión limite de descarga

void menu_exit() // libera memoria y sale del menu inmediatamente
{
    menu.freeMemory();
    showMenu = false;
}

void menu_goback() // Por ahora no debería hacer nada en especial
{
}

void menu_modeCurrentContant()
{
    controlMode = 1;
    timeDuration = NO_LIMIT;
    control_resetAllForNewMode();
    menu_exit();
}

void menu_modePowerContant()
{
    controlMode = 2;
    timeDuration = NO_LIMIT;
    control_resetAllForNewMode();
    menu_exit();
}

void menu_modeResistanceContant()
{
    controlMode = 3;
    timeDuration = NO_LIMIT;
    control_resetAllForNewMode();
    menu_exit();
}

void menu_modeTime()
{
    long eValue;    // Valor leído desde el encoder
    long oldEValue; // Valor guardado antes de entrar a configurar tiempo
    uint8_t key;
    Tiempo time;    // contendrá el tiempo de funcionamiento seleccionado
    bool selectedTime = false;

    // Detengo la salida pwm si estaba trabajando
    isPowerOn = false;
    control_stopOutputsAndReset();

    // Recupero si había un valor previo guardado
    time = clock_totalTime_to_standar_format(timeDuration);
    
    lcd_printClock(time.horas, time.minutos, time.segundos, controlMode);
    lcd_drawLine(1, BLACK);
    
    eValue = 1;
    encoder_setBasicParameters(1, 4, true, eValue, 1);
    oldEValue = eValue;
    
    while(!selectedTime){
        if(encoder.encoderChanged()){
            eValue = encoder.readEncoder();
            // limpio linea anterior
            lcd_drawLine(oldEValue, WHITE);
            // Dibujo el nuevo
            lcd_drawLine(eValue, BLACK);

            oldEValue = eValue;
        }        
        
        key = isButtonClicked();
        if(key == SHORT_CLICK){
            lcd_drawLine(eValue, WHITE);
            key = 0;
            switch (eValue){
                case 1: // Configuro la hora
                    encoder_setBasicParameters(0, 23, true, time.horas, 1);
                    while(key != SHORT_CLICK){
                        eValue=encoder.readEncoder();
                        lcd_printPartialClock(eValue, 1, COLOR_WB);
                        key = isButtonClicked();
                    }
                    lcd_printPartialClock(eValue, 1, COLOR_BW);
                    time.horas = eValue;
                    break;
                case 2: // Configuro los Minutos
                    encoder_setBasicParameters(0, 59, true, time.minutos, 1);
                    while(key != SHORT_CLICK){
                        eValue=encoder.readEncoder();
                        lcd_printPartialClock(eValue, 2, COLOR_WB);
                        key = isButtonClicked();
                    }
                    lcd_printPartialClock(eValue, 2, COLOR_BW);
                    time.minutos = eValue;
                    break;
                case 3: // Configuro los Segundos
                    encoder_setBasicParameters(0, 59, true, time.segundos, 1);
                    while(key != SHORT_CLICK){
                        eValue=encoder.readEncoder();
                        lcd_printPartialClock(eValue, 3, COLOR_WB);
                        key = isButtonClicked();
                    }
                    lcd_printPartialClock(eValue, 3, COLOR_BW);
                    time.segundos = eValue;
                    break;
                case 4: // Selección del modo de operación
                        encoder_setBasicParameters(1, 3, true, controlMode, 1);
                        while(key != SHORT_CLICK){
                            controlMode = encoder.readEncoder();
                            lcd_printSelectXConst(controlMode, COLOR_WB);
                            key = isButtonClicked();
                        }
                        lcd_printSelectXConst(controlMode, COLOR_BW);
                        break;
                
                default:
                    break;
            }
            encoder_setBasicParameters(1, 4, true, oldEValue, 1);
            eValue = oldEValue;
            lcd_drawLine(oldEValue, BLACK);
        }
        else if(key == LONG_CLICK){
            // Guardo el tiempo seleccionado
            timeDuration = clock_standar_format_to_totalTime(time);
            lcd_printConfiguredTime(time.horas, time.minutos, time.segundos, timeDuration, controlMode);
            control_resetAllForNewMode();
            // Espero hasta que se haga un click en el botón
            while(!isButtonClicked());
            selectedTime = true; // Se selecciono el tiempo, entonces salgo del while
        }
    }

    // Libero recursos y salgo del menu
    menu_exit();
}

void menu_loadTest()
{
    control_performLoadTest();
    menu_exit();
}

void menu_setVmin()
{
    long oldEncValue = encoder.readEncoder();
    long encoderValue;
    uint8_t key;

    encoder_setBasicParameters(0, 2000, false, vLimit*100, 150);

    key = isButtonClicked();
    lcd.clearDisplay();
    lcd_printNewSetpoint(vLimit, 4); // imprimo Vmin limite 
    lcd.display();

    while(key != SHORT_CLICK){ //sale con una pulsación corta
        if(encoder.encoderChanged()){
            encoderValue = encoder.readEncoder();
            
            vLimit = encoderValue/100.0;
            lcd_printNewSetpoint(vLimit, 4); // imprimo nuevo Vmin limite seleccionado
            lcd.display();
        }
        key = isButtonClicked();
    }
    
    encoder_setBasicParameters(0, 1000, false, oldEncValue);
    menu_exit();
}

void menu_setDefeultMode()  // Re configuro a valores por defecto
{
    controlMode = 1;    // Modo C cte. por defecto   
    isLoadTestRunning = false; // Deshabilito el Modo Prueba de carga
    totalTime = 0;  // Reinicio contador del tiempo de funcionamiento
    timeDuration = NO_LIMIT;
    clock_resetClock(timeDuration);
}

void menu_init(){
    // Agrega elementos al menú principal
    menu.addMenuItem("Configurar");
    menu.addMenuItem("Modo");
    menu.addMenuItem("Calibracion");
    menu.addMenuItem("Prueba de Carga",  menu_loadTest);
    menu.addMenuItem("Salir", menu_exit);

    // Submenu Backlight
    MenuItem* subMenu1 = new MenuItem("Configurar");
    subMenu1->subMenuItemCount = 4;
    subMenu1->subMenu = new MenuItem[subMenu1->subMenuItemCount];
    subMenu1->subMenu[0] = MenuItem("Backlight", lcd_toggleLed);
    subMenu1->subMenu[1] = MenuItem("V minimo", menu_setVmin);
    subMenu1->subMenu[2] = MenuItem("Reset Mode Sel", menu_setDefeultMode);
    subMenu1->subMenu[3] = MenuItem("Atras", menu_goback);

    // Submenu "Modo"
    MenuItem* subMenu2 = new MenuItem("Modo");
    subMenu2->subMenuItemCount = 5;
    subMenu2->subMenu = new MenuItem[subMenu2->subMenuItemCount];
    subMenu2->subMenu[0] = MenuItem("Corriente CTE", menu_modeCurrentContant);
    subMenu2->subMenu[1] = MenuItem("Potencia  CTE", menu_modePowerContant);
    subMenu2->subMenu[2] = MenuItem("Resistor  CTE", menu_modeResistanceContant);
    subMenu2->subMenu[3] = MenuItem("Tiempo",  menu_modeTime);
    //subMenu2->subMenu[3] = MenuItem("Reset Mode Set",  menu_setDefeultMode);
    subMenu2->subMenu[4] = MenuItem("Atras", menu_goback);

    // Submenu Calibración
    MenuItem* subMenu3 = new MenuItem("Calibracion");
    subMenu3->subMenuItemCount = 4;
    subMenu3->subMenu = new MenuItem[subMenu3->subMenuItemCount];
    subMenu3->subMenu[0] = MenuItem("Calibrar", calibration_calibrate);
    subMenu3->subMenu[1] = MenuItem("Parametros", calibration_show);
    subMenu3->subMenu[2] = MenuItem("Borrar cfg", calibration_resetParameters);
    subMenu3->subMenu[3] = MenuItem("Atras", menu_goback);

    // Asocia los submenús a las opciones del menú principal
    menu.addSubMenu("Configurar", subMenu1);
    menu.addSubMenu("Modo", subMenu2);
    menu.addSubMenu("Calibracion", subMenu3);

    // Muestra el menú principal
    menu.displayMenu();
}

#endif