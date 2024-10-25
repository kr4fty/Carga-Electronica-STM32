#ifndef _MY_MENU_H
#define _MY_MENU_H

#include "menu.h"
#include "calibrate.h"
#include "tclock.h"
#include "control.h"

// Inicializa la librería de menús
MenuLibraryWithSubmenus menu;

bool showMenu;              //True: si se muestra el menu y no las mediciones
extern uint8_t controlMode; // Modo de control, corriente cte. por defecto (C_CONST_MODE es 1)
extern bool isPowerOn;      // True: proceso funcionando

void menu_exit() // libera memoria y sale del menu inmediatamente
{
    menu.freeMemory();
    showMenu = false;
}

void menu_goback() // Por ahora no deberia hacer nada en especial
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
    long eValue;    // Valor leido desde el encoder
    long oldEValue; // Valor guardado antes de entrar a configurar tiempo
    uint8_t key;
    Tiempo time;    // contendra el tiempo de funcionamiento seleccionado
    bool selectedTime = false;

    // Detengo la salida pwm si estaba trabajando
    isPowerOn = false;
    control_stopOutputsAndReset();

    // Recupero si habia un valor previo guardado
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
                case 4: // I cte. o W cte.
                        switch (controlMode){
                            case 1:
                                controlMode = 2;
                                break;
                            case 2:
                                controlMode = 3;
                                break;
                            case 3:
                                controlMode = 1;
                                break;
                            default:
                                break;
                        }

                        lcd_printSelectXConst(controlMode);
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
            // Espero hasta que se haga un click en el boton
            while(!isButtonClicked());
            selectedTime = true; // Se selecciono el tiempo, entonces salgo del while
        }
    }

    // Libero recursos y salgo del menu
    menu_exit();
}

void menu_setDefeultMode()  // Reconfiguro a valores por defecto
{
    controlMode = 1;
    totalTime = 0;
    timeDuration = NO_LIMIT;
    clock_resetClock(timeDuration);
}

void menu_init(){
    // Agrega elementos al menú principal
    menu.addMenuItem("Backlight");
    menu.addMenuItem("Modo");
    menu.addMenuItem("Calibracion");
    menu.addMenuItem("Reset Mode Sel",  menu_setDefeultMode);
    menu.addMenuItem("Salir", menu_exit);

    // Submenu Backlight
    MenuItem* subMenu1 = new MenuItem("Backlight");
    subMenu1->subMenuItemCount = 3;
    subMenu1->subMenu = new MenuItem[subMenu1->subMenuItemCount];
    subMenu1->subMenu[0] = MenuItem("ON", lcd_backlightOn);
    subMenu1->subMenu[1] = MenuItem("OFF", lcd_backlightOff);
    subMenu1->subMenu[2] = MenuItem("Atras", menu_goback);

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

    // Submenu Calibracion
    MenuItem* subMenu3 = new MenuItem("Calibracion");
    subMenu3->subMenuItemCount = 4;
    subMenu3->subMenu = new MenuItem[subMenu3->subMenuItemCount];
    subMenu3->subMenu[0] = MenuItem("Calibrar", calibration_calibrate);
    subMenu3->subMenu[1] = MenuItem("Parametros", calibration_show);
    subMenu3->subMenu[2] = MenuItem("Borrar cfg", calibration_resetParameters);
    subMenu3->subMenu[3] = MenuItem("Atras", menu_goback);

    // Asocia los submenús a las opciones del menú principal
    menu.addSubMenu("Backlight", subMenu1);
    menu.addSubMenu("Modo", subMenu2);
    menu.addSubMenu("Calibracion", subMenu3);

    // Muestra el menú principal
    menu.displayMenu();
}

#endif