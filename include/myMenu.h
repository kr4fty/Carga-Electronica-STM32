#ifndef _MY_MENU_H
#define _MY_MENU_H

#include "menu.h"
#include "calibrate.h"
#include "tclock.h"

// Inicializa la librería de menús
MenuLibraryWithSubmenus menu;

bool showMenu; //True: si se muestra el menu y no las mediciones
uint8_t controlMode=1; // Modo de control, corriente cte. por defecto (C_CONST_MODE es 1)

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
    menu_exit();
}

void menu_modePowerContant()
{
    controlMode = 2;
    menu_exit();
}

void menu_modeTime()
{
    encoder_setBasicParameters(1, 3, true,1);
    long evalue;    // Valor leido desde el encoder
    uint8_t key;
    Tiempo time;    // contendra el tiempo de funcionamiento seleccionado
    bool selectedTime = false;

    while(!selectedTime){
        evalue=encoder.readEncoder();

        // Imprimo la hora con los colores invertidos en fragmento seleccionado
        lcd_printClock(time.horas, time.minutos, time.segundos, evalue);
        
        key = isButtonClicked();
        if(key == SHORT_CLICK){
            key = 0;
            switch (evalue)
            {
                case 1: // Configuro la hora
                    encoder_setBasicParameters(0, 23, true,0);
                    while(key != SHORT_CLICK){
                        evalue=encoder.readEncoder();
                        lcd_printClock(evalue, time.minutos, time.segundos, 1);
                        key = isButtonClicked();
                    }
                    time.horas = evalue;
                    break;
                case 2:
                    encoder_setBasicParameters(0, 59, true,0);
                    while(key != SHORT_CLICK){
                        evalue=encoder.readEncoder();
                        lcd_printClock(time.horas, evalue, time.segundos, 2);
                        key = isButtonClicked();
                    }
                    time.minutos = evalue;
                    break;
                case 3:
                    encoder_setBasicParameters(0, 59, true,0);
                    while(key != SHORT_CLICK){
                        evalue=encoder.readEncoder();
                        lcd_printClock(time.horas, time.minutos, evalue, 3);
                        key = isButtonClicked();
                    }
                    time.segundos = evalue;
                    break;
                
                default:
                    break;
            }
            encoder_setBasicParameters(1, 3, true,1);
        }
        else if(key == LONG_CLICK){
            // Guardo el tiempo seleccionado
            timeDuration = clock_standar_format_to_totalTime(time);
            lcd_printClock(time.horas, time.minutos, time.segundos, 0, timeDuration);
            delay(5000);
            selectedTime = true;
        }
    }

    // Libero recursos y salgo del menu
    menu_exit();
}

void menu_init(){
    // Agrega elementos al menú principal
    menu.addMenuItem("Backlight");
    menu.addMenuItem("Modo");
    menu.addMenuItem("Calibracion");
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
    subMenu2->subMenuItemCount = 4;
    subMenu2->subMenu = new MenuItem[subMenu2->subMenuItemCount];
    subMenu2->subMenu[0] = MenuItem("Corriente CTE", menu_modeCurrentContant);
    subMenu2->subMenu[1] = MenuItem("Potencia  CTE", menu_modePowerContant);
    subMenu2->subMenu[2] = MenuItem("Tiempo",  menu_modeTime);
    subMenu2->subMenu[3] = MenuItem("Atras", menu_goback);

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