#ifndef _MY_MENU_H
#define _MY_MENU_H

#include "menu.h"
#include "calibrate.h"

// Inicializa la librería de menús
MenuLibraryWithSubmenus menu;

bool showMenu; //True: si se muestra el menu y no los mediciones

void menu_exit() // libera memoria y sale del menu inmediatamente
{
    menu.freeMemory();
    showMenu = false;
}

void menu_goback()
{

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
    subMenu2->subMenu[0] = MenuItem("Corriente CTE");
    subMenu2->subMenu[1] = MenuItem("Potencia  CTE");
    subMenu2->subMenu[2] = MenuItem("Tiempo");
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