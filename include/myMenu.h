#ifndef _MY_MENU_H
#define _MY_MENU_H

#include "menu.h"

// Inicializa la librería de menús
MenuLibraryWithSubmenus menu;

bool showMenu; //True: si se muestra el menu y no los mediciones

void menu_exit()
{
    menu.freeMemory();
    showMenu = false;
}

void menu_init(){
    // Agrega elementos al menú principal
    menu.addMenuItem("Backlight");
    menu.addMenuItem("Modo     ");
    menu.addMenuItem("Salir    ", menu_exit);

    // Submenu Backlight
    MenuItem* subMenu1 = new MenuItem("Backlight");
    subMenu1->subMenuItemCount = 2;
    subMenu1->subMenu = new MenuItem[subMenu1->subMenuItemCount];
    subMenu1->subMenu[0] = MenuItem("ON       ", lcd_backlightOn);
    subMenu1->subMenu[1] = MenuItem("OFF      ", lcd_backlightOff);

    // Submenu "Modo"
    MenuItem* subMenu2 = new MenuItem("Modo");
    subMenu2->subMenuItemCount = 3;
    subMenu2->subMenu = new MenuItem[subMenu2->subMenuItemCount];
    subMenu2->subMenu[0] = MenuItem("Corriente CTE");
    subMenu2->subMenu[1] = MenuItem("Potencia  CTE");
    subMenu2->subMenu[2] = MenuItem("Tiempo       ");

    // Asocia los submenús a las opciones del menú principal
    menu.addSubMenu("Backlight", subMenu1);
    menu.addSubMenu("Modo     ", subMenu2);

    // Muestra el menú principal
    menu.displayMenu();
}

#endif