#ifndef _MENU_H
#define _MENU_H

#include <Arduino.h>
#include "config.h"
#include "display.h"
#include "encoder.h"

class MenuItem {
public:
    const char* name;
    MenuItem* subMenu; // Puntero a un submenú
    int subMenuItemCount;
    void (*action)(); // Puntero a función para la acción

    MenuItem(const char* name, void (*action)() = nullptr) 
        : name(name), subMenu(nullptr), subMenuItemCount(0), action(action) {}
    MenuItem() : name(nullptr), subMenu(nullptr), subMenuItemCount(0), action(nullptr) {} // Constructor predeterminado
};

class MenuLibraryWithSubmenus {
public:
    char menuList[10][14];

    MenuLibraryWithSubmenus(){
        menuIndex = 0;
        menuItemsCount = 0;
        currentSubMenu = nullptr;
    }
    void addMenuItem(const char* item, void (*action)()=nullptr) {
        if (menuItemsCount < maxMenuItems) {
            menuItems[menuItemsCount++] = new MenuItem(item, action);
        }
    }
    void addSubMenu(const char* item, MenuItem* subMenu) {
        for (int i = 0; i < menuItemsCount; i++) {
            if (strcmp(menuItems[i]->name, item) == 0) {
                menuItems[i]->subMenu = subMenu;
                break;
            }
        }
    }
    void displayMenu(){
        encoder_setBasicParameters(0, menuItemsCount-1, true);

        sprintf(menuList[0],"Menu");

        for (int i = 0; i < menuItemsCount; i++) {
            sprintf(menuList[i+1] ,"%s", menuItems[i]->name);
        }

        lcd_printMenu(menuList, menuItemsCount+1, menuIndex);
    }
    void highlightMenuItem(long encoderValue){ // Resaltar el nuevo Item seleccionado mediante el encoder
        oldMenuIndex = menuIndex;
        menuIndex = encoderValue;
        if (currentSubMenu) {
            updateSelectedSubenuItem(currentSubMenu);
        } else {
            updateSelectedMenuItem();
        }
    }
    void executeMenuAction(){ // Ejecuta la acción asociada al ítem del menú seleccionado.
        if (currentSubMenu) {
            selectMenuItem(menuIndex);
            menuIndex = 0;
            oldMenuIndex = 0;
            displayMenu();
            currentSubMenu = nullptr; // Regresa al menú principal
        } else {
            if (menuItems[menuIndex]->subMenu) {
                currentSubMenu = menuItems[menuIndex]->subMenu; // Entra al submenú
                menuIndex = 0; // Reinicia el índice del submenú
                displaySubMenu(currentSubMenu);
            } else {
                selectMenuItem(menuIndex);
                menuIndex = 0;
                oldMenuIndex = 0;
                displayMenu();
            }
        }
    }
    void freeMemory() {
        // Libera la memoria de los elementos del menú principal
        for (int i = 0; i < menuItemsCount; i++) {
            delete menuItems[i]; // Libera cada elemento del menú
        }
        menuItemsCount = 0; // Resetea el contador de elementos

        // Libera la memoria de los submenús
        for (int i = 0; i < menuItemsCount; i++) {
            if (menuItems[i]->subMenu) {
                delete[] menuItems[i]->subMenu; // Libera el array de submenús
                menuItems[i]->subMenu = nullptr; // Evita punteros colgantes
            }
        }
    }

private:
    int menuIndex;
    int oldMenuIndex;
    int menuItemsCount;
    const int maxMenuItems = 10; // Número máximo de elementos en el menú
    MenuItem* menuItems[10]; // Array para almacenar los elementos del menú
    MenuItem* currentSubMenu; // Puntero al submenú actual

    void selectMenuItem(int index){
        MenuItem* selectedItem;

        // Verifica si estamos en un submenú
        if (currentSubMenu) {
            selectedItem = &currentSubMenu->subMenu[index]; // Selecciona el elemento del submenú
        } else {
            selectedItem = menuItems[index]; // Selecciona el elemento del menú principal
        }

        // Si hay una acción asociada, la ejecutamos
        if (selectedItem->action) {
            selectedItem->action(); // Llama a la función asociada
        }  else{ 
            lcd_pintSelectedMenu(selectedItem->name);
            delay(1000); // Muestra la selección durante 1 segundo
        }
    }
    void displaySubMenu(MenuItem* subMenu){
        encoder_setBasicParameters(0, subMenu->subMenuItemCount-1, true);

        sprintf(menuList[0], "%s", subMenu->name);

        for (int i = 0; i < subMenu->subMenuItemCount; i++) {
            sprintf(menuList[i+1], "%s",subMenu->subMenu[i].name);
        }

        lcd_printMenu(menuList, subMenu->subMenuItemCount+1, menuIndex);
    }
    void updateSelectedSubenuItem(MenuItem* subMenu){
        lcd_printMenuItem(subMenu->subMenu[oldMenuIndex].name, oldMenuIndex+1);

        lcd_printMenuItem(subMenu->subMenu[menuIndex].name, menuIndex+1, COLOR_WB); 
    }
    void updateSelectedMenuItem(){
        lcd_printMenuItem(menuItems[oldMenuIndex]->name, oldMenuIndex+1);

        lcd_printMenuItem(menuItems[menuIndex]->name, menuIndex+1, COLOR_WB);  
    }
};

#endif