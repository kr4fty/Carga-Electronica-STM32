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
        startIndex = 0;
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

        // Inicializar el índice de inicio
        startIndex = 0;

        lcd_printMenu(menuList, menuItemsCount+1, menuIndex, startIndex);
    }
    void highlightMenuItem(long encoderValue){ // Resalta el nuevo Item seleccionado mediante el encoder
        oldMenuIndex = menuIndex;
        menuIndex = encoderValue;

        // Calcular el índice de inicio para el desplazamiento
        if (menuIndex >= MAX_DISPLAYED_ITEMS) {
            startIndex = menuIndex - MAX_DISPLAYED_ITEMS + 1;
        } else {
            startIndex = 0;
        }
        if (currentSubMenu) {
            updateSelectedSubMenuItem(currentSubMenu);
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
    int startIndex; // Variable para el índice de inicio del desplazamiento
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
        } else {
            lcd_printSelectedMenu(selectedItem->name);
            while(!isButtonClicked()); // Muestra la selección mientras no haya un click
        }
    }
    void displaySubMenu(MenuItem* subMenu){
        encoder_setBasicParameters(0, subMenu->subMenuItemCount-1, true, 0);

        sprintf(menuList[0], "%s", subMenu->name);

        for (int i = 0; i < subMenu->subMenuItemCount; i++) {
            sprintf(menuList[i+1], "%s",subMenu->subMenu[i].name);
        }

        // Inicializar el índice de inicio
        startIndex = 0;

        lcd_printMenu(menuList, subMenu->subMenuItemCount+1, menuIndex, startIndex);
    }
    void updateSelectedSubMenuItem(MenuItem* subMenu){
        // Si nos movemos dentro de los limites (0, MAX_DISPLAYED_ITEMS), solo refresco el Item que se resalto
        if( (menuIndex<MAX_DISPLAYED_ITEMS) && (oldMenuIndex<MAX_DISPLAYED_ITEMS) && (menuIndex<MAX_DISPLAYED_ITEMS) ){
            lcd_printSelected(oldMenuIndex+1-startIndex);
            lcd_printSelected(menuIndex+1-startIndex, COLOR_WB);
        }
        else{ // Se salio de los limites, entonces reimprimo toda la pantalla
            lcd_printMenu(menuList, subMenu->subMenuItemCount+1, menuIndex, startIndex);
        }
    }
    void updateSelectedMenuItem(){
        // Si nos movemos dentro de los limites (0, MAX_DISPLAYED_ITEMS), solo refresco el Item que se resalto
        if( (menuIndex<MAX_DISPLAYED_ITEMS) && (oldMenuIndex<MAX_DISPLAYED_ITEMS) && (menuIndex<MAX_DISPLAYED_ITEMS) ){
            lcd_printSelected(oldMenuIndex+1-startIndex);
            lcd_printSelected(menuIndex+1-startIndex, COLOR_WB);
        }
        else{ // Se salio de los limites, entonces reimprimo toda la pantalla
            lcd_printMenu(menuList, menuItemsCount+1, menuIndex, startIndex);
        }
    }
};

#endif