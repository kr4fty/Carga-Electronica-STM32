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
    MenuLibraryWithSubmenus(Adafruit_PCD8544 &disp, AiEsp32RotaryEncoder &enc){
        display = &disp;
        encoder = &enc;
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
        encoder->setEncoderValue(0);
        encoder->setBoundaries(0, menuItemsCount-1, true);

        display->clearDisplay();
        display->setTextSize(1);
        display->setTextColor(BLACK, WHITE);
        display->setCursor(0, 0);
        display->println("Menu:");

        for (int i = 0; i < menuItemsCount; i++) {
            if (i == menuIndex) {
                display->setTextColor(WHITE, BLACK); // Resalta la opción seleccionada
            } else {
                display->setTextColor(BLACK, WHITE);
            }
            display->println(menuItems[i]->name);
        }

        display->display();
    }
    void update(){
        if (encoder->encoderChanged()) {
            oldMenuIndex = menuIndex;
            menuIndex = encoder->readEncoder();
            if (currentSubMenu) {
                //displaySubMenu(currentSubMenu);
                updateSelectedSubenuItem(currentSubMenu);
            } else {
                //displayMenu();
                updateSelectedMenuItem();
            }
        }
    }
    void checkButton(){
        if (isButtonClicked()) {
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
            delay(200); // Debounce
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
    Adafruit_PCD8544 *display;
    AiEsp32RotaryEncoder *encoder;
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
            display->clearDisplay();
            display->setTextSize(1);
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 0);
            display->print("Seleccionado: ");
            display->println(menuItems[index]->name);
            display->display();
            delay(1000); // Muestra la selección durante 1 segundo
        }
    }
    void displaySubMenu(MenuItem* subMenu){
        encoder->setEncoderValue(0);
        encoder->setBoundaries(0, subMenu->subMenuItemCount-1, true);

        display->clearDisplay();
        display->setTextSize(1);
        display->setTextColor(BLACK, WHITE);
        display->setCursor(0, 0);
        display->println(subMenu->name);

        for (int i = 0; i < subMenu->subMenuItemCount; i++) {
            if (i == menuIndex) {
                display->setTextColor(WHITE, BLACK); // Resalta la opción seleccionada
            } else {
                display->setTextColor(BLACK, WHITE);
            }
            display->println(subMenu->subMenu[i].name);
        }

        display->display();
    }
    void updateSelectedSubenuItem(MenuItem* subMenu){
        display->setCursor(0, (oldMenuIndex+1)*8);
        display->setTextColor(BLACK, WHITE);
        display->print(subMenu->subMenu[oldMenuIndex].name);

        display->setCursor(0, (menuIndex+1)*8);
        display->setTextColor(WHITE, BLACK);
        display->print(subMenu->subMenu[menuIndex].name);
        display->display();
    }
    void updateSelectedMenuItem(){
        display->setCursor(0, (oldMenuIndex+1)*8);
        display->setTextColor(BLACK, WHITE);
        display->print(menuItems[oldMenuIndex]->name);

        display->setCursor(0, (menuIndex+1)*8);
        display->setTextColor(WHITE, BLACK);
        display->print(menuItems[menuIndex]->name);
        display->display();
    }
};

#endif
