#ifndef _DISPLAY_LCD_H
#define _DISPLAY_LCD_H

#include <Adafruit_PCD8544.h>
#include "config.h"

#define COLOR_WB    0 // Color WHITE sobre BLACK
#define COLOR_BW    1 // Color BLACK sobre WHITE
#define SIZE_S      1 // Tamaño normal de la fuente
#define SIZE_M      2 // Tamaño mediano de la fuente
#define SIZE_L      3 // Tamaño grande de la fuente
#define FONT_H      8 // Altura de la fuente
#define FONT_W      6 // Ancho de la fuente
                                        

Adafruit_PCD8544 lcd = Adafruit_PCD8544(LCD_SCLK_PIN, LCD_DIN_PIN, LCD_DC_PIN, LCD_CS_PIN, LCD_RST_PIN);
#define DISPLAY_UPDATE_WINDOW 200 // Actualizo cada 200 mili segundos

bool updateDisplay = false;
char _buff[20];
bool backlightStatus;

/*void floatTostr(float numero, uint8_t size_buff, uint8_t decimales)
{
    if((decimales+1)<=size_buff){
        int parte_entera, parte_decimal;

        parte_entera = (int)numero; // Obtenemos parte Entera
        switch (size_buff-decimales-1) // digitos de la parte entera
        {
        case 0:
            sprintf(_buff,".");
            break;
        case 1:
            sprintf(_buff,"%1d.",parte_entera); // Obtenemos parte Entera
            break;
        case 2:
            sprintf(_buff,"%2d.",parte_entera); // Obtenemos parte Entera
            break;
        case 3:
            sprintf(_buff,"%3d.",parte_entera); // Obtenemos parte Entera
            break;
        default:
            break;
        }

        parte_decimal = (int)((numero - parte_entera) * pow(10,decimales));  // Multiplicamos por 10^x para obtener los decimales deseados
        switch (decimales)
        {
        case 1:
            sprintf(_buff, "%s%01d", _buff, parte_decimal);
            break;
        case 2:
            sprintf(_buff, "%s%02d", _buff, parte_decimal);
            break;
        case 3:
            sprintf(_buff, "%s%03d", _buff, parte_decimal);
            break;
        case 4:
            sprintf(_buff, "%s%04d", _buff, parte_decimal);
            break;
        case 5:
            sprintf(_buff, "%s%05d", _buff, parte_decimal);
            break;
        default:
            break;
        }
    }
    else
        sprintf(_buff,"NULL");
}*/

void lcd_init()
{
    pinMode(LCD_BKLIGHT_PIN, OUTPUT);
    // Por defecto arranca con el baclkight encendido
    backlightStatus = HIGH;
    digitalWrite(LCD_BKLIGHT_PIN, backlightStatus);
    lcd.begin();
    lcd.cp437(true);
    lcd.setContrast(75);
    lcd.setRotation(2);
    lcd.clearDisplay();
    lcd.setTextColor(BLACK, WHITE);
    lcd.setTextSize(SIZE_S);
    lcd.setCursor(8, LCDHEIGHT/2-4);
    lcd.print("INICIANDO...");
    lcd.display();
    delay(2000);
    lcd.clearDisplay();
}

void lcd_backlightOn()
{
    digitalWrite(LCD_BKLIGHT_PIN, HIGH);
}

void lcd_backlightOff()
{
    digitalWrite(LCD_BKLIGHT_PIN, LOW);
}

void lcd_toggleLed() // Cambia de estado el led cada vez que se lo invoca
{
    digitalWrite(LCD_BKLIGHT_PIN, digitalRead(LCD_BKLIGHT_PIN)?LOW:HIGH);
    backlightStatus = digitalRead(LCD_BKLIGHT_PIN);
}

void lcd_printCalibration()
{
    lcd.clearDisplay();
    lcd.setTextSize(SIZE_S);
    lcd.setTextColor(WHITE, BLACK);
    lcd.print("CALIBRACION");
    lcd.setTextColor(BLACK, WHITE);
    lcd.setCursor(0*SIZE_S*FONT_W, 2*SIZE_S*FONT_H);
    lcd.print("Gire hasta obtener una I=1A Presione para terminar");
    lcd.display();
}

void lcd_printIraw(float iRaw, uint8_t color=COLOR_BW)
{
    //floatTostr(iRaw, 5, 2);
    dtostrf(iRaw, 5, 2, _buff);

    lcd.setTextSize(SIZE_S);
    lcd.setCursor(9*SIZE_S*FONT_W, 5*SIZE_S*FONT_H);
    if(color == COLOR_WB){
        lcd.setTextColor(WHITE, BLACK);
    }
    lcd.print(_buff);
    lcd.setTextColor(BLACK, WHITE);

    lcd.display();
}

void lcd_printCalibrationParameters(double i0Araw, double i1Araw)
{
    lcd.clearDisplay();
    lcd.drawLine(0*SIZE_S*FONT_W, 0*SIZE_S*FONT_H, 14*SIZE_S*FONT_W, 0*SIZE_S*FONT_H, BLACK);
    lcd.setCursor(0*SIZE_S*FONT_W, 0*SIZE_S*FONT_H+2);
    lcd.print("Parametros:");
    lcd.drawLine(0*SIZE_S*FONT_W, 1*SIZE_S*FONT_H+2, 14*SIZE_S*FONT_W, 1*SIZE_S*FONT_H+2, BLACK);

    lcd.setCursor(0*SIZE_S*FONT_W, 2*SIZE_S*FONT_H);
    dtostrf(i0Araw, 6, 2, _buff);
    lcd.printf("adc 0A: %s\n", _buff);
    dtostrf(i1Araw, 6, 2, _buff);
    lcd.printf("adc 1A: %s\n\n", _buff);

    lcd.setTextColor(WHITE, BLACK);
    lcd.print("Click -> Salir");
    lcd.setTextColor(BLACK, WHITE);
    lcd.display();
}

void lcd_printBaseFrame(uint8_t mode=1)
{
    lcd.clearDisplay();
    lcd.setTextSize(SIZE_M);
    lcd.setCursor(6*SIZE_M*FONT_W, 0*SIZE_M*FONT_H);
    lcd.print("V");
    lcd.setCursor(6*SIZE_M*FONT_W, 1*SIZE_M*FONT_H);
    
    // Dependiendo del modo varia la unidad del Setpoint
    switch (mode){
        case 0: // Sin modo de operacion
            break;
        case 1: // Modo Corriente Constante
            lcd.print("A");
            break;
        case 2: // Modo Potencia Constante
            lcd.print("W");
            break;
        case 3: // Modo Resistencia Constante
            lcd.print((char)234); // ASCII del caracter 'Ω'
            break;
        default:
            break;
    }

    lcd.setTextSize(SIZE_S);
    lcd.setCursor(4*SIZE_S*FONT_W, 4*SIZE_S*FONT_H);
    lcd.print("mAh");
    lcd.setCursor(12*SIZE_S*FONT_W, 4*SIZE_S*FONT_H);
    lcd.print("Wh");
    lcd.setCursor(12*SIZE_S*FONT_W, 5*SIZE_S*FONT_H);
    //lcd.print("  :  :      ");
    lcd.print((char)248); // ascii de 'º'
    lcd.print("C");

    updateDisplay = true;
}

void lcd_printNewSetpoint(float value, uint8_t mode=1)
{
    //floatTostr(value, 4, 2);
    switch (mode){
        case 1:
            dtostrf(value, 4, 2, _buff);
            break;
        case 2:
            dtostrf(value, 4, 1, _buff);
            break;
        case 3:
            dtostrf(value, 4, 1, _buff);
            break;
        default:
            break;
    }
    
    lcd.setTextSize(SIZE_L);
    lcd.fillRect(0, ((LCDHEIGHT-SIZE_L*FONT_H)/2)-1, 84, (SIZE_L*FONT_H)+1, WHITE);
    lcd.setCursor(0, (LCDHEIGHT-(SIZE_L*FONT_H))/2);
    lcd.setTextColor(BLACK,WHITE);
    lcd.print(_buff);
    lcd.setCursor(1, (LCDHEIGHT-(SIZE_L*FONT_H))/2+1);
    lcd.setTextColor(BLACK);
    lcd.print(_buff);
    lcd.setTextSize(SIZE_M);
    lcd.setCursor(6*SIZE_M*FONT_W, 1*SIZE_M*FONT_H);
    // Dependiendo del modo varia la unidad del Setpoint
    switch (mode){
        case 0: // Sin modo de operacion
            break;
        case 1: // Modo Corriente Constante
            lcd.print("A");
            break;
        case 2: // Modo Potencia Constante
            lcd.print("W");
            break;
        case 3: // Modo Resistencia Constante
            lcd.print((char)234); // ASCII del caracter 'Ω'
            break;
        default:
            break;
    }
    
    lcd.drawRect(0, ((LCDHEIGHT-SIZE_L*FONT_H)/2)-2, 84, (SIZE_L*FONT_H)+2, BLACK);
    lcd.setTextColor(BLACK,WHITE);
    lcd.display();
}

void lcd_printTinyNewSetpoint(float value, uint8_t mode=1)
{
    //floatTostr(value, 6, 2);
    switch (mode){
        case 1:
            dtostrf(value, 6, 2, _buff);
            break;
        case 2:
            dtostrf(value, 6, 1, _buff);
            break;
        case 3:
            dtostrf(value, 6, 1, _buff);
            break;
        default:
            break;
    }

    lcd.setTextSize(SIZE_M);
    lcd.setCursor(0*SIZE_M*FONT_W, 1*SIZE_M*FONT_H);
    lcd.setTextColor(WHITE, BLACK);
    lcd.print(_buff);
    lcd.setTextColor(BLACK, WHITE);

    updateDisplay = true;
}

void lcd_printPowerOnMessage()
{
    lcd.setTextSize(SIZE_S);
    lcd.setCursor(0*SIZE_S*FONT_W, 5*SIZE_S*FONT_H);
    lcd.setTextColor(WHITE, BLACK);
    lcd.print("   POWER ON   ");
    lcd.setTextColor(BLACK, WHITE);

    updateDisplay = true;
}

void lcd_printPowerOffMessage()
{
    lcd.setTextSize(SIZE_S);
    lcd.setCursor(0*SIZE_S*FONT_W, 5*SIZE_S*FONT_H);
    lcd.print("   POWER OFF  ");

    updateDisplay = true;
}

void lcd_printNoBattery()
{
    lcd.setTextSize(SIZE_S);
    lcd.setCursor(0*SIZE_S*FONT_W, 5*SIZE_S*FONT_H);
    lcd.setTextColor(WHITE, BLACK);
    lcd.print("  NO BATTERY  ");
    lcd.setTextColor(BLACK, WHITE);

    updateDisplay = true;
}

void lcd_printBatteryConnected()
{
    lcd.setTextSize(SIZE_S);
    lcd.setCursor(0*SIZE_S*FONT_W, 5*SIZE_S*FONT_H);
    lcd.print("BATT CONNECTED");

    updateDisplay = true;
}

void lcd_printReset()
{
    lcd.setTextSize(SIZE_S);
    lcd.setCursor(0*SIZE_S*FONT_W, 5*SIZE_S*FONT_H);
    lcd.print("RESET COUNTERS");

    updateDisplay = true;
}

void lcd_printNotification(char *text, uint8_t color=COLOR_BW)
{
    lcd.setTextSize(SIZE_S);
    lcd.setCursor(0*SIZE_S*FONT_W, 5*SIZE_S*FONT_H);
    if(color == COLOR_WB){
        lcd.setTextColor(WHITE, BLACK);
    }
    lcd.print(text);
    lcd.setTextColor(BLACK, WHITE);

    lcd.display();
}

void lcd_printOverTemperatureMessage()
{
    lcd.setTextSize(SIZE_S);
    lcd.setCursor(0*SIZE_S*FONT_W, 5*SIZE_S*FONT_H);
    lcd.setTextColor(WHITE, BLACK);
    lcd.print(" TEMP. MAXIMA ");
    lcd.setTextColor(BLACK, WHITE);

    updateDisplay = true;
}

void lcd_printAmpHour(float a_h)
{
    sprintf(_buff, "%4d",(int)a_h);

    lcd.setTextSize(SIZE_S);
    lcd.setCursor(0*SIZE_S*FONT_W, 4*SIZE_S*FONT_H);
    lcd.print(_buff);

    updateDisplay = true;
}

void lcd_printWattHour(float w_h)
{
    lcd.setTextSize(SIZE_S);

    if(w_h<1){
        lcd.setCursor(8*SIZE_S*FONT_W, 4*SIZE_S*FONT_H);
        sprintf(_buff, "%3d",(int)(w_h*1000));
        lcd.print(_buff);
        lcd.print("m");
    }
    else if(w_h<10){
        lcd.setCursor(8*SIZE_S*FONT_W, 4*SIZE_S*FONT_H);
        //floatTostr(w_h, 4, 2);
        dtostrf(w_h, 4, 2, _buff);
        lcd.print(_buff);
    }
    else if(w_h<100){
        lcd.setCursor(8*SIZE_S*FONT_W, 4*SIZE_S*FONT_H);
        //floatTostr(w_h, 4, 1);
        dtostrf(w_h, 4, 1, _buff);
        lcd.print(_buff);
    }
    else{
        lcd.setCursor(7*SIZE_S*FONT_W, 4*SIZE_S*FONT_H);
        //floatTostr(w_h, 4, 1);
        dtostrf(w_h, 4, 1, _buff);
        lcd.print(_buff);
    }

    updateDisplay = true;
}

void lcd_printTime(uint8_t hs, uint8_t min, uint8_t seg, uint8_t color=COLOR_BW)
{
    sprintf(_buff, "%02d:%02d:%02d",hs,min,seg);

    lcd.setTextSize(SIZE_S);
    lcd.setCursor(0*SIZE_S*FONT_W, 5*SIZE_S*FONT_H);
    if(color == COLOR_WB){
        lcd.setTextColor(WHITE, BLACK);
    }
    lcd.print(_buff);
    lcd.setTextColor(BLACK, WHITE);

    updateDisplay = true;
}

void lcd_printTemperature(float mosfet_temp)
{
    sprintf(_buff, "%3d",(int)mosfet_temp);

    lcd.setTextSize(SIZE_S);
    lcd.setCursor(9*SIZE_S*FONT_W, 5*SIZE_S*FONT_H);
    lcd.print(_buff);

    updateDisplay = true;
}

void lcd_printVin(float v_in)
{
    //floatTostr(v_in, 6, 2);
    dtostrf(v_in, 6, 2, _buff);

    lcd.setTextSize(SIZE_M);
    lcd.setCursor(0*SIZE_M*FONT_W, 0*SIZE_M*FONT_H);
    lcd.print(_buff);

    updateDisplay = true;
}

void lcd_printIin(float i_in)
{
    //floatTostr(i_in, 6, 2);
    dtostrf(i_in, 6, 2, _buff);

    lcd.setTextSize(SIZE_M);
    lcd.setCursor(0*SIZE_M*FONT_W, 1*SIZE_M*FONT_H);
    lcd.print(_buff);

    updateDisplay = true;
}

void lcd_printPin(float p_in)
{
    //floatTostr(p_in, 6, 1);
    dtostrf(p_in, 6, 1, _buff);

    lcd.setTextSize(SIZE_M);
    lcd.setCursor(0*SIZE_M*FONT_W, 1*SIZE_M*FONT_H);
    lcd.print(_buff);

    updateDisplay = true;
}

void lcd_printRout(float rOut)
{
    //floatTostr(p_in, 6, 1);
    dtostrf(rOut, 6, 1, _buff);

    lcd.setTextSize(SIZE_M);
    lcd.setCursor(0*SIZE_M*FONT_W, 1*SIZE_M*FONT_H);
    lcd.print(_buff);

    updateDisplay = true;
}

void lcd_printERROR(uint8_t x, uint8_t y, uint8_t sz=1)
{
    lcd.setCursor(x*sz*FONT_W, y*sz*FONT_H);
    lcd.setTextSize(sz);
    lcd.print("ERROR");
    
    lcd.display();
}

void lcd_printSelected(uint8_t posy, uint8_t color=COLOR_BW)
{
    lcd.setCursor(0, posy*FONT_H);
    if(color==COLOR_WB){
        lcd.print((char)175);
    }else{
        lcd.print(" ");
    }
    lcd.display();
}

void lcd_printMenuItem(const char *str, uint8_t posy, uint8_t color=COLOR_BW)
{
    lcd.setTextSize(SIZE_S);
    lcd.setCursor(0*SIZE_S*FONT_W, posy*FONT_H);

    if(posy==0){ // Titulo
        uint8_t str_len=strlen(str);
        uint8_t i, diff;
        char aux[15];
        diff = 14 - str_len;
        // Centro el texto del Titulo del Menu/Submenu
        for(i=0; i<14; i++){
            if((i<diff/2) || i>=(str_len+diff/2)){
                aux[i] = (char)240; // Relleno con caracteres '≡' hacia los lados
            }
            else { // En el centro el Titulo
                aux[i] = str[i-diff/2];
            }
        }
        aux[i] = '\0';
        lcd.print(aux);
    }
    else{ // Items
        lcd_printSelected(posy, color);
        lcd.print(str);
    }

    lcd.display();
}

void lcd_printMenu(char strs[][14], uint8_t strSz, uint8_t actualPos=1)
{
    lcd.clearDisplay();
    for(uint8_t i=0; i<strSz; i++){
        if(i==(actualPos+1)){
            lcd_printMenuItem(strs[i], i, COLOR_WB);
        }else{
            lcd_printMenuItem(strs[i], i);
        }
    }
}

void lcd_printSelectedMenu(const char *str){
    lcd.clearDisplay();
    lcd.setTextSize(SIZE_S);
    lcd.setCursor(0, 0);
    lcd.println("Seleccionado: ");
    lcd.print(str);

    lcd.display();
}

void lcd_drawLine(uint8_t posX, uint8_t color=BLACK)
{
    switch (posX){
        case 1:
            lcd.drawLine(0, 2*SIZE_M*FONT_H+1, 2*SIZE_M*FONT_W, 2*SIZE_M*FONT_H+1, color);
            break;
        case 2:
            lcd.drawLine((posX-1)*(2*SIZE_M*FONT_W + SIZE_S*FONT_W), 2*SIZE_M*FONT_H+1, (posX-1)*(4*SIZE_M*FONT_W + SIZE_S*FONT_W), 2*SIZE_M*FONT_H+1, color);
            break;
        case 3:
            lcd.drawLine((posX-1)*(2*SIZE_M*FONT_W + SIZE_S*FONT_W), 2*SIZE_M*FONT_H+1, (posX-1)*(3*SIZE_M*FONT_W + SIZE_S*FONT_W), 2*SIZE_M*FONT_H+1, color);
            break;
        case 4:
            lcd.drawLine(4*SIZE_S*FONT_W, 6*SIZE_S*FONT_H-1, 11*SIZE_S*FONT_W, 6*SIZE_S*FONT_H-1, color);
            break;
        
        default:
            break;
    }

    lcd.display();
}

void lcd_printSelectXConst(uint8_t mode)
{
    lcd.setCursor(4*SIZE_S*FONT_W, 5*SIZE_S*FONT_H-2);
    switch (mode){
        case 1:
            lcd.print("C Const");
            break;
        case 2:
            lcd.print("P Const");
            break;
        case 3:
            lcd.print("R Const");
            break;
        default:
            break;
    }
    lcd.display();
}

void lcd_printConfiguredTime(uint8_t hs, uint8_t min, uint8_t seg, unsigned long tTime=0, uint8_t mode=1)
{
    char aux[14];
    lcd.clearDisplay();
    lcd.drawLine(0, 0, 14*SIZE_S*FONT_W, 0, BLACK);
    lcd.setCursor(0,2);
    lcd.setTextSize(SIZE_S);
    lcd.print("TIME SELECT:");
    lcd.drawLine(0, 1*SIZE_S*FONT_H+2, 14*SIZE_S*FONT_W, 1*SIZE_S*FONT_H+2, BLACK);

    lcd.setTextSize(SIZE_S);
    lcd.setCursor(0, 2*SIZE_S*FONT_H);
    sprintf(_buff, "%02d:%02d:%02d",hs,min,seg);
    lcd.println(_buff);
    lcd.printf("Total: %dS\n", tTime);
    switch (mode){
        case 1:
            sprintf(aux,"C Const");
            break;
        case 2:
            sprintf(aux,"P Const");
            break;
        case 3:
            sprintf(aux,"R Const");
            break;
        default:
            break;
    }
    lcd.printf("Modo: %s\n", aux);
    lcd.setTextColor(WHITE, BLACK);
    lcd.print("Click -> Salir");
    lcd.setTextColor(BLACK, WHITE);

    lcd.display();
}

void lcd_printPartialClock(uint8_t value, uint8_t posx, uint8_t color=COLOR_BW)
{
    lcd.setTextSize(SIZE_M);
    lcd.setCursor((posx-1)*(2*SIZE_M*FONT_W+SIZE_S*FONT_W), 1*SIZE_M*FONT_H);
    if(color == COLOR_WB){
        lcd.setTextColor(WHITE, BLACK);
        lcd.printf("%02d",value);
        lcd.setTextColor(BLACK, WHITE);
    }
    else{
        lcd.printf("%02d",value);
    }
    
    lcd.setTextSize(SIZE_S);

    lcd.display();
}

void lcd_printClock(uint8_t hs, uint8_t min, uint8_t seg, uint8_t mode)
{
    lcd.clearDisplay();
    lcd.drawLine(0, 0, 14*SIZE_S*FONT_W, 0, BLACK);
    lcd.setCursor(0,2);
    lcd.setTextSize(SIZE_S);
    lcd.print("TIME SELECT:");
    lcd.drawLine(0, 1*SIZE_S*FONT_H+2, 14*SIZE_S*FONT_W, 1*SIZE_S*FONT_H+2, BLACK);

    for(uint8_t i=1; i<=4; i++){
        switch(i){
            case 1: // imprimo Hora modificada
                    lcd_printPartialClock(hs, i);
                    lcd.setCursor(2*SIZE_M*FONT_W, SIZE_S*FONT_H*2+FONT_H/2);
                    lcd.printf(":");
                    break;
            case 2: // imprimo Minutos modificados
                    lcd_printPartialClock(min, i);
                    lcd.setCursor(4*SIZE_M*FONT_W+SIZE_S*FONT_W, SIZE_S*FONT_H*2+FONT_H/2);
                    lcd.printf(":");
                    break;
            case 3: // imprimo Segundos modificados
                    lcd_printPartialClock(seg, i);
                    break;
            case 4:
                    lcd_printSelectXConst(mode);
                    break;
            default:
                    break;
        }
    }
    lcd.display();
}

void lcd_printAmpereSetpoint(float ampsetpoint)
{
    dtostrf(ampsetpoint, 4, 2, _buff);
    lcd.setTextSize(SIZE_S);
    lcd.setCursor(0*SIZE_S*FONT_W, 5*SIZE_S*FONT_H);
    lcd.printf("Amp:%s", _buff);

    updateDisplay = true;
}

void lcd_printShowLoadTestResults(float vin, float iin)
{
    lcd.clearDisplay();
    lcd_printVin(vin);
    lcd.setTextSize(SIZE_M);
    lcd.setCursor(6*SIZE_M*FONT_W, 0*SIZE_M*FONT_H);
    lcd.print("V");
    lcd_printIin(iin);
    lcd.setCursor(6*SIZE_M*FONT_W, 1*SIZE_M*FONT_H);
    lcd.print("A");
    lcd.setTextSize(SIZE_S);
    lcd.setCursor(0, 5*SIZE_S*FONT_H);
    lcd.print("  RESULTADOS  ");
    
    lcd.display();
}

void lcd_display()
{
    lcd.display();
    updateDisplay = false;
}

#endif