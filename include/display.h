

#ifndef _DISPLAY_LCD_H
#define _DISPLAY_LCD_H
#include <Adafruit_PCD8544.h>
#include "config.h"
                                        

Adafruit_PCD8544 lcd = Adafruit_PCD8544(LCD_SCLK_PIN, LCD_DIN_PIN, LCD_DC_PIN, LCD_CS_PIN, LCD_RST_PIN);
#define LED LED_BUILTIN
#define DISPLAY_UPDATE_WINDOW 250 // Actualizo cada 250 mili segundos

bool updateDisplay = false;

void lcd_init()
{
    pinMode(LCD_BKLIGHT_PIN, OUTPUT);
    lcd.begin();
    lcd.cp437(true);
    lcd.setContrast(75);
    lcd.setRotation(2);
    lcd.clearDisplay();
    lcd.setTextColor(BLACK, WHITE);
    lcd.setTextSize(1);
    lcd.setCursor(8, LCDHEIGHT/2-4);
    lcd.print("INICIANDO...");
    lcd.display();
    lcd.clearDisplay();
}

void lcd_printCalibration()
{
    lcd.clearDisplay();
    lcd.setTextSize(1);
    lcd.print("CALIBRACION");
    lcd.setCursor(0*6, 2*8);
    lcd.print("Coloque una carga de 1A y presione para finalizar");
    lcd.display();
}

void lcd_printBaseFrame()
{
    lcd.clearDisplay();
    lcd.setTextSize(2);
    lcd.setCursor(2*6*6,2*0*8);
    lcd.print("V");
    lcd.setCursor(2*6*6,2*1*8);
    lcd.print("A");
    lcd.setTextSize(1);
    lcd.setCursor(1*4*6,1*4*8);
    lcd.print("mAh");
    lcd.setCursor(1*12*6,1*4*8);
    lcd.print("Wh");
    lcd.setCursor(1*12*6,1*5*8);
    //lcd.print("  :  :      ");
    lcd.print((char)248); // ascii de 'º'
    lcd.print("C");

    updateDisplay = true;
}
void lcd_printNewSetpoint(float value)
{
    char buff[6];
    dtostrf(value, 4, 2, buff);
    lcd.setTextSize(3);
    lcd.fillRect(0, ((LCDHEIGHT-3*8)/2)-1, 84, (3*8)+1, WHITE);
    lcd.setCursor(0,(LCDHEIGHT-(3*8))/2);
    lcd.setTextColor(BLACK,WHITE);
    lcd.print(buff);
    lcd.setCursor(1,(LCDHEIGHT-(3*8))/2+1);
    lcd.setTextColor(BLACK);
    lcd.print(buff);
    lcd.setTextSize(2);
    lcd.setCursor(2*6*6,2*1*8);
    lcd.print("A");
    lcd.drawRect(0, ((LCDHEIGHT-3*8)/2)-2, 84, (3*8)+2, BLACK);
    lcd.setTextColor(BLACK,WHITE);
    lcd.display();
}

void lcd_printPowerOnMessage()
{
    lcd.setTextSize(1);
    lcd.setCursor(0, 1*5*8);
    lcd.setTextColor(WHITE, BLACK);
    lcd.print("   POWER ON   ");
    lcd.setTextColor(BLACK, WHITE);

    updateDisplay = true;
}

void lcd_printPowerOffMessage()
{
    lcd.setTextSize(1);
    lcd.setCursor(0, 1*5*8);
    lcd.print("   POWER OFF  ");

    updateDisplay = true;
}
void lcd_printOverTemperatureMessage()
{
    lcd.setTextSize(1);
    lcd.setCursor(0*5, 5*8);
    lcd.setTextColor(WHITE, BLACK);
    lcd.print(" TEMP. MAXIMA ");
    lcd.setTextColor(BLACK, WHITE);

    updateDisplay = true;
}

void lcd_printAmpHour(float a_h)
{
    char buff[5];
    sprintf(buff, "%4d",(int)a_h);

    lcd.setTextSize(1);
    lcd.setCursor(0*6,4*8);
    lcd.print(buff);

    updateDisplay = true;
}

void lcd_printWattHour(float w_h)
{
    char buff[5];
    lcd.setTextSize(1);

    if(w_h<1){
        lcd.setCursor(8*6,4*8);
        sprintf(buff, "%3d",(int)(w_h*1000));
        lcd.print(buff);
        lcd.print("m");
    }
    else if(w_h<10){
        lcd.setCursor(8*6,4*8);
        dtostrf(w_h, 4, 2, buff);
        lcd.print(buff);
    }
    else if(w_h<100){
        lcd.setCursor(8*6,4*8);
        dtostrf(w_h, 4, 1, buff);
        lcd.print(buff);
    }
    else{
        lcd.setCursor(7*6,4*8);
        dtostrf(w_h, 4, 1, buff);
        lcd.print(buff);
    }

    updateDisplay = true;
}

void lcd_printTime(uint8_t hs, uint8_t min, uint8_t seg)
{
    char buff[9];
    sprintf(buff, "%02d:%02d:%02d",hs,min,seg);

    lcd.setTextSize(1);
    lcd.setCursor(0*6,5*8);
    lcd.setTextColor(WHITE, BLACK);
    lcd.print(buff);
    lcd.setTextColor(BLACK, WHITE);

    updateDisplay = true;
}

void lcd_printTemperature(float mosfet_temp)
{
    char buff[4];
    sprintf(buff, "%3d",(int)mosfet_temp);

    lcd.setTextSize(1);
    lcd.setCursor(9*6,5*8);
    lcd.print(buff);

    updateDisplay = true;
}

void lcd_printVin(float v_in)
{
    char buff[6];
    dtostrf(v_in, 6, 2, buff);

    lcd.setTextSize(2);
    lcd.setCursor(2*0*6,2*0*8);
    lcd.print(buff);

    updateDisplay = true;
}

void lcd_printIin(float i_in)
{
    char buff[6];
    dtostrf(i_in, 6, 2, buff);

    lcd.setTextSize(2);
    lcd.setCursor(2*0*6,2*1*8);
    lcd.print(buff);

    updateDisplay = true;
}
void lcd_display()
{
    lcd.display();
    updateDisplay = false;
}


#endif