#include <Arduino.h>

#include <SPI.h>
#include <U8g2lib.h>

// | D53 | CS |
// | D49 | DC |
// | D51(mosi) | DI |
// | D52(sclk) | CLK |

const uint8_t pinCS = 53;
const uint8_t pinDC = 49;
const uint8_t pinRST = -1;
const uint8_t pinMOSI = 51;
const uint8_t pinSCLK = 52;

U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ pinCS, /* dc=*/ pinDC);


// UB8833 connections

#define DIR_IN  0x00
#define DIR_OUT 0xFF
#define DATA_DIR   DDRC
#define ADDR_H_DIR DDRA
#define ADDR_L_DIR DDRC // same as data, must be latched with AS

const uint8_t pinAS = 2;
const uint8_t pinDS = 3;
const uint8_t pinRST = 45;
const uint8_t pinXTAL = 46;
const uint8_t pinRW = 47;

void setup()
{
    Serial.begin(115200);
    SPI.begin();
    u8g2.begin();
    u8g2.setFont(u8g2_font_5x7_tr);

    DATA_DIR = DIR_IN;
    ADDR_H_DIR = DIR_IN;

    pinMode(pinAS, INPUT);
    pinMode(pinDS, INPUT);
    pinMode(pinRST, OUTPUT);
    pinMode(pinRW, INPUT);
    pinMode(pinXTAL, OUTPUT);
    digitalWrite(pinRST, LOW);
    analogWrite(pinXTAL, 128); // clock with 50%


    digitalWrite(pinRST, HIGH);
}

void loop()
{
    // put your main code here, to run repeatedly:

    u8g2.clearDisplay();
    u8g2.setCursor(7,7);
    u8g2.setDrawColor(1);
    u8g2.drawStr(0,20,"Hello World!");
    u8g2.drawStrX2(0,40,"JU+TE");
    u8g2.drawLine(0,0, 127, 63);
    u8g2.updateDisplay();

    Serial.println("Hello Mega Comp Ju+Te r");
    delay(1000);
}

// put function definitions here:
int myFunction(int x, int y)
{
    return x + y;
}
