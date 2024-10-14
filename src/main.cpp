#include <Arduino.h>

#include <SPI.h>
#include <U8g2lib.h>

#include <mbLog.h>
#include <elapsedMillis.h>

// | D53 | CS |
// | D49 | DC |
// | D51(mosi) | DI |
// | D52(sclk) | CLK |

const uint8_t pinCS = 53;
const uint8_t pinDC = 49;
const uint8_t pinMOSI = 51;
const uint8_t pinSCLK = 52;

U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ pinCS, /* dc=*/ pinDC);


// UB8833 connections

#define DIR_IN  0x00
#define DIR_OUT 0xFF
#define DATA_DIR   DDRC
#define ADDR_H_DIR DDRA
#define ADDR_L_DIR DDRC // same as data, must be latched with AS

#define DATA_OUT (PORTC)
#define DATA_IN  (PINC)
#define ADDR_H   (PINC)
#define ADDR_L   (PINA)

// PORTA - UB8833 High Adr
// PORTC - UB8833 Low Adr / Data

const uint8_t pinAS = 2;
const uint8_t pinDS = 3;
const uint8_t pinRST = 45;
const uint8_t pinXTAL = 46;
const uint8_t pinRW = 47;

elapsedMillis emDisplay = 0;

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
    digitalWrite(pinRST, HIGH);
    // analogWrite(pinXTAL, 128); // clock with 50%

}

void loop()
{
    if(emDisplay > 1000)
    {
        emDisplay = 0;

        LOG <<"---------------------" <<LOG.endl;
        LOG <<"Hello Mega Comp Ju+Te r" <<LOG.endl;

        int as = 0;

        // start 8833 and watch
        digitalWrite(pinRST, HIGH);

        uint8_t dataa = 0;
        uint8_t datac = 0;

        for(int i = 0; i < 5000; i++)
        {
            digitalWrite(pinXTAL, LOW);
            digitalWrite(pinXTAL, HIGH);

            digitalWrite(pinXTAL, LOW);
            digitalWrite(pinXTAL, HIGH);

            digitalWrite(pinXTAL, LOW);
            digitalWrite(pinXTAL, HIGH);

            digitalWrite(pinXTAL, LOW);
            digitalWrite(pinXTAL, HIGH);

            digitalWrite(pinXTAL, LOW);
            digitalWrite(pinXTAL, HIGH);

            digitalWrite(pinXTAL, LOW);
            digitalWrite(pinXTAL, HIGH);

            digitalWrite(pinXTAL, LOW);
            digitalWrite(pinXTAL, HIGH);

            digitalWrite(pinXTAL, LOW);
            digitalWrite(pinXTAL, HIGH);

            dataa |= PINA;
            datac |= PINC;
            // as = digitalRead(pinAS);
            // LOG <<"as" <<as <<" d/low:" <<PINC <<" hi:" <<PINA <<LOG.endl;
        }

        digitalWrite(pinRST, LOW);

        char sbuf[20];
        u8g2.clearDisplay();
        u8g2.setCursor(7,7);
        u8g2.setDrawColor(1);
        u8g2.drawStrX2(0,16,"JU+TE");

        snprintf(sbuf, 20, "A:%x D:%x", dataa, datac);
        u8g2.drawStr(0,30, sbuf);
        // u8g2.drawLine(0,0, 127, 63);
        u8g2.updateDisplay();

    }
}

// put function definitions here:
int myFunction(int x, int y)
{
    return x + y;
}
