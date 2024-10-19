#include <Arduino.h>

#include <SPI.h>
#include <U8g2lib.h>

#include <mbLog.h>
#include <elapsedMillis.h>

#include "rom.h"

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
#define ADDR_L   (PINC)
#define ADDR_H   (PINA)

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

    delay(10000);
    LOG <<"This is the ent." <<LOG.endl;
}

uint32_t clkcount = 0;
int as = 0, oldas = 0;
int ds = 0, oldds = 1;
int rw = 0;
uint16_t addr = 0;

void cpureset()
{
    // Reset should be kept low for 3 full clock cycles ???
    digitalWrite(pinRST, LOW);
    delay(1);

    DATA_DIR = DATA_IN;
    
    // must clock 18 cycles for EMR to reset internals
    for(int i = 0; i < 20; i++)
    {
        digitalWrite(pinXTAL, LOW);
        digitalWrite(pinXTAL, HIGH);
    }
    digitalWrite(pinRST, HIGH);

    oldas = digitalRead(pinAS);
    oldds = digitalRead(pinDS);

    for(int i = 0; i < 8; i++)
    {
        digitalWrite(pinXTAL, LOW);
        digitalWrite(pinXTAL, HIGH);
    }

    clkcount = 0;
}

int cpucycle()
{
    int i = 0;
    for(i = 0; i < 11; i++)
    {
        digitalWrite(pinXTAL, LOW);
        digitalWrite(pinXTAL, HIGH);
        clkcount++;

        as = digitalRead(pinAS);
        if(oldas == 0 && as == 1)
            break;
        oldas = as;
    }
    // as strobe, read rw and low adr
    rw = digitalRead(pinRW);
    addr = (ADDR_H << 8) | ADDR_L;


    oldds = 1;
    for(i = 0; i < 11; i++)
    {
        digitalWrite(pinXTAL, LOW);
        digitalWrite(pinXTAL, HIGH);
        clkcount++;

        ds = digitalRead(pinDS);
        if(oldds == 1 && ds == 0)
            break;
        oldds = ds;
    }
    // LOG <<clkcount <<" ds:" <<ds <<" rw:" <<rw <<" addr:" <<LOG.hex <<addr <<LOG.dec <<LOG.endl;


    // if reading, output [addr]

    oldds = 0;
    for(i = 0; i < 11; i++)
    {
        digitalWrite(pinXTAL, LOW);    delay(1);
        digitalWrite(pinXTAL, HIGH);    delay(1);
        clkcount++;

        ds = digitalRead(pinDS);
        if(oldds == 0 && ds == 1)
            break;
        oldds = ds;
    }
    // LOG <<clkcount <<" ds:" <<ds <<" rw:" <<rw <<" addr:" <<LOG.hex <<addr <<" data:" <<DATA_IN <<LOG.dec <<LOG.endl;

    if(addr | DATA_IN)
    {
        LOG <<clkcount <<" ds:" <<ds <<" rw:" <<rw <<" addr:" <<LOG.hex <<addr <<" data:" <<DATA_IN <<LOG.dec <<LOG.endl;
        LOG <<"---" <<LOG.endl;
    }

    return i;
}

// just run the clock...
uint16_t cpucycle_dummy(uint32_t ascount = 1)
{
    uint16_t result = 0;

    oldas = digitalRead(pinAS);
    oldds = digitalRead(pinDS);
    uint32_t i = 0;
    for(i = 0; i < ascount * 8; i++)
    {
        digitalWrite(pinXTAL, LOW);
        digitalWrite(pinXTAL, HIGH);
        clkcount++;

        // count rising edge
        as = digitalRead(pinAS);
        if(as == 0)
        {
            clkcount = 0;
        }
        if(oldas == 0 && as == 1)
        {
            addr = ((uint16_t)ADDR_H << 8) | ADDR_L;
            rw = digitalRead(pinRW);
            if(addr > 0x07ff)
            {
                LOG <<LOG.endl <<result <<(rw?" R[":" W[") <<LOG.hex <<addr <<LOG.dec <<"] ";
            }

            result++;
            if (result >= ascount) break;
        }
        oldas = as;

        // ds may or may not go low        
        ds = digitalRead(pinDS);
        if(ds == 0)
        {
            if(rw == 1 && addr > 0x7ff)
            {
                // 0x8d, 0x08, 0xff erster befehl is jp 08ff
                DATA_DIR = DIR_OUT;
                uint8_t data = (addr>(0x800+2048)) ? 0xff : _1988_01__2k__address_0800[addr-0x800];
                DATA_OUT = data;
                LOG <<"_[" <<LOG.hex <<data <<"]" <<LOG.dec <<clkcount;
            }
        }
        else //ds == 1
        {
            // if(oldds == 0)
            // {
            //     // keep data out on one more clock
            // }
            // else
            {
                DATA_DIR = DIR_IN;
                LOG <<"~";
            }
        }
        oldds = ds;
    }
    return result;
}

void loop()
{
    if(emDisplay > 100)
    {
        emDisplay = 0;

        LOG <<"---------------------" <<LOG.endl;
        LOG <<"Hello Mega Comp Ju+Te r" <<LOG.endl;


        // start 8833 and watch
        cpureset();

        int asses = cpucycle_dummy(40);
        LOG <<LOG.endl <<"as#:" <<asses <<LOG.endl;

        // for(int i = 0; i < 10000; i++)
        // {
        //     int clks = cpucycle();
        //     if(clks < 20)
        //     {
        //         if(clkcount % 1000 > 980)
        //         {
        //             LOG <<"clkcount:" <<clkcount <<LOG.endl;
        //         }
        //         if(addr > 0)
        //         {
        //             LOG <<"addr after " <<i <<" cpu cygles" <<LOG.endl;
        //             break;
        //         }
        //     }
        //     else
        //     {
        //         LOG <<"cpucycle error" <<LOG.endl;
        //         break;
        //     }
        // }
        // for(int i = 0; i < 10; i++)
        // {
        //     int clks = cpucycle();
        // }

        // char sbuf[20];
        // u8g2.clearDisplay();
        // u8g2.setCursor(7,7);
        // u8g2.setDrawColor(1);
        // u8g2.drawStrX2(0,16,"JU+TE");

        // snprintf(sbuf, 20, "A:%x D:%x", addr, 0);
        // u8g2.drawStr(0,30, sbuf);
        // // u8g2.drawLine(0,0, 127, 63);
        // u8g2.updateDisplay();

    }
}
