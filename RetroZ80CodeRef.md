# Ref..

 http://8bitforce.com/projects/retroshield/ [Or here the Z80 code](https://gitlab.com/8bitforce/retroshield-arduino/-/blob/master/kz80/kz80_test/kz80_test.ino?ref_type=heads)

```
////////////////////////////////////////////////////////////////////
// Z80 RetroShield Code
// 2019/02/11
// Version 0.1
// Erturk Kocalar

// The MIT License (MIT)

// Copyright (c) 2019 Erturk Kocalar, 8Bitforce.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

////////////////////////////////////////////////////////////////////
// include the library code:
////////////////////////////////////////////////////////////////////

#include <LiquidCrystal.h>
#include <avr/pgmspace.h>

// Set this to enable outputing ADDR, DATA, Freq
// on LCD and use UP/DOWN for controlling RESET#.
#define USE_LCDKEYPAD 0

// Set this to output memory operations during timer clock.
// warning: this will slow down the timer interrupt, so adjust clock
//          freq accordingly.
#define outputDEBUG 0

////////////////////////////////////////////////////////////////////
// Configuration
////////////////////////////////////////////////////////////////////

/*
  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
*/

#define LCD_RS  8
#define LCD_EN  9
#define LCD_D4  4
#define LCD_D5  5
#define LCD_D6  6
#define LCD_D7  7
#define LCD_BL  10
#define LCD_BTN  0

#define NUM_KEYS   5
#define BTN_DEBOUNCE 10
#define BTN_RIGHT  0
#define BTN_UP     1
#define BTN_DOWN   2
#define BTN_LEFT   3
#define BTN_SELECT 4
const int adc_key_val[NUM_KEYS] = { 30, 180, 360, 535, 760 };
int key = -1;
int oldkey = -1;
boolean BTN_PRESS = 0;
boolean BTN_RELEASE = 0;

////////////////////////////////////////////////////////////////////
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);


////////////////////////////////////////////////////////////////////
// My Variables
////////////////////////////////////////////////////////////////////

int backlightSet = 25;

////////////////////////////////////////////////////////////////////
// Z80 DEFINITIONS
////////////////////////////////////////////////////////////////////

// Z80 HW CONSTRAINTS
// !!! TODO !!!
// 1- RESET_N must be asserted at least 2 clock cycles.
// 2- CLK can not be low more than 5 microseconds.  Can be high indefinitely.
//

////////////////////////////////////////////////////////////////////
// Monitor Code
////////////////////////////////////////////////////////////////////
// static const unsigned char 
PROGMEM const unsigned char rom_bin[] = {
 0x31,0xFF,0x83,0xED,0x56,0xFB,0x3E,0x4D,
 0xD3,0x01,0x3E,0x37,0xD3,0x01,0xC3,0x3C,
 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
 0xF3,0xFB,0xED,0x4D,0x21,0x51,0x00,0x06,
 0x1D,0x7E,0xCD,0x6E,0x00,0x23,0x10,0xF9,
 0xCD,0x7A,0x00,0xCD,0x6E,0x00,0xC3,0x48,
 0x00,0x0A,0x0D,0x54,0x58,0x44,0x3A,0x20,
 0x20,0x20,0x20,0x0A,0x0D,0x52,0x58,0x44,
 0x3A,0x20,0x20,0x20,0x0A,0x0D,0x52,0x65,
 0x61,0x64,0x79,0x3E,0x0A,0x0D,0xF5,0xDB,
 0x01,0xE6,0x01,0xCA,0x6F,0x00,0xF1,0xD3,
 0x00,0xC9,0xDB,0x01,0xE6,0x02,0xCA,0x7A,
 0x00,0xDB,0x00,0xC9
};

////////////////////////////////////////////////////////////////////
// MEMORY LAYOUT
////////////////////////////////////////////////////////////////////

// ROM(s) (Monitor)
#define ROM_START   0x0000
#define ROM_END     (ROM_START+sizeof(rom_bin))

// RAM (4KB)
#define RAM_START   0x8000
#define RAM_END     0x8FFF
byte    RAM[RAM_END-RAM_START+1];


////////////////////////////////////////////////////////////////////
// 8251 Peripheral
// emulate just enough so keyboard/display works thru serial port.
////////////////////////////////////////////////////////////////////
//

#define ADDR_8251_DATA          0x00
#define ADDR_8251_MODCMD        0x01

#define STATE_8251_RESET        0x01
#define STATE_8251_INITIALIZED  0x00
#define CMD_8251_INTERNAL_RESET 0x40
#define CMD_8251_RTS            0x20
#define CMD_8251_DTR            0x02
#define STAT_8251_TxRDY         0x01
#define STAT_8251_RxRDY         0x02
#define STAT_8251_TxE           0x04
#define STAT_DSR                0x80

byte reg8251_STATE;      // register to keep track of 8251 state: reset or initialized
byte reg8251_MODE;
byte reg8251_COMMAND;
byte reg8251_STATUS;
byte reg8251_DATA;

////////////////////////////////////////////////////////////////////
// Z80 Processor Control
////////////////////////////////////////////////////////////////////
//

/* Digital Pin Assignments */
#define DATA_OUT (PORTL)
#define DATA_IN  (PINL)
#define ADDR_H   (PINC)
#define ADDR_L   (PINA)
#define ADDR     ((unsigned int) (ADDR_H << 8 | ADDR_L))

#define uP_RESET_N  38
#define uP_MREQ_N   41
#define uP_IORQ_N   39
#define uP_RD_N     53
#define uP_WR_N     40
#define uP_NMI_N    51
#define uP_INT_N    50
#define uP_CLK      52

// Fast routines to drive clock signals high/low; faster than digitalWrite
// required to meet >100kHz clock
//
#define CLK_HIGH      (PORTB = PORTB | 0x02)
#define CLK_LOW       (PORTB = PORTB & 0xFC)
#define STATE_RD_N    (PINB & 0x01)
#define STATE_WR_N    (PING & 0x02)
#define STATE_MREQ_N  (PING & 0x01)
#define STATE_IORQ_N  (PING & 0x04)

#define DIR_IN  0x00
#define DIR_OUT 0xFF
#define DATA_DIR   DDRL
#define ADDR_H_DIR DDRC
#define ADDR_L_DIR DDRA

unsigned long clock_cycle_count;
unsigned long uP_start_millis;
unsigned long uP_stop_millis;
unsigned int  uP_ADDR;
byte uP_DATA;

void uP_init()
{
  // Set directions
  DATA_DIR = DIR_IN;
  ADDR_H_DIR = DIR_IN;
  ADDR_L_DIR = DIR_IN;
  
  pinMode(uP_RESET_N, OUTPUT);
  pinMode(uP_WR_N, INPUT);
  pinMode(uP_RD_N, INPUT);
  pinMode(uP_MREQ_N, INPUT);
  pinMode(uP_IORQ_N, INPUT);
  pinMode(uP_INT_N, OUTPUT);
  pinMode(uP_NMI_N, OUTPUT);
  pinMode(uP_CLK, OUTPUT);
  
  uP_assert_reset();
  digitalWrite(uP_CLK, LOW);
  
  clock_cycle_count = 0;
  uP_start_millis = millis();
  // Next State
}

void intel8251_init()
{
  reg8251_STATE     = STATE_8251_RESET;
  reg8251_MODE      = 0b01001101;       // async mode: 1x baudrate, 8n1
  reg8251_COMMAND   = 0b00100111;       // enable tx/rx; assert DTR & RTS
  reg8251_STATUS    = 0b10000101;       // TxRDY, TxE, DSR (ready for operation). RxRDY=0
  reg8251_DATA      = 0x00;
}

void uP_assert_reset()
{
  // Drive RESET conditions
  digitalWrite(uP_RESET_N, LOW);
  digitalWrite(uP_INT_N, HIGH);
  digitalWrite(uP_NMI_N, HIGH);
}

void uP_release_reset()
{
  // Drive RESET conditions
  digitalWrite(uP_RESET_N, HIGH);
}


////////////////////////////////////////////////////////////////////
// Processor Control Loop
////////////////////////////////////////////////////////////////////
// This is where the action is.
// it reads processor control signals and acts accordingly.
//
// Z80 takes multiple cycles to accomplish each rd/write.
// so if we act on IORQ, RD, WR at every clock cycle, then
// we perform erroneous extra IO read/writes.
// FIX WR: perform IO write only on IORQ/WR falling edge.
// FIX RD: perform IO read only on IORQ/RD falling edge.
byte prevIORQ = 0;
byte prevMREQ = 0;
byte prevDATA = 0;

ISR(TIMER1_COMPA_vect)
{ 
  CLK_HIGH;    // CLK goes high
  
  uP_ADDR = ADDR;
  
  // unlike memory mapped devices in 6502 & 6809,
  // Z80 bus has two modes: Memory (MREQ_N) and IO (IORQ_N)

  //////////////////////////////////////////////////////////////////////
  // Memory Access?
  if (!STATE_MREQ_N)    
  {

    // Memory Read?
    if (!STATE_RD_N)
    {
      // change DATA port to output to uP:
      DATA_DIR = DIR_OUT;

      // ROM?
      if ( (ROM_START <= uP_ADDR) && (uP_ADDR <= ROM_END) )
        DATA_OUT = pgm_read_byte_near(rom_bin + (uP_ADDR - ROM_START));
      else
      // RAM?
      if ( (uP_ADDR <= RAM_END) && (RAM_START <= uP_ADDR) )
        DATA_OUT = RAM[uP_ADDR - RAM_START];

    } else
    // Write?
    if (!STATE_WR_N)
    {
      // Memory Write
      if ( (RAM_START <= uP_ADDR) && (uP_ADDR <= RAM_END) )
        RAM[uP_ADDR - RAM_START] = DATA_IN;
    }

    if (outputDEBUG)
    {
      char tmp[20];
      sprintf(tmp, "MREQ RW=%0.1X A=%0.4X D=%0.2X\n", STATE_WR_N, uP_ADDR, DATA_OUT);
      Serial.write(tmp);
    }
  } else
  
  //////////////////////////////////////////////////////////////////////
  // IO Access?
  if (!STATE_IORQ_N)
  {    
    // IO Read?
    if (!STATE_RD_N && prevIORQ)    // perform actual read on falling edge
    {
      // change DATA port to output to uP:
      DATA_DIR = DIR_OUT;

      // 8251 access

      if ( ADDR_L == ADDR_8251_DATA)
      {
        // DATA register access
        prevDATA = reg8251_DATA = Serial.read();
        // clear RxRDY bit in 8251
        reg8251_STATUS = reg8251_STATUS & (~STAT_8251_RxRDY);
        // Serial.write("8251 serial read\n");
      }
      else

      if ( ADDR_L == ADDR_8251_MODCMD )
      {
        // Mode/Command Register access
        if (reg8251_STATE == STATE_8251_RESET)
          prevDATA = reg8251_MODE;
        else
          prevDATA = reg8251_STATUS;
      }
      // output data at this cycle too
      DATA_OUT = prevDATA;
    } else
    
    // continuing IO Read?
    if (!STATE_RD_N && !prevIORQ)    // continue output same data
    {
      // change DATA port to output to uP:
      DATA_DIR = DIR_OUT;

      DATA_OUT = prevDATA;
    } else

    // IO Write?
    if (!STATE_WR_N && prevIORQ)      // perform write on falling edge
    {
      // 8251 access
      if (ADDR_L == ADDR_8251_DATA)
      {
        // write to DATA register
        reg8251_DATA = DATA_IN;
        // TODO: Spit byte out to serial
        Serial.write(reg8251_DATA);        
      }
      else
      if ( ADDR_L == ADDR_8251_MODCMD )
        // write to Mode/Command Register
        if (reg8251_STATE == STATE_8251_RESET)
        {
          // 8251 changes from MODE to COMMAND
          reg8251_STATE = STATE_8251_INITIALIZED;
          // we ignore the mode command for now.
          // reg8251_MODE = DATA_IN
          Serial.write("8251 reset\n");
          
        } else {
          // Write to 8251 command register
          reg8251_COMMAND = DATA_IN;
          // TODO: process command sent
        }

    } else
  //////////////////////////////////////////////////////////////////////
    // if (STATE_RD_N && STATE_WR_N)        // 1 && 1
    {
      // Interrupt Mode 2 Acknowledge scenario
      // IORQ asserted, RD & WR not asserted
      // Z80 expects interrupt vector on databus

      // change DATA port to output to uP:
      DATA_DIR = DIR_OUT;

      // default to vector 0
      DATA_OUT = 0;
    }

    if (outputDEBUG)
    {
      char tmp[20];
      sprintf(tmp, "IORQ RW=%0.1X A=%0.4X D=%0.2X\n", STATE_WR_N, uP_ADDR, DATA_OUT);
      Serial.write(tmp);
    }
  }

  prevIORQ = STATE_IORQ_N;
  prevMREQ = STATE_MREQ_N;
  
  //////////////////////////////////////////////////////////////////////
  // start next cycle
  CLK_LOW;    // E goes low

  // one full cycle complete
  clock_cycle_count ++;

  // natural delay for DATA Hold time (t_HR)
  DATA_DIR = DIR_IN;
    
}

////////////////////////////////////////////////////////////////////
// Serial Event
////////////////////////////////////////////////////////////////////

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response. Note: Multiple bytes of data may be available.
 */
void serialEvent0() 
{
  if (Serial.available())
  {
    // if (reg8251_STATUS & CMD_8251_RTS)  // read serial byte only if RTS is asserted
    {
      // stop interrupts while changing 8251 guts.
      cli();                            
      
      // convert to uppercase if needed
      // int ch = toupper( Serial.read() );

      // reg8251_DATA = Serial.read();

      // RxRDY bit for cpu
      reg8251_STATUS = reg8251_STATUS | STAT_8251_RxRDY;
      sei();
    }
  }
  return;
}



////////////////////////////////////////////////////////////////////
// int getKey() - LCD/Keyboard function from vendor
////////////////////////////////////////////////////////////////////

int getKey()
{
  key = get_key2();
  if (key != oldkey)
    {
      delay(BTN_DEBOUNCE);
      key = get_key2();
      if (key != oldkey) {
        oldkey = key;
        if (key == -1)
          BTN_RELEASE = 1;
        else
          BTN_PRESS = 1;
      }
    } else {
      BTN_PRESS = 0;
      BTN_RELEASE = 0;
    }
  return (key != -1);
}

int get_key2()
{
  int k;
  int adc_key_in;

  adc_key_in = analogRead( LCD_BTN );
  for( k = 0; k < NUM_KEYS; k++ )
  {
    if ( adc_key_in < adc_key_val[k] )
    {
      return k;
    }
  }
  if ( k >= NUM_KEYS )
    k = -1;
  return k;
}

////////////////////////////////////////////////////////////////////
// Button Press Callbacks - LCD/Keyboard function from vendor
////////////////////////////////////////////////////////////////////

void btn_Pressed_Select()
{
  // toggle LCD brightness
  analogWrite(LCD_BL, (backlightSet = (25 + backlightSet) % 100) );
}

void btn_Pressed_Left()
{
  // Serial.println("Left.");
  digitalWrite(uP_NMI_N, LOW);
}

void btn_Pressed_Right()
{
  // Serial.println("Right.");
  digitalWrite(uP_NMI_N, HIGH);
}

void btn_Pressed_Up()
{
  // Serial.println("Up.");
  
  // release uP_RESET
  digitalWrite(uP_RESET_N, HIGH);
}

void btn_Pressed_Down()
{
  // Serial.println("Down.");
  
  // assert uP_RESET
  digitalWrite(uP_RESET_N, LOW);
  
  // flush serial port
  while (Serial.available() > 0)
    Serial.read();
}


////////////////////////////////////////////////////////////////////
// Setup
////////////////////////////////////////////////////////////////////

void setup() 
{

  Serial.begin(115200);
  
  if (USE_LCDKEYPAD)
  {
    pinMode(LCD_BL, OUTPUT);
    analogWrite(LCD_BL, backlightSet);  
    lcd.begin(16, 2);
  }
  
  // Initialize processor GPIO's
  uP_init();
  intel8251_init();
  
  // Set up timer1 interrupt to handle clock & addr/data in the fastest possible way.
  // timer interrupt also runs in parallel with the loop() function, so we have a
  // simple multi-threading going on :)
  
  cli();
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 20; // 100kHz mode: 19;  // = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  // CS12 - CS11 - CS10 Prescalar
  // 1=1/1, 2=1/8, 3=1/64, 4=1/256, 5=1/1024
  TCCR1B |= (1 << CS11);   // 95kHz
  // TCCR1B |= (1<<CS10) | (1<<CS12);   // 0.74kHz
  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();

  // delay 250ms so uP sees RESET asserted.
  delay(250);

  Serial.write("init complete\n");
  // Go, go, go
  uP_release_reset();
}


////////////////////////////////////////////////////////////////////
// Loop()
// * This function runs in parallel with timer interrupt handler.
//   i.e. simplest multi-threading.
// * try to be done as quickly as possible so processor does not
//   slow down.
////////////////////////////////////////////////////////////////////

void loop()
{
  if (USE_LCDKEYPAD)
  {
    // Handle key presses
    //
    if ( getKey() ) {
      // button pressed
      if ( BTN_PRESS ) {
        if (key == BTN_SELECT) btn_Pressed_Select();
        if (key == BTN_UP)     btn_Pressed_Up();
        if (key == BTN_DOWN)   btn_Pressed_Down();
        if (key == BTN_LEFT)   btn_Pressed_Left();
        if (key == BTN_RIGHT)  btn_Pressed_Right();      
      }
    } else
    // display processor info & performance
    // if (clock_cycle_count % 10 == 0) 
    {
      char tmp[20];
      float freq;
        
      lcd.setCursor(0, 0);
      // lcd.print(clock_cycle_count);
      sprintf(tmp, "A=%0.4X D=%0.2X", uP_ADDR, DATA_OUT);
      lcd.print(tmp);
      lcd.setCursor(0,1);
      freq = (float) clock_cycle_count / (millis() - uP_start_millis + 1);
      lcd.print(freq);  lcd.print(" kHz   Z80");
    }
  }
  // Service serial port incoming characters.
  //
  serialEvent0();
    
}
```
