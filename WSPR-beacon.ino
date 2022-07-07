/*
 * Modified 07 July 2022 - K5TNA
 * 
 * Minimal WSPR beacon using Si5351Arduino library
 *
 * Based on code:
 * Copyright (C) 2015 - 2016 Jason Milldrum 
 * 
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see .
 */
 // https://gist.github.com/NT7S/2b5555aa28622c1b3fcbc4d7c74ad926

#include "si5351.h"
#include "Wire.h"
#include <JTEncode.h>
#include <int.h>
#include <TinyGPS.h>
#include "Wire.h"

#define TONE_SPACING            146           // ~1.46 Hz
#define WSPR_CTC                10672         // CTC value for WSPR
#define SYMBOL_COUNT            WSPR_SYMBOL_COUNT
#define CORRECTION              0             // Change this for your ref osc

#define TIME_HEADER             "T"           // Header tag for serial time sync message
#define TIME_REQUEST            7             // ASCII bell character requests a time sync message 
#define GREEN                   5
#define RED                     4
#define YELLOW                  3
#define TX_LED_PIN              7
//#define SYNC_LED_PIN            13
//TinyGPS gps;
Si5351 si5351;

JTEncode jtencode;
unsigned long freq = 7040500UL;                // Change this
char call[7] = " K5TNA";                        // Change this
char loc[5] = "EM20";                           // Change this
uint8_t dbm = 10;
uint8_t tx_buffer[SYMBOL_COUNT];

// Global variables used in ISRs
volatile bool proceed = false;
////////////////////////////////////////////////////////////////////////////
// Timer interrupt vector.  This toggles the variable we use to gate
// each column of output to ensure accurate timing.  Called whenever
// Timer1 hits the count set below in setup().
ISR(TIMER1_COMPA_vect)
{
    proceed = true;
     //Serial.println("timer fired");
}
/////////////////////////////////////////////////////

// Loop through the string, transmitting one character at a time.
void encode()
{
    uint8_t i;
  Serial.println("encode()");
    jtencode.wspr_encode(call, loc, dbm, tx_buffer);

    // Reset the tone to 0 and turn on the output
    si5351.set_clock_pwr(SI5351_CLK0, 1);
    digitalWrite(TX_LED_PIN, HIGH);

    // Now do the rest of the message
    for(i = 0; i < SYMBOL_COUNT; i++)
    {digitalWrite(RED,HIGH);
    digitalWrite(YELLOW,HIGH);
      uint64_t frequency = (freq * 100) + (tx_buffer[i] * TONE_SPACING);
        si5351.set_freq(frequency, SI5351_CLK0);
        Serial.print("freq = ");
        Serial.println(tx_buffer[i]);
        proceed = false;
        digitalWrite(RED,LOW);
    digitalWrite(YELLOW,LOW);
        while(!proceed);
    }
    Serial.println("message done");
    // Turn off the output
    si5351.set_clock_pwr(SI5351_CLK0, 0);
    digitalWrite(TX_LED_PIN, LOW);
}
/*================== setup code */
void setup()
{
   Serial.begin(9600);
   Serial.println("------->WSPR-beacon setup");
   Serial.println("Modified by K5TNA");
   delay(1000);
  // Use the Arduino's on-board LED as a keying indicator.
  pinMode(TX_LED_PIN, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  digitalWrite(RED,HIGH);
  digitalWrite(YELLOW,HIGH);
  digitalWrite(TX_LED_PIN, HIGH);
  digitalWrite(GREEN, HIGH);
  delay(5000);
  digitalWrite(TX_LED_PIN, LOW);
  digitalWrite(RED,LOW);
  digitalWrite(YELLOW,LOW);
  Serial.begin(9600);

  // Set time sync provider
  //setSyncProvider(requestSync);  //set function to call when sync required

  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other
  // than 25 MHz
  /* check to see if the Si5351 is present
   */
  Serial.println("init Si5351"); delay(1000);
  if(!si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, CORRECTION)){
    Serial.println("si5351 init failed"); 
    delay(5000);
  }
    Serial.println("init Si5351 complete"); delay(10000);
  // Set CLK0 output
  si5351.set_freq(freq * 100, SI5351_CLK0);
  
  digitalWrite(GREEN, LOW);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power
  si5351.set_clock_pwr(SI5351_CLK0, 0); // Disable the clock initially
  digitalWrite(GREEN, LOW);
  // Set up Timer1 for interrupts every symbol period.
  noInterrupts();          // Turn off interrupts.
  TCCR1A = 0;              // Set entire TCCR1A register to 0; disconnects
                           //   interrupt output pins, sets normal waveform
                           //   mode.  We're just using Timer1 as a counter.
  TCNT1  = 0;              // Initialize counter value to 0.
  TCCR1B = (1 << CS12) |   // Set CS12 and CS10 bit to set prescale
    (1 << CS10) |          //   to /1024
    (1 << WGM12);          //   turn on CTC
                           //   which gives, 64 us ticks
  TIMSK1 = (1 << OCIE1A);  // Enable timer compare interrupt.
  OCR1A = WSPR_CTC;       // Set up interrupt trigger count;
  interrupts();            // Re-enable interrupts.
  digitalWrite(TX_LED_PIN,HIGH);
  Serial.println("call encode"); 
  encode(); // transmit once and stop
}


void loop()
{
   // blink LED when we've finished the transmit
  digitalWrite(TX_LED_PIN, HIGH);
  digitalWrite(GREEN, LOW);
  delay(2000);
  digitalWrite(TX_LED_PIN, LOW);
  digitalWrite(GREEN, HIGH);
  delay(2000);
  // Serial.println("Sending completed");
}
