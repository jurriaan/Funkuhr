/**
 * Funkuhr.cpp - Library for interacting with DCF77 radio clock modules.
 *
 * Based on the Arduino DCF77 decoder v0.2 by Mathias Dalheimer (md@gonium.net).
 * Adapted by Andreas Tacke (at@mail.fiendie.net).
 * Adapted by Jurriaan Pruis (email@jurriaanpruis.nl).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "Funkuhr.h"


#define DCF77PIN 2  // Input pin for the DCF receiver
#define BLINKPIN 13 // LED indicator output
#define DCF_min_millis 50
#define DCF_max_millis 250
#define DCF_split_millis 150 // Number of milliseconds before we assume a logic 1
#define DCF_sync_millis 1200 // No signal at second 59

// DCF time format struct
struct DCF77Buffer
{
  unsigned long long prefix :17;
  unsigned long long zoneoffset: 2;
  unsigned long long prefix2: 2;
  unsigned long long Min :7; // minutes
  unsigned long long P1 :1; // parity minutes
  unsigned long long Hour :6; // hours
  unsigned long long P2 :1; // parity hours
  unsigned long long Day :6; // day
  unsigned long long Weekday :3; // day of week
  unsigned long long Month :5; // month
  unsigned long long Year :8; // year (5 -> 2005)
  unsigned long long P3 :1; // parity
};

// Parity struct
volatile struct
{
  uint8_t parity_min :1;
  uint8_t parity_hour :1;
  uint8_t parity_date :1;
} flags;


// Clock variables
volatile uint8_t DCFSignalState = 0;
volatile uint8_t previousSignalState;
unsigned long previousFlankTime;
int bufferPosition;
unsigned long long dcf_rx_buffer;

sync_callback callback;


/**
 * Calculates the parity
 */
void calculateParity() {
  uint8_t parity = 0;
  for(int pos = 21; pos < 60; pos++)
  {
    // Update the parity bits. First: Reset when minute, hour or date starts.
    if (pos == 29 || pos == 36) parity = 0;
    
    // Save the parity when the corresponding segment ends
    if (pos == 28) { flags.parity_min = parity; };
    if (pos == 35) { flags.parity_hour = parity; };
    if (pos == 58) { flags.parity_date = parity; };
    parity = parity ^ ((dcf_rx_buffer>>pos) & 1);
  }
}

/**
 * Evaluates the information stored in the buffer. This is where the DCF77
 * signal is decoded and the callback is triggered
 */
void finalizeBuffer(void)
{
  if (bufferPosition > 44)
  {
    struct DCF77Buffer *rx_buffer;
    dcf_rx_buffer = dcf_rx_buffer << (59 - bufferPosition);
    calculateParity();
    rx_buffer = (struct DCF77Buffer *)(unsigned long long)&dcf_rx_buffer;
    
    if (flags.parity_min == rx_buffer->P1 &&
    flags.parity_hour == rx_buffer->P2 &&
    flags.parity_date == rx_buffer->P3)
    {
      Dcf77Time time;
      // Convert the received bits from BCD to decimal
      time.min = rx_buffer->Min-((rx_buffer->Min/16)*6);
      time.hour = rx_buffer->Hour-((rx_buffer->Hour/16)*6);
      time.zone = rx_buffer->zoneoffset-((rx_buffer->Hour/16)*6);
      time.day = rx_buffer->Day-((rx_buffer->Day/16)*6);
      time.month = rx_buffer->Month-((rx_buffer->Month/16)*6);
      time.year = rx_buffer->Year-((rx_buffer->Year/16)*6);
      if(time.month != 0 && time.year != 0) callback(time);
    }
  }
  
  bufferPosition = 0;
  dcf_rx_buffer=0;
}


/**
 * Append a signal to the dcf_rx_buffer. Argument can be 1 or 0. An internal
 * counter shifts the writing position within the buffer. If position > 59,
 * a new minute begins -> time to call finalizeBuffer().
 */
void appendSignal(bool signal)
{
  dcf_rx_buffer = dcf_rx_buffer | ((unsigned long long) signal << bufferPosition);
  
  bufferPosition++;
  
  if (bufferPosition > 59)
  finalizeBuffer();
}

/**
 * Evaluates the signal as it is received. 
 */
void scanSignal(void)
{
  unsigned long thisFlankTime = millis();
  if (DCFSignalState == 1)
  {
    if (thisFlankTime - previousFlankTime > DCF_sync_millis)
    {
      finalizeBuffer();
    }
    
    else if (thisFlankTime - previousFlankTime < 300)
    {
      bufferPosition--;
      
      if (bufferPosition < 0)
      bufferPosition = 0;
    }
    
    if (thisFlankTime - previousFlankTime > 300)
    previousFlankTime = thisFlankTime;
  }
  unsigned int diff = thisFlankTime - previousFlankTime;
  if (diff > DCF_min_millis && diff < DCF_max_millis)
  {
    appendSignal(millis() - previousFlankTime > DCF_split_millis);
  }
}

/**
* Interrupt handler for INT0. Called when the signal on Pin 2 changes.
*/
void int0handler()
{
  // Inverted because the signal is fed through a transistor
  DCFSignalState = digitalRead(DCF77PIN);
  if (DCFSignalState != previousSignalState) {
    scanSignal();
    digitalWrite(BLINKPIN, !!DCFSignalState);
    previousSignalState = DCFSignalState;
  }
  
}


/**
* Initialize the variables and configure the interrupt behaviour.
*/
void Funkuhr::init(sync_callback gotSync)
{
  previousSignalState = 0;
  previousFlankTime = 0;
  bufferPosition = 0;
  dcf_rx_buffer = 0;
  callback = gotSync;
  pinMode(DCF77PIN, INPUT);
  attachInterrupt(0, int0handler, CHANGE);
}

/**
* Constructor
*/
Funkuhr::Funkuhr()
{
  
}

/**
* Take a look at the current buffer (for debugging)
*/
unsigned long long Funkuhr::getBuffer() {
  return dcf_rx_buffer;
}