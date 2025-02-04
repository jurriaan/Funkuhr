/**
 * Funkuhr.h - Library for interacting with DCF77 radio clock modules.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <Arduino.h>

#ifndef Funkuhr_h
#define Funkuhr_h
 
struct Dcf77Time
{
  uint8_t min;
  uint8_t hour;
  uint8_t day;
  uint8_t month;
  uint8_t year;
  uint8_t zone;
};

typedef void (*sync_callback)(Dcf77Time time);
class Funkuhr
{
  public:
    Funkuhr();
    void init(sync_callback callback);
    unsigned long long getBuffer();
};

#endif
