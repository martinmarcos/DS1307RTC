/*
 * DS1307RTC.h - library for DS1307 RTC
  
  Copyright (c) Michael Margolis 2009
  This library is intended to be uses with Arduino Time library functions

  The library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  30 Dec 2009 - Initial release
  5 Sep 2011 updated for Arduino 1.0
  10 Sep 2017 updated to work with Wayne Truchsess' I2C library (Rev5)
 */

#define USE_I2C_LIBRARY

#if defined (__AVR_ATtiny84__) || defined(__AVR_ATtiny85__) || (__AVR_ATtiny2313__)
#include <TinyWireM.h>
#define Wire TinyWireM
#else
#ifdef USE_I2C_LIBRARY
#include <I2C.h>
#else //USE_I2C_LIBRARY
#include <Wire.h>
#endif //USE_I2C_LIBRARY
#endif
#include "DS1307RTC.h"

#define DS1307_CTRL_ID (uint8_t)0x68

DS1307RTC::DS1307RTC()
{
#ifdef USE_I2C_LIBRARY
  I2c.begin();
  I2c.timeOut((uint16_t) 5); //Set I2C timeout to 5ms
#else //USE_I2C_LIBRARY
  Wire.begin();
#endif //USE_I2C_LIBRARY
}
  
// PUBLIC FUNCTIONS
time_t DS1307RTC::get()   // Acquire data from buffer and convert to time_t
{
  tmElements_t tm;
  if (read(tm) == false) return 0;
  return(makeTime(tm));
}

bool DS1307RTC::set(time_t t)
{
  tmElements_t tm;
  breakTime(t, tm);
  return write(tm); 
}

// Acquire data from the RTC chip in BCD format
bool DS1307RTC::read(tmElements_t &tm)
{
  uint8_t sec;
#ifdef USE_I2C_LIBRARY
  if (I2c.write(DS1307_CTRL_ID,(uint8_t)0x00) != 0) {
    exists = false;
    return false;
  }
  exists = true;

  // request the 7 data fields   (secs, min, hr, dow, date, mth, yr)
  if (I2c.read(DS1307_CTRL_ID, (uint8_t)tmNbrFields)) return false;
  if (I2c.available() < tmNbrFields) return false;
  sec = I2c.receive();
  tm.Second = bcd2dec(sec & 0x7f);
  tm.Minute = bcd2dec(I2c.receive() );
  tm.Hour =   bcd2dec(I2c.receive() & 0x3f);  // mask assumes 24hr clock
  tm.Wday = bcd2dec(I2c.receive() );
  tm.Day = bcd2dec(I2c.receive() );
  tm.Month = bcd2dec(I2c.receive() );
  tm.Year = y2kYearToTm((bcd2dec(I2c.receive())));
#else //USE_I2C_LIBRARY
  Wire.beginTransmission(DS1307_CTRL_ID);
#if ARDUINO >= 100  
  Wire.write((uint8_t)0x00); 
#else
  Wire.send(0x00);
#endif  
  if (Wire.endTransmission() != 0) {
    exists = false;
    return false;
  }
  exists = true;

  // request the 7 data fields   (secs, min, hr, dow, date, mth, yr)
  Wire.requestFrom(DS1307_CTRL_ID, tmNbrFields);
  if (Wire.available() < tmNbrFields) return false;
#if ARDUINO >= 100
  sec = Wire.read();
  tm.Second = bcd2dec(sec & 0x7f);   
  tm.Minute = bcd2dec(Wire.read() );
  tm.Hour =   bcd2dec(Wire.read() & 0x3f);  // mask assumes 24hr clock
  tm.Wday = bcd2dec(Wire.read() );
  tm.Day = bcd2dec(Wire.read() );
  tm.Month = bcd2dec(Wire.read() );
  tm.Year = y2kYearToTm((bcd2dec(Wire.read())));
#else
  sec = Wire.receive();
  tm.Second = bcd2dec(sec & 0x7f);   
  tm.Minute = bcd2dec(Wire.receive() );
  tm.Hour =   bcd2dec(Wire.receive() & 0x3f);  // mask assumes 24hr clock
  tm.Wday = bcd2dec(Wire.receive() );
  tm.Day = bcd2dec(Wire.receive() );
  tm.Month = bcd2dec(Wire.receive() );
  tm.Year = y2kYearToTm((bcd2dec(Wire.receive())));
#endif
#endif //USE_I2C_LIBRARY
  if (sec & 0x80) return false; // clock is halted
  return true;
}

bool DS1307RTC::write(tmElements_t &tm)
{
#ifdef USE_I2C_LIBRARY
  // To eliminate any potential race conditions,
  // stop the clock before writing the values,
  // then restart it after.
  tm.Second |= 0x80;
  if (I2c.write(DS1307_CTRL_ID,0x00,(uint8_t*)&tm,(uint8_t)0x08) != 0) {
    exists = false;
    return false;
  }
  exists = true;

  // Now go back and set the seconds, starting the clock back up as a side effect
  tm.Second &= ~0x80;
  if (I2c.write(DS1307_CTRL_ID,(uint8_t)0x00,tm.Second) != 0) {
    exists = false;
    return false;
  }
#else //USE_I2C_LIBRARY
  // To eliminate any potential race conditions,
  // stop the clock before writing the values,
  // then restart it after.
  Wire.beginTransmission(DS1307_CTRL_ID);
#if ARDUINO >= 100  
  Wire.write((uint8_t)0x00); // reset register pointer  
  Wire.write((uint8_t)0x80); // Stop the clock. The seconds will be written last
  Wire.write(dec2bcd(tm.Minute));
  Wire.write(dec2bcd(tm.Hour));      // sets 24 hour format
  Wire.write(dec2bcd(tm.Wday));   
  Wire.write(dec2bcd(tm.Day));
  Wire.write(dec2bcd(tm.Month));
  Wire.write(dec2bcd(tmYearToY2k(tm.Year))); 
#else  
  Wire.send(0x00); // reset register pointer  
  Wire.send(0x80); // Stop the clock. The seconds will be written last
  Wire.send(dec2bcd(tm.Minute));
  Wire.send(dec2bcd(tm.Hour));      // sets 24 hour format
  Wire.send(dec2bcd(tm.Wday));   
  Wire.send(dec2bcd(tm.Day));
  Wire.send(dec2bcd(tm.Month));
  Wire.send(dec2bcd(tmYearToY2k(tm.Year)));   
#endif
  if (Wire.endTransmission() != 0) {
    exists = false;
    return false;
  }
  exists = true;

  // Now go back and set the seconds, starting the clock back up as a side effect
  Wire.beginTransmission(DS1307_CTRL_ID);
#if ARDUINO >= 100  
  Wire.write((uint8_t)0x00); // reset register pointer  
  Wire.write(dec2bcd(tm.Second)); // write the seconds, with the stop bit clear to restart
#else  
  Wire.send(0x00); // reset register pointer  
  Wire.send(dec2bcd(tm.Second)); // write the seconds, with the stop bit clear to restart
#endif
  if (Wire.endTransmission() != 0) {
    exists = false;
    return false;
  }
#endif //USE_I2C_LIBRARY
  exists = true;
  return true;
}

unsigned char DS1307RTC::isRunning()
{
#ifdef USE_I2C_LIBRARY
  if(I2c.write(DS1307_CTRL_ID,(uint8_t)0x00)) {
    exists = false;
    return 2;
  }
  exists = true;

  // Just fetch the seconds register and check the top bit
  if(I2c.read(DS1307_CTRL_ID,(uint8_t)0x01)) {
    exists = false;
    return 2;
  }
  exists = true;

  return !(I2c.receive() & 0x80);
#else //USE_I2C_LIBRARY
  Wire.beginTransmission(DS1307_CTRL_ID);
#if ARDUINO >= 100  
  Wire.write((uint8_t)0x00); 
#else
  Wire.send(0x00);
#endif  
  Wire.endTransmission();

  // Just fetch the seconds register and check the top bit
  Wire.requestFrom(DS1307_CTRL_ID, 1);
#if ARDUINO >= 100
  return !(Wire.read() & 0x80);
#else
  return !(Wire.receive() & 0x80);
#endif  
#endif //USE_I2C_LIBRARY
}

void DS1307RTC::setCalibration(char calValue)
{
  unsigned char calReg = abs(calValue) & 0x1f;
  if (calValue >= 0) calReg |= 0x20; // S bit is positive to speed up the clock
#ifdef USE_I2C_LIBRARY
  if(I2c.write(DS1307_CTRL_ID,(uint8_t)0x07,(uint8_t)calReg)) {
    exists = false;
    return;
  }
  exists = true;
#else //USE_I2C_LIBRARY
  Wire.beginTransmission(DS1307_CTRL_ID);
#if ARDUINO >= 100  
  Wire.write((uint8_t)0x07); // Point to calibration register
  Wire.write(calReg);
#else  
  Wire.send(0x07); // Point to calibration register
  Wire.send(calReg);
#endif
  Wire.endTransmission();  
#endif //USE_I2C_LIBRARY
}

char DS1307RTC::getCalibration()
{
#ifdef USE_I2C_LIBRARY
  if(I2c.write(DS1307_CTRL_ID,(uint8_t)0x07)) {
    exists = false;
    return 0x40;
  }
  exists = true;

  if(I2c.read(DS1307_CTRL_ID,(uint8_t)0x01)) {
    exists = false;
    return 0x40;
  }
  exists = true;

  unsigned char calReg = I2c.receive();
#else //USE_I2C_LIBRARY
  Wire.beginTransmission(DS1307_CTRL_ID);
#if ARDUINO >= 100  
  Wire.write((uint8_t)0x07); 
#else
  Wire.send(0x07);
#endif  
  Wire.endTransmission();

  Wire.requestFrom(DS1307_CTRL_ID, 1);
#if ARDUINO >= 100
  unsigned char calReg = Wire.read();
#else
  unsigned char calReg = Wire.receive();
#endif
#endif //USE_I2C_LIBRARY
  char out = calReg & 0x1f;
  if (!(calReg & 0x20)) out = -out; // S bit clear means a negative value
  return out;
}

// PRIVATE FUNCTIONS

// Convert Decimal to Binary Coded Decimal (BCD)
uint8_t DS1307RTC::dec2bcd(uint8_t num)
{
  return ((num/10 * 16) + (num % 10));
}

// Convert Binary Coded Decimal (BCD) to Decimal
uint8_t DS1307RTC::bcd2dec(uint8_t num)
{
  return ((num/16 * 10) + (num % 16));
}

bool DS1307RTC::exists = false;

DS1307RTC RTC = DS1307RTC(); // create an instance for the user
