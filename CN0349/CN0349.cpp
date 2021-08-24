/*
  CN0349.cpp
  MIT License
Copyright (c) 2017 
Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, 
subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE 
USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Wire.h>// used for I2C communication
#include "avr/pgmspace.h"
#include "CN0349.h"
#include <stdlib.h>
#include <inttypes.h>
#include <util/delay.h>

void CN0349::configureAD5934(uint8_t settlingTimes, float startFreq, float freqIncr, uint8_t numIncr) {
  Wire.begin();
  _delay_ms(1000);
  setNumberOfSettlingTimes(settlingTimes);
  setStartFrequency(startFreq);
  setFrequencyIncrement(freqIncr);
  setNumberOfIncrements(numIncr);
}
///////////////////////////////////////////////////////////////////////////////////////////
/////////AD5934
///////////////////////////////////////////////////////////////////////////////////////////
bool CN0349::AD5934byteWrite(uint8_t address, uint8_t data) {
  Wire.beginTransmission(AD5934_ADDR);
  Wire.write(address); // address specifier
  Wire.write(data); // value specifier
  uint8_t i2cStatus = Wire.endTransmission();
  _delay_ms(1);
  if (i2cStatus)
    return false;
  else
    return true;
}

uint16_t CN0349::AD5934byteRead(uint8_t address) {
  uint8_t rxByte;
  Wire.beginTransmission(AD5934_ADDR);
  Wire.write(address); // address specifier
  int i2cStatus = Wire.endTransmission();
  _delay_ms(1);
  Wire.requestFrom(AD5934_ADDR, 1);
  if (1 <= Wire.available()) {
    rxByte = Wire.read();
  }
  else {
    rxByte = -1;
  }
  return rxByte;
}

uint16_t CN0349::checkStatus() {
  return (AD5934byteRead(STATUS_REGISTER[0]) & 7);
}

// start frequency and frequency increment formula:
uint8_t CN0349::frequencyCode(float freqInHz, uint8_t byteNum) {
  long value = long((freqInHz / (CLOCK_SPEED / 16)) * pow(2, 27));
  uint8_t code[3];
  code[0] = (value & 0xFF0000) >> 0x10;
  code[1] = (value & 0x00FF00) >> 0x08;
  code[2] = (value & 0x0000FF);
  return code[byteNum];
}

bool CN0349::setStartFrequency(float freqInHz) {
  bool statusValue;
  for (uint8_t n = 0; n < 3; n++) {
    statusValue = AD5934byteWrite(START_FREQUENCY_REGISTER[n], frequencyCode(freqInHz, n));
  }
  return statusValue;
}

bool CN0349::setFrequencyIncrement(float freqInHz) {
  bool statusValue;
  for (uint8_t n = 0; n < 3; n++) {
    statusValue = AD5934byteWrite(FREQ_INCREMENT_REGISTER[n], frequencyCode(freqInHz, n));
  }
  return statusValue;
}

bool CN0349::setNumberOfIncrements(uint8_t n) {
  bool i2cStatus;
  uint8_t numIncrements = (n)<(511)?(n):(511);
  i2cStatus = AD5934byteWrite(NUM_INCREMENTS_REGISTER[0], numIncrements >> 8);
  i2cStatus = AD5934byteWrite(NUM_INCREMENTS_REGISTER[1], numIncrements & 255);
  return i2cStatus;
}

bool CN0349::setNumberOfSettlingTimes(uint8_t n) {
  int decode;
  int numSettlingTimes = (n)<(2044)?(n):(2044);
  if (n > 1023) { // put into 9 bit
    decode = 3; // times 4
    numSettlingTimes /= 4;
  }
  else if (n > 511) { // put into 9 bit!
    decode = 1; //times 2
    numSettlingTimes /= 2;
  }
  else {  // within 9 bit range
    decode = 0; //default
    numSettlingTimes = n;
  }
  bool i2cStatus;
   //get MSB add decode value with a 0 at the end to include the MSB
  // ignore first 8 bits
  i2cStatus = AD5934byteWrite(NUM_SETTLING_CYCLES_REGISTER[0], (numSettlingTimes >> 8) + (decode << 1));
  //put 8 bit number in the second address.
  i2cStatus = AD5934byteWrite(NUM_SETTLING_CYCLES_REGISTER[1], numSettlingTimes & 255);
  return i2cStatus;
}

bool CN0349::setControlRegister(uint8_t code) {
  uint8_t rxByte = AD5934byteRead(CONTROL_REGISTER[0]);
  rxByte &= 0x0F; // clear upper four bits
  rxByte |= code << 4; // set to 1011
  bool s = AD5934byteWrite(CONTROL_REGISTER[0], rxByte);
  _delay_ms(1);
  return s;
}

bool CN0349::setControlRegister2() { //initalize D11 D10 D9 D8 @0x80 Excitation Voltage 2.0Vp-p, Internal PGA=1
  uint8_t rxByte = AD5934byteRead(CONTROL_REGISTER[0]);
  rxByte &= 0xF0; // clear lower four bits (11110000)
  rxByte |= 0b00000001; // set to 0001 Excitation Voltage 2.0Vp-p, Internal PGA=1
  bool s = AD5934byteWrite(CONTROL_REGISTER[0], rxByte);
  _delay_ms(10);
  return s;
}

float CN0349::sweep(uint8_t switch1, uint8_t switch2) { //performs frequency sweep for real and unreal components, returns the magnitude
  float magnitude;
  //configureAD5934(15, 8 * pow(10, 3), 4, 2);    // number of settling times ,start frequency (Hz),frequency increment (Hz), number of increments
  //configureAD5934(2, 1 * pow(10, 5),4,0 );
  //configureAD5934(2, 1 * pow(10, 3), 1 * pow(10, 4),0 );
  //configureAD5934(2, 1 * pow(10, 3), 1 * pow(10, 3),100 );
  //_delay_ms(10);
  setControlRegister2();
 //_delay_ms(10);
  ADG715reset();            //clear out switches
  //_delay_ms(10);
  ADG715writeChannel(switch1, 1); //turn on switchs
  //_delay_ms(10);
  ADG715writeChannel(switch2, 1);
  //_delay_ms(10);
  int real = 0;
  int imag = 0;
  ////////////////////////////////////////////////0.Inizialize bit D11,D10,D9,D8
  setControlRegister2();
  ////////////////////////////////////////////////1. place AD5934 in standby mode
  setControlRegister(STANDBY);
  ////////////////////////////////////////////////2. initialize with start frequency
  setControlRegister(INITIALIZE);
  _delay_ms(100);
  ////////////////////////////////////////////////3. start frequency sweep
  setControlRegister(START_SWEEP);
  //_delay_ms(100);
  ////////////////////////////////////////////////4. poll status register until complete
  //while (checkStatus() < 6) {
  while (checkStatus() < 4) {
   //for (int i = 0; i < 10; i++) {
    //print(checkStatus());
    //_delay_ms(80);
    if (checkStatus() == validImpedanceData) {
      // 5. read values
      real = AD5934byteRead(REAL_DATA_REGISTER[0]) << 8;
      real |= AD5934byteRead(REAL_DATA_REGISTER[1]);

     // if (real > 0x7FFF) { // negative value
     //   real &= 0x7FFF;
     //   real -= 0x10000;
     // }
      imag = AD5934byteRead(IMAG_DATA_REGISTER[0]) << 8;
      imag |= AD5934byteRead(IMAG_DATA_REGISTER[1]);
     // if (imag > 0x7FFF) { // negative value
     //   imag &= 0x7FFF;
     //   imag -= 0x10000;
     // }
      magnitude = sqrt(pow(double(real), 2) + pow(double(imag), 2));
	    double phase = atan(double(imag) / double(real)); // if you ever need it
      setControlRegister(INCREMENT);
	  //setControlRegister(REPEAT_FREQ);
    }
  //}
 // _delay_ms(80);
  }
  setControlRegister(POWER_DOWN);
  return magnitude;
}

float CN0349::calibrate(double rcal, double rfb) {
  float magnitude;
  int switch1, switch2 = 0;

  //reference:
  //      Rcal(ohms) | Rfb(ohms)  |Channelscalibrate
  //RTD:   R3(100)     R9(100)      4,1
  //High1: R3(100)     R9(100)      4,1
  //High2: R4(1000)    R9(100)      5,1
  //Low1:  R4(1000)    R8(1000)     5,2
  //Low2:  R7(10000)   R8(1000)     6,2

  if (rcal == 100 && rfb == 100) { //rtd and 1st high calibration, r3, r9
    switch1 = 4;
    switch2 = 1;
  }
  else if (rcal == 1000 && rfb == 100) { //2nd high calibration r4, r9
    switch1 = 5;
    switch2 = 1;
  }
  else if (rcal == 1000 && rfb == 1000) { //1st low calibration r4, r8
    switch1 = 5;
    switch2 = 2;
  }
  else if (rcal == 10000 && rfb == 1000) { //1st low calibration r7, r8
    switch1 = 6;
    switch2 = 2;
  }
  else {
    rfb = 0;
    rcal = 0;
  }
  if (!(rfb && rcal == 0)) {
    magnitude = sweep(switch1, switch2);
  }
  else {
    magnitude = 0;
  }
  return magnitude;
};

uint8_t CN0349::measure(float GF_rtd, float GF, double NOS, char state, float* T_imp, float* rawimp, float* Y_cell, float* T_cell) {  //high or low measurment ranges
  const float A = 3.9083 * pow(10, -3);
  const float B = -5.775 * pow(10, -7);
  float magnitude = 0;
  int switch1, switch2 = 0;
  int flag = 1;
  float NX, YX = 0;
  //reference
  //      Rfb(ohms)  |Channels
  //RTD:   R9(100)       1,7
  //High:  R9(100)       1,8
  //Low:   R8(1000)      2,8

  //rtd measurement rfb = r9 also path to thermistor
  switch1 = 1;  //measure rtd first
  switch2 = 7;
  magnitude = sweep(switch1, switch2);    //measure temperature
  *T_imp = 1 / (magnitude * GF_rtd);
  *T_cell = (-A + sqrt(pow(A, 2) - 4 * B * (1 - *T_imp / 100))) / (2 * B); //convert impedence to temperature (known pt100 formula)
  //Rt = R0 * (1 + A* t + B*t2 + C*(t-100)* t3)

  if (state == 'H') { //high measurement rfb = r9
    switch1 = 1;
    switch2 = 8;
  }
  else if (state == 'L') { //low measurement rfb=r8
    switch1 = 2;
    switch2 = 8;
  }
  else {
    flag = 0;
	return 0;
  }
  if (!(flag = 0)) {
    magnitude = sweep(switch1, switch2);      //get conductance magnitude
    //Serial.println(magnitude, 30);
  }
  else {
    magnitude = 0;
  }
  // three point calibration equation
  //YX = (NX-NOS)*GF
  //YCELL = YX / (1 - 100 * YX);
  *rawimp = 1 / (magnitude * GF);
  NX = magnitude;
  YX = (NX - NOS) * GF;
  if (state == 'H') { //high measurement rfb = r9
      *Y_cell = YX / (1 - R2 * YX) * 1000; //1/ohms, go to mS
   // *Y_cell = YX / (1 - R2 * YX) ; //1/ohms, go from S to mS

  }
  else {
    *Y_cell = YX * 1000;  //go to S/m
    //*Y_cell = YX;  //go from S/m to mS/cm

  }
  
  return 1;
};


///////////////////////////////////////////////////////////////////////////////////////////
/////////ADG715
///////////////////////////////////////////////////////////////////////////////////////////
uint8_t CN0349::ADG715CH(uint8_t channel) {   //checks channel numbers
  channel -= 1; //assume input is 1-8 -> 0-7
  if (channel < 0)
    channel = 0;
  else if (channel > 7)
    channel = 7;
  return channel;
};

//return status as a byte of all channel (1-8)
uint8_t CN0349::ADG715read(uint8_t channel) {  //if channel exceeds 9, read all
  uint8_t value = 255; //error possibly?
  Wire.requestFrom(ADG715_ADDR, 0x01); //request one byte from address
  while (Wire.available())
    value = Wire.read(); //grab one byte
  if (channel < 9) //1-8
  {
    channel = ADG715CH(channel); //resize to 0-7
    value = (((value) >> (channel)) & 0x01);
  }
  return value; //return all
};

void CN0349::ADG715writeChannel(uint8_t channel, uint8_t state) { //change status of a specified channel (1-8)
  uint8_t value;
  value = ADG715read(9); //read all channels
  state ? ((value) |= (1UL << (ADG715CH(channel)))) : ((value) &= ~(1UL << (ADG715CH(channel))));
  Wire.beginTransmission(ADG715_ADDR);
  Wire.write(value);
  Wire.endTransmission();
};

void CN0349::ADG715reset() { //clear out register
  Wire.beginTransmission(ADG715_ADDR);
  Wire.write(0x00); //clear out register
  Wire.endTransmission();
};
