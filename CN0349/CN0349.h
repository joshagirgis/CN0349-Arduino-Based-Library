

#ifndef CN0349_h
#define CN0349_h

#include <Wire.h>// used for I2C communication
#include "avr/pgmspace.h"
#define AD5934_ADDR 0x0D   //i2c addresses of CN-0349
#define ADG715_ADDR 0x48
#include <EEPROM.h> 
#include <stdlib.h>
#include <inttypes.h>
#include <util/delay.h>
#include <math.h>

///////////////////////////////////////////////////////////////////////////////////////////
//Constants
///////////////////////////////////////////////////////////////////////////////////////////
//cn-0349
const float PROGMEM CLOCK_SPEED = 1 * pow(10, 6);//16.776 * pow(10, 6); // AD5934 input clock of 1 MHz (on board clock generator)
const uint8_t PROGMEM validImpedanceData = 2;   //when the data is good
const uint8_t PROGMEM validSweep = 4;   // when the sweep is over
//const float alpha = 0.021; // 2.1%°C is the temperature coefficient at 25°C for sea water
//Resistors:
const float PROGMEM R2 = 100;
const float PROGMEM R3 = 100;
const float PROGMEM R4 = 1000;
const float PROGMEM R7 = 10000;
const float PROGMEM R8 = 1000;
const float PROGMEM R9 = 100;
const uint8_t PROGMEM GF_low_addr =1;
const uint8_t PROGMEM NOS_low_addr=4;
const uint8_t PROGMEM GF_high_addr=6;
const uint8_t PROGMEM NOS_high_addr=12;
const uint8_t PROGMEM calibState_addr=32;
///////////////////////////////////////////////////////////////////////////////////////////
//Register Map of CN-0349
///////////////////////////////////////////////////////////////////////////////////////////
const uint8_t PROGMEM CONTROL_REGISTER[2] =                    { 0x80, 0x81       }; // see mapping below
const uint8_t PROGMEM START_FREQUENCY_REGISTER[3] =            { 0x82, 0x83, 0x84 }; // 24 bits for start frequency
const uint8_t PROGMEM FREQ_INCREMENT_REGISTER[3] =             { 0x85, 0x86, 0x87 }; // 24 bits for frequency increment
const uint8_t PROGMEM NUM_INCREMENTS_REGISTER[2] =             { 0x88, 0x89       }; //  9 bits for # of increments
const uint8_t PROGMEM NUM_SETTLING_CYCLES_REGISTER[2] =        { 0x8A, 0x8B       }; //  9 bits + 1 modifier for # of settling times
const uint8_t PROGMEM STATUS_REGISTER[1] =                     { 0x8F             }; // see mapping below
const uint8_t PROGMEM REAL_DATA_REGISTER[2] =                  { 0x94, 0x95       }; // 16-bit, twos complement format
const uint8_t PROGMEM IMAG_DATA_REGISTER[2] =                  { 0x96, 0x97       }; // 16-bit, twos complement format

///////////////////////////////////////////////////////////////////////////////////////////
//Control register map @ 0x80, 0x81 of CN-0349
///////////////////////////////////////////////////////////////////////////////////////////
// Note:
//0x80 contains D15 to D8
//0x81 contains D7 to D0

// Control register map (D15 to D12)
const uint8_t PROGMEM DEFAULT_VALUE =  0b0000; // initial setting
const uint8_t PROGMEM INITIALIZE    =  0b0001; // excite the unknown impedance initially
const uint8_t PROGMEM START_SWEEP   =  0b0010; // begin the frequency sweep
const uint8_t PROGMEM INCREMENT     =  0b0011; // step to the next frequency point
const uint8_t PROGMEM REPEAT_FREQ   =  0b0100; // repeat the current frequency point measurement
const uint8_t PROGMEM POWER_DOWN    =  0b1010; // VOUT and VIN are connected internally to ground
const uint8_t PROGMEM STANDBY       =  0b1011; // VOUT and VIN are connected internally to ground

// D11 = no operation

// Control register map (D10 to D9)
// D8 = PGA gain (0 = x5, 1 = x1) // amplifies the response signal into the ADC
// D7 = reserved, set to 0
// D6 = reserved, set to 0
// D5 = reserved, set to 0
// D4 = reset // interrupts a frequency sweep
// D3 = external system clock, set to 1; internal system clock, set to 0
// D2 = reserved, set to 0
// D1 = reserved, set to 0
// D0 = reserved, set to 0

///////////////////////////////////////////////////////////////////////////////////////////
//Start Frequency Register Code @ 0x82, 0x83, 0x84 of CN-0349
///////////////////////////////////////////////////////////////////////////////////////////
// Start Frequency Code = (2^27)*(startFreqHz)/(CLOCK_SPEED/16)
// however it needs to be split into three values because it is has three registers

///////////////////////////////////////////////////////////////////////////////////////////
// number of increments register @ 0x88, 0x89 of CN-0349
///////////////////////////////////////////////////////////////////////////////////////////
// 0x88: D15 to D9 -- don't care
// 0x89: D8 -- number of increments bit 1
// 0x89: D7 to D0 -- number of increments bits 2 through 9; 9-bit integer number stored in binary format

///////////////////////////////////////////////////////////////////////////////////////////
// number of settling times @ 0x8A, 0x8B of CN-0349
///////////////////////////////////////////////////////////////////////////////////////////
// 0x8A: D15 to D11 -- don't care
// 0x8A: D10 to D9 -- 2-bit decode
//        0 0 = default
//        0 1 = # of cycles x 2
//        1 0 = reserved
//        1 1 = # of cycles x 4
// 0x8A: D8 -- MSB number of settling times
// 0x8B: D7 to D0 -- number of settling times; 9-bit integer number stored in binary format

///////////////////////////////////////////////////////////////////////////////////////////
//Status Register @ 0x8F of CN-0349
///////////////////////////////////////////////////////////////////////////////////////////
//0000 0001 Reserved
//0000 0010 Valid real/imaginary data
//0000 0100 Frequency sweep complete
//0000 1000 Reserved
//0001 0000 Reserved
//0010 0000 Reserved
//0100 0000 Reserved
//1000 0000 Reserved

///////////////////////////////////////////////////////////////////////////////////////////
//ADG715 switches of CN-0349
///////////////////////////////////////////////////////////////////////////////////////////
//calibrating
//      Rcal(ohms) | Rfb(ohms)  |Channels
//RTD:   R3(100)     R9(100)      4,1
//High1: R3(100)     R9(100)      4,1
//High2: R4(1000)    R9(100)      5,1
//Low1:  R4(1000)    R8(1000)     5,2
//Low2:  R7(10000)   R8(1000)     6,2

//measuring
//      Rfb(ohms)  |Channels
//RTD:   R3(100)       1,7
//High:  R3(100)       1,8
//Low:   R4(1000)      2,8

class CN0349{
	public:
		void configureAD5934(uint8_t settlingTimes, float startFreq, float freqIncr, uint8_t numIncr);
		float calibrate(double rcal, double rfb);
		uint8_t measure(float GF_rtd, float GF, double NOS, char state, float* T_imp, float* rawimp, float* Y_cell, float* T_cell);
	private:
	    uint16_t checkStatus();
		bool AD5934byteWrite(uint8_t address, uint8_t data);
		uint16_t AD5934byteRead(uint8_t address);
		uint8_t frequencyCode(float freqInHz, uint8_t byteNum);
		bool setStartFrequency(float freqInHz);
		bool setFrequencyIncrement(float freqInHz);
		bool setNumberOfIncrements(uint8_t n);
		bool setNumberOfSettlingTimes(uint8_t n);
		bool setControlRegister(uint8_t code);
		bool setControlRegister2(); //initalize D11 D10 D9 D8 @0x80 Excitation Voltage 2.0Vp-p, Internal PGA=1
		float sweep(uint8_t switch1, uint8_t switch2); //performs frequency sweep for real and unreal components, returns the magnitude
		float tempcondtosal(float cond, float temp); //convert microsiemens to salinity, valid for 2 to 42 ppt
		uint8_t ADG715CH(uint8_t channel);   //checks channel numbers
		uint8_t ADG715read(uint8_t channel);  //if channel exceeds 9, read all
		void ADG715writeChannel(uint8_t channel, uint8_t state); //change status of a specified channel (1-8)
		void ADG715reset(); //clear out register
		};
#endif
