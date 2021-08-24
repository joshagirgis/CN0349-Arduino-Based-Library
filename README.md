# CN0349_ConductivitySensor
Analog Devices Conductivity Sensor using Wire (Arduino) I2C library

This library allows use of the CN0349 Board, the AD5943 chip, and/or the ADG715 chips to read temperature and impedance/admittance
The AD5934 chip needs to be calibrated to correctly measure the impedance/admittance and then one can use a conductivity probe to get water admittivity. However water admittivity is going to very very close to water conductivity so they should be very close to each other. 
**A full example of using the CN0349, and calculating Gain factors for first time use can be seen in CN0349Test.ino.**

## Wiring
https://wiki.analog.com/resources/eval/user-guides/circuits-from-the-lab/cn0349.
 <br />
This link provides a pinout near the bottom of the page but to remove confusion if you hold the board so that screw header faces you and the 8 pin header away: 
 <br />https://www.analog.com/en/design-center/reference-designs/hardware-reference-design/circuits-from-the-lab/cn0349.html
```
  PINS:
  1 3 5 7
  2 4 6 8
  ```
  This pinout corresponds to the following
```
  SCL,SDA,DGND,VDD
  SCL,SDA,DGND,VDD
```
Wire SDA to arduino's SDA(A4 for UNO) SCL (A5 for UNO). VDD to 3.3V, DGND to GND.
The LED should light up when correctly connected

## Software Requirements

  •EEPROM library from Arduino
  
  •Wire Library from Arduino
  
## Software Usage
 
 ### Definitions
 
```
void configureAD5934(uint8_t settlingTimes, float startFreq, float freqIncr, uint8_t numIncr);
```
configureAD5934(arguments) will configure the correct settling times, Start frequency, frequency increment, and number of increments when the chip tries to do a frequency sweep.
It will also call Wire.begin()
```
float calibrate(double rcal, double rfb);
```  
calibrate will do a frequency sweep across a known onboard resistor, with the help of a feedback resistor, using the ADG715 (octal mux) and get their "Magnitude". The onboard resistors are as follows:
```
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
```
The 1 and 2 modes are for calculating the gain factors for measure below.
```
uint8_t CN0349::measure(float GF_rtd, float GF, double NOS, char state, float* T_imp, float* rawimp, float* Y_cell, float* T_cell)  //high or low measurment ranges

``` 	
measure(arguments) will measure an the unknown water impedance(imp) in ohms->convert to admittance(Y_cell) in mS and measure a Pt100 RTD's impedance(T_imp) in ohms-> converting to temperature(T_cell) in degrees C.

The CN0349 uses the ADG715 to switch precision feedback resistors to measure the Pt100 rtd and conductivity. There are three (/two) modes, RTD, High and LOW. 
```
//measuring
//      Rfb(ohms)  |Channels
//RTD:   R3(100)       1,7
//High:  R3(100)       1,8
//Low:   R4(1000)      2,8
```

**measure(arguments) needs the gain factors and offsets (GF_rtd, GF, NOS) to be calculated. Those can be calculated as per:** 
https://www.analog.com/media/en/reference-design-documentation/reference-designs/CN0349.pdf

**An example of calculating those singleton values can be seen in CN0349Test.ino.**

By default the library is setup to save these values to the atmega's EEPROM, the constant addresses can be found in CN0349.h. NOTE: the EEPROM for the atmega chip is cleared after reprogramming with an AVR ICSP programmer, look into setting the fuses to not clear the EERPROM when reprogamming.

## RF Considerations
The CN0349 may cause RF interference. (GPS/MODEM BEWARE). This is from the ADuM isolator parts. For redesign purposes, there are a few techniques on how to handle this, and thankfully someone wrote them up in a nice little app note. 
 <br />https://www.analog.com/media/en/technical-documentation/application-notes/AN-0971.pdf.
 <br />
One easy option, without redesign, is to move the CN0349 farther away from other RF parts. 
 <br />
If a compact system is needed one can remove the ADuM isolator parts, if they are intent on reducing the RF emissions. The parts to remove would be the ADuM5000 and the ADuM1250. NOTE: this defeats the "Fully, Isolated" part of the sensor.
 <br />
Once this is done one would have to also remove R16 and R17 pullup resistors, short the VDD to VDDiso, and short GND to GNDiso.
 <br />
The respective board should look like this (note the desoldered resistors):
 <br />
https://imgur.com/a/p5SC4JI
 <br />
A better design could include an I2C optical isolator such as:
 <br />
https://www.digikey.com/product-detail/en/ixys-integrated-circuits-division/CPC5902G/CLA380-ND/2816056
 <br />
or an RF shield
