# CN0349_ConductivitySensor
Analog Devices Water/Seawater Conductivity Sensor using Wire (Arduino) I2C library

This Library allows use of the CN0349 Board, the AD5933 chip, and/or the ADG715 chips to read water temperature, conductivity, and salinity.
The CN0349 is a 2 probe conductivity sensor!
The AD5933 chip needs to be calibrated to correctly measure conductivity.

## Requirements

  •EEPROM library from Arduino
  
  •Wire Library from Arduino
  
## Usage
 
 ### Definitions
 
```
void configureAD5934(uint8_t settlingTimes, float startFreq, float freqIncr, uint8_t numIncr);
```
configureAD5934(arguments) will configure the correct settling times, Start frequency, frequency increment, and number of increments when the chip tries to do a frequency sweep.
It will also call Wire.begin()
```
float calibrate(double rcal, double rfb);
```  
calibrate will do a frequency sweep across the known onboard resistors and get their Magnitude(impedance);
```
uint8_t measure(float GF_rtd, float GF, double NOS, float slope, float intercept, char state, float* T_imp, float* imp, float* Y_cell, float* T_cell, float* YT_cell); //high or low measurment ranges
``` 	
measure(arguments) will measure an the unknown water impedance(imp) in ohms->convert to conductivity(Y_cell) in mS/cm and measure the Pt100 RTD's impedance(T_imp) in ohms-> converting to temperature(T_cell) in degrees C.
Additionally it will calculate the salinity of the water using those properties.
I have included a linear regression argument in slope and intercept so the user can calibrate their values again if their probe is not a K=1.0 cell constant or is some custom configuration of pins.
Other wise use slope=1 and intercept=0

**measure(arguments) needs the gain factors and offsets to be calculated. Those can be calculated as per:** https://www.analog.com/media/en/reference-design-documentation/reference-designs/CN0349.pdf

**An example of calculating those singleton values can be seen in CN0349Test.ino.**

By default the library is setup to save these values to the atmega's EEPROM, the constant addresses can be found in CN0349.h.

