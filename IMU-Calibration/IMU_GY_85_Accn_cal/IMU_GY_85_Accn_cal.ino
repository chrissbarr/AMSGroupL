/*
Arduino Code for the calibration of the Accelerometer found on the GY-85 9DoF IMU
Written by Russell SA Brinkworth 2014
Based on code found in FreeIMU Arduino examples by Fabio Varesano, Filipe Vieira and TJS
Using ADXL345 (accelerometer)

Using a flat surface put the accelerometer in each of the 6 3D orientations that causes
1g to be experienced by only on of the axes. Find the maximum and minimum values for
each axis and use them to calculate the gain and offset for the accelerometer

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

See <http://www.gnu.org/licenses/> for a copy of the GNU General Public License.
*/

#include <Wire.h> // I2C library
#include "ADXL345.h" // acc library

ADXL345 Accel;

int raw_data[3] = {0}; // raw data from accelerometer
int ave_data[3] = {0}; // averaged data

void setup(void)
{
  Serial.begin(9600);
  Wire.begin();  
  Serial.println("Initialising accelerometer...");
  Accel.init(ADXL345_ADDR_ALT_LOW); // intialise accelerometer
  delay(2);  // Wait for Vcc Stabilization 
  //The accelerometer is set to a default rate of 100Hz the I2C is capable up to 200Hz Bandwidth
  // to change the bitrate, call the function Accel.setRate(val) where val is a float representing data rate (refer to datasheet)
  // or use Accel.ser_bw(code) where code is a byte explained in ADXL345.h
  Accel.set_bw(ADXL345_BW_100);  // Set the bandwith to 100Hz and therefore the Output data rate (ODR) to 200Hz
  Accel.writeTo(ADXL345_DATA_FORMAT, 0x0B);  //+-2g, 10 bit mode ~4mg/LSB 
  Accel.writeTo(ADXL345_POWER_CTL, 0x08);    // Start measurement 
  delay(7); // Delay fom datasheet 1.1ms + 1/ODR where ODR = 200Hz
  // use offsets if a channel saturates > +/-1024
  Accel.writeTo(ADXL345_OFSX, (0 & 0xFF)); // fixed offsets for the x axis, range = +/-127
  Accel.writeTo(ADXL345_OFSY, (0 & 0xFF));
  Accel.writeTo(ADXL345_OFSZ, (0 & 0xFF));
  Accel.gains[0] = 1;  // Use if want to change gain in the hardware
  Accel.gains[1] = 1;
  Accel.gains[2] = 1;
  Serial.println("Calibrating accelerometer...");
}

void loop()
{ 
  Accel.readAccel(raw_data);
  
  for (int i = 0; i < 32; i++) // sum 32 data points 
  {
    Accel.readAccel(raw_data);
    ave_data[0] += raw_data[0];
    ave_data[1] += raw_data[1];
    ave_data[2] += raw_data[2];
    delay(25);
  }

  for (int j = 0; j < 2; j++)
  {
    ave_data[j] = ave_data[j] >> 5; // divide by 32 to find average
    Serial.print(ave_data[j]);
    Serial.print(", ");
  }
  ave_data[2] = ave_data[2] >> 5;
  Serial.println(ave_data[2]);
}
