/*
Arduino Code for the calibration of the Magnetometer found on the GY-85 9DoF IMU
Written by Russell SA Brinkworth 2014
Based on code found in FreeIMU Arduino examples by Fabio Varesano, Filipe Vieira and TJS
HMC5883 (magnotometer)

Move the manetometer around in all orientations to correctly find the max and min values for all 3 axes

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
#include <HMC58X3.h> // mag library

HMC58X3 magn;

float raw_data[3] = {0.0, 0.0, 0.0}; // magnetometer needs somewhere to put raw data into
float filtered_data[3] = {0.0}; // filtered magnetometer values
float max_values[3] = {-5000.0, -5000.0, -5000.0}; // array to hold max values. Initilise to large negative values
float min_values[3] = {5000.0, 5000.0, 5000.0}; // array to hold min values. Initilise to large negative values
#define TCmag 10.0 // time constant for smoothing raw values

#define loop_delay 40000 // loop delay in us. This is the desired time between loops.
unsigned long loop_time = micros();   // loop time place holder for loop delay calculation
int loop_number = 0;

void setup(void)
{  
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Initialising magnetometer...");
  magn.init(true); // no delay needed as we have already a delay(5) in HMC58X3::init()
  Serial.println("Calibrating magnetometer...");
  magn.calibrate(1, 32); // Calibrate HMC using self test, not recommended to change the gain after calibration. (Gain, number of sample). Use gain 1=default, valid 0-7, 7 not recommended.
  magn.setMode(0); // Single mode conversion was used in calibration, now set continuous mode
  delay(40); // make sure magnetometer is ready
  magn.getValues(&filtered_data[0],&filtered_data[1],&filtered_data[2]); // Get initial value to set filter history
}

void loop()
{   
  magn.getValues(&raw_data[0],&raw_data[1],&raw_data[2]); // get raw magnetometer values
  for (int i = 0; i <= 2; i++)
  {
    filtered_data[i] = (1/TCmag)*raw_data[i] + (1 - 1/TCmag)*filtered_data[i]; // filter the raw values
    if(filtered_data[i] > max_values[i]) // find max value
    {
      max_values[i] = filtered_data[i];
    }
    else if(filtered_data[i] < min_values[i]) // find min value
    {
      min_values[i] = filtered_data[i];
    }
  }
  if(loop_number == 0) // do not print every loop
  {
    Serial.print("Max(x): ");
    Serial.print(max_values[0], 6); // print max filtered mag value for x axis
    Serial.print(", Min(x): ");
    Serial.print(min_values[0], 6); // print min filtered mag value for x axis
    Serial.print(",  Max(y): ");
    Serial.print(max_values[1], 6);
    Serial.print(", Min(y): ");
    Serial.print(min_values[1], 6);
    Serial.print(",  Max(z): ");
    Serial.print(max_values[2], 6);
    Serial.print(", Min(z): ");
    Serial.println(min_values[2], 6); // remember to put an end of line on the last print statement
  }
  else if(loop_number == 9) // reset loop number
  {
    loop_number = -1;
  }
  ++loop_number; // increment loop number
  Loop_Timing();
}

void Loop_Timing()
{
  if (micros() > loop_time)   // check overflow of micros() has not happened
  {
    while (micros()-loop_time < loop_delay - 150 && micros() > loop_time) // delay if needed but keep checking in case of interupts. Ensure no overflow in timer
    {
      delayMicroseconds(100);
    }
    if (micros()-loop_time < loop_delay - 5 && micros() > loop_time) // delay for the last bit if needed. Ensure no overflow in timer
    {
      delayMicroseconds(loop_delay-(micros()-loop_time)-5); // stay slightly ahead of required speed
    }
  }
  loop_time = micros(); // save current time for next loop
}
