/*
Arduino Code for the calibration of the Gyroscope found on the GY-85 9DoF IMU
Written by Russell SA Brinkworth 2014
Based on code found in FreeIMU Arduino examples by Fabio Varesano, Filipe Vieira and TJS
ITG3200 (gyroscope)

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
#include <ITG3200.h> // gyro library

ITG3200 gyro = ITG3200();

float raw_data[3] = {0.0}; // gyro needs a vector to put raw data into
float Gyro_data[3] = {0.0}; // location of filtered gyro data
float Gyro_offset[3] = {0, 0, 0}; // nominal gyro offset values for all 3 axes to remove drift. Must be very close to real value to start with but will be updated by the adaptive filter
float Gyro_scale = 1.0; // gyroscope scale value. Used to onvert to rad
#define gyro_limit 0.0 // gyro movement limit before stop including value in drift correction calculation (adaptive filter)
#define TCgyro 100.0 // time constant for gyro offset calculation (x, y, z)
float yaw = 0.0; // angular displacement of z axis

#define loop_delay 40000 // loop delay in us. This is the desired time between loops.
unsigned long loop_time = micros();   // loop time place holder for loop delay calculation
int loop_number = 0;

void setup(void)
{  
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Initialising gyro...");
  gyro.reset();
  gyro.init(ITG3200_ADDR_AD0_LOW);
  Serial.println("Calibrating gyro...");
  gyro.zeroCalibrate(250,2); // initial drift correction on gyro. (Total number of samples, sample delay)
}

void loop()
{   
  gyro.readGyro(raw_data); // Get gyro data
  Drift_Removal(); // remove the drift (offset) from the gyro values
  yaw += Gyro_data[2]; // calculate angular displacement in the z axis
  if(loop_number == 0) // do not print every loop
  {
    Serial.print(Gyro_data[2], 6); // print the yaw velocity estimate with 6 decimal places
    Serial.print("    ");
    Serial.println(yaw, 6); // print the yaw estimate with 6 decimal places
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

void Drift_Removal()
{
  for (int i = 0; i <= 2; i++)
  {
    // drift calculation for gyro
    if (abs(raw_data[i]-Gyro_offset[i]) < gyro_limit) // only update adaptive filter if no movement detected in axis
    {
      Gyro_offset[i] = (1/TCgyro)*raw_data[i] + (1-1/TCgyro)*Gyro_offset[i]; // filter the gyro measurement to estimate offset for drift removal
    }
    Gyro_data[i] = loop_delay * (raw_data[i] - Gyro_offset[i]) / (1000 * Gyro_scale); // put gyro data into vector with drift removed. Scale data to rad
  }
}
