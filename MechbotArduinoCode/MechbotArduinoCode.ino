//////////////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2013 - 2015, University of South Australia
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//
//  Redistributions of source code must retain the above copyright notice, this list
//  of conditions and the following disclaimer.
//  Redistributions in binary form must reproduce the above copyright notice, this
//  list of conditions and the following disclaimer in the documentation and/or other
//  materials provided with the distribution.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
//  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
//  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
//  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
//  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
//  OF SUCH DAMAGE.
//
//////////////////////////////////////////////////////////////////////////////////////////
//
//  File            :  mechbot_v3_1_student.ino
//
//  Project         :  MechBot UGV Platform
//
//  Author(s)       :  Russell Brinkworth
//
//  Contributor(s)  :  Joseph East, Graeme Robertson, Darren Veenstra, Colin Smith,
//                     Daniel Griffiths & Phil Skelton
//
//  Maintainer(s)   :  Russell Brinkworth, Daniel Griffiths & Phil Skelton
//
//  Creation Date   :  24th January, 2013
//
//  Latest Update   :  19th February, 2015
//
//////////////////////////////////////////////////////////////////////////////////////////
//
//  Description: Arduino code to run the MechBot UGV platforms.
//
//  Note the following order of declaration:
//
//    1. Libraries.
//    2. Device hooks, ROS messages, global variables.
//    3. Function prototypes.
//    4. Publishers.
//    5. Subscriber callbacks.
//    6. Subscribers.
//    7. Arduino structures.
//    8. Your functions.
//    9. Helper functions.
//
//  Tested with LUbuntu 14.04 and ROS Indigo (Phil Skelton, December 2014).
//  Absolutely no guarantees are provided for compatibility with other versions.
//
////////////////////////////////////////////////////////////////////////////////////////////
//
//  Version history:
//
//    1.0.0 : Initial version.
//    1.1.0 : Increased analog inputs.
//    2.0.0 : Reordered code.
//            Optimised loops.
//            Added loop timers.
//            Staggered sensor updates.
//    2.1.0 : Changed ultrasonics from blocking call to interrupt to reduce loop time.
//    2.1.1 : Removed some orphaned function definitions.
//    2.1.2 : Replaced constants with defines.
//    2.2.0 : Integrated IMU into code.
//    2.3.0 : Reworked loop timing so code runs faster.
//            Control system on-off with variable.
//            Delay to ultrasonic first ping.
//            Gyro drift removal for z-axis.
//    2.3.1 : Resorted code order in main loop.
//            Battery capacity based on minimum cell voltage.
//            Encoder counts estimate lost ones due to missing interrupts and correct for loop time.
//            Ultrasonic power turned on during initialisation.
//    2.3.2 : Reordered code in main loop, now with 25Hz updates on most ROS communications.
//            Relabelled ultrasonics.
//    2.3.3 : Reduced speed of updates in main loop to be 10Hz communication through ROS rather than 25Hz.
//    2.3.4 : Removed function prototypes. REVOKED IN VERSION 3.1.0 (Phil Skelton)
//            Tidied up variable names.
//            Enacted 'Request IP' message over ROS as LCD_status.
//            Tidied up formatting.
//            Remove servo status topic.
//    2.3.5 : Filter for IR sensors and separated reading from publishing.
//    2.3.6 : Motor controller variables modified over ROS.
//    2.3.7 : Optional high speed encoder values.
//    3.0.0 : Rewrite for version 2 of the hardware.
//    3.1.0 : Updated code for version 3 of hardware (primary CPU upgrade).
//            Re-formatted, spell-checked, tidied up (to a limit) the entire code.
//            Changed all non-library variables, definitions and functions from 'camelCase' to 'underscores' formatting.
//            Forced ROS topics to use a constant prefix that should be unique to each robot (e.g. mechbot_01, bivbot_01).
//            Revoked a change implemented in version 2.3.4 and reimplemented function prototypes.
//            Cleared LCD incomming message when battery % written.
//    3.1.0s: Student version. Filters and control code removed.
//            Accurate loop timing removed.
//            Battery capacity model and display checks removed.
//            Check for multiple LCD messages removed.
//            Removed high speed encoder option.
//            Put all subroutines into a single main loop.
//
//////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////
// 1. Libraries.
//////////////////////////////////////////////////////////////////////////////////////////

#include <ros.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"

#include <ADXL345.h> // Accelerometer library.
#include <Arduino.h>
#include <HMC58X3.h> // Magnetometer library.
#include <ITG3200.h> // Gyroscope library.
#include <LiquidCrystal.h>
#include <PinChangeInt.h> // Interrupt library that allows extra pins to be used.
#include <Servo.h>
#include <stdlib.h>
#include <Wire.h>

#include "IRTemp.h"


//////////////////////////////////////////////////////////////////////////////////////////
// 2. Device hooks, ROS messages, global variables.
//////////////////////////////////////////////////////////////////////////////////////////

ros::NodeHandle node_handle; // Mandatory ROS initialiser.

// Students needs to change this constant to be what their MechBot is.
// For all intents and purposes, this should match both the physical label on the robot, and the VICON label.
// All ROS messages will use this prefix.
const String ROBOT_IDENTIFIER = "/mechbot_12"; // CHANGE THIS NAME TO MATCH THE SYSTEM NAME!

// Students should not change these values.
const String GETTER_IDENTIFIER = "/get";
const String SETTER_IDENTIFIER = "/set";

boolean got_connection = false; // Flag to say if got connection, as defined by a message sent to LCD for display over ROS.

// ------------------------ Motor Speed Controller Variables ------------------------

const String MOTOR_CONTROL_TOPIC_STRING = ROBOT_IDENTIFIER + SETTER_IDENTIFIER + "/motor_control";
const char * MOTOR_CONTROL_TOPIC = MOTOR_CONTROL_TOPIC_STRING.c_str(); // ROS topic for motor control.

int motor_speed_left = 0; // Motor speed reference.
int motor_speed_right = 0;
float control_data[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // Motor control values (on?, Proportional, Integral, Derivative, Encoders High Speed).
// Put in motor controller variables here.
int error_sum[2] = {0, 0};

#define PID_HARDCODED //if defined, PID values will be locked to the below values:
#ifdef PID_HARDCODED
float p = 2;
float i = 0.5;
float d = 0.1;
#endif


// ------------------------ Battery values specific section ------------------------

// Battery monitoring setup
#define BMS_ADDRESS 0x0DA7 // I2C address for battery management system.
#define TC_CAPACITY 20.0 // Time constant for battery capacity readings.

enum // Possible commands to send to battery management system.
{
  MIN_CELL_VOLTAGE = 1, // Request minimum cell voltage.
  CELL_NUMBER, // How many cells are connected in battery (3 is expected, 6 is maximum).
  BATTERY_VOLTAGE, // Total voltage from battery.
  CELL_VOLTAGES, // Voltage of each cell.
  SYS_CURRENT, // Current being used by system.
  OPP_TEMP, // Operating temperature.
  EX_VOLTAGE, // External voltage value.
  SHUTDOWN, // Tell system to shutdown.
};

const String BATTERY_STATUS_TOPIC_STRING = ROBOT_IDENTIFIER + GETTER_IDENTIFIER + "/battery_status";
const char * BATTERY_STATUS_TOPIC = BATTERY_STATUS_TOPIC_STRING.c_str();

unsigned char BMS_msg[32] = {0}; // Buffer for battery management system signals.
int cell_voltage = 0; // Variable holding the minimum cell voltage from the battery management system.
float battery_data[2]; // Current set up for 2 battery values (battery capacity, current draw), can be changed.
float battery_level = 0; // Last displayed battery level.
// Can put variables for battery capacity filter and/or display timing here.
float battery_max_mv = 4200
float battery_min_mv = 3600
std_msgs::Float32MultiArray battery_msg; // ROS message variable for battery and power data.


// ------------------------ Force sensor specific section ------------------------

#define FORCE_PIN A5

const String FORCE_STATUS_TOPIC_STRING = ROBOT_IDENTIFIER + GETTER_IDENTIFIER + "/force_status";
const char * FORCE_STATUS_TOPIC = FORCE_STATUS_TOPIC_STRING.c_str();

float force_data[1]; // Current set up for 1 force value, can be changed.

std_msgs::Float32MultiArray force_msg;


// ------------------------ IMU specific section ------------------------

#define GYRO_LIMIT 0.0 // Constant for gyro movement limit before stop including value in drift correction calculation (i.e. adaptive filter).
#define GYRO_TC 100.0 // Time constant for gyro offset calculation (x, y, z).

ADXL345 accel;
HMC58X3 magn;
ITG3200 gyro = ITG3200();

const String IMU_STATUS_TOPIC_STRING = ROBOT_IDENTIFIER + GETTER_IDENTIFIER + "/IMU_status";
const char * IMU_STATUS_TOPIC = IMU_STATUS_TOPIC_STRING.c_str();

const float mag_offset[3] = {0.0, 0.0, 0.0}; // Magnetometer offset calibration values. These will be different for every IMU.
const float mag_scale[3] = {1.0, 1.0, 1.0}; // Magnetometer scale calibration. These will be different for every IMU.

float IMU_data[9]; // Setup array for IMU data. 3 x acceleration, 3 x magnetometer and 3 x gyroscope.
float acc_data[3]; // Accelerometer needs a vector to put data into.
float acc_offset[3] = {0.0, 0.0, 0.0}; // Accelerometer offsets.
float acc_scale[3] = {1.0, 1.0, 1.0}; // Accelerometer gains.

float gyro_data[3]; // Gyro needs a vector to put data into.
float gyro_offset[3] = {0, 0, 0}; // Nominal gyro offset values for all 3 axes to remove drift. Must be very close to real value to start with but will be updated by the adaptive filter.
const float gyro_scale = 1; // Gyroscope scale value. Convert to radians / second.

std_msgs::Float32MultiArray IMU_msg; // ROS message variable for IMU data.


// ------------------------ InfraRed sensor specific section ------------------------

// Put time constant for IR LPF here.

const String IR_STATUS_TOPIC_STRING = ROBOT_IDENTIFIER + GETTER_IDENTIFIER + "/IR_status";
const char * IR_STATUS_TOPIC = IR_STATUS_TOPIC_STRING.c_str();

const unsigned int IR_SENSOR_PINS[4] = {A1, A2, A3, A4}; // Set the signal pins for the IRs (rear, right, left, front).
const unsigned int IR_ENABLE_PINS[4] = {53, 49, 48, 47}; // Set the enable pins for the IRs.

float IR_data[4]; // Current set up for 4 IR sensors, can be changed.
// Put variable for previous values for IR LPF here.

std_msgs::Float32MultiArray IR_msg;


// ------------------------ LCD specific section ------------------------

//#define LCD_RW 39 // R/W pin for LCD
#define LCD_NUM_CHARS 16 // Number of characters in a line on the LCD.

LiquidCrystal charLCD(40, 39, 38, 33, 32, 31, 30); // Setup for the default, set your own pins if necessary (RS, RW, EN, D4, D5, D6, D7).

const String LCD_WRITE_TOPIC_STRING = ROBOT_IDENTIFIER + SETTER_IDENTIFIER + "/LCD_write";
const char * LCD_WRITE_TOPIC = LCD_WRITE_TOPIC_STRING.c_str();
const String LCD_STATUS_TOPIC_STRING = ROBOT_IDENTIFIER + GETTER_IDENTIFIER + "/LCD_status";
const char * LCD_STATUS_TOPIC = LCD_STATUS_TOPIC_STRING.c_str();

boolean LCD_new_message = true;
boolean refresh_screen = false; // Boolean to make LCD writes go on alternate lines.
char LCD_incoming_msg[LCD_NUM_CHARS + 1]; // This is where the LCD message is stored once it has been received from ROS. Need one extra character for the terminating null character.

std_msgs::String LCD_msg; // Holds string last printed to the LCD.


// ------------------------ IP address specific section ------------------------

const String IP_ADDRESS_TOPIC_STRING = ROBOT_IDENTIFIER + GETTER_IDENTIFIER + "/IP_address";
const char * IP_ADDRESS_TOPIC = IP_ADDRESS_TOPIC_STRING.c_str();

char IP_address_msg[LCD_NUM_CHARS + 1];

boolean new_IP_address = true;


// ------------------------ Motor specific section ------------------------

#define MOTOR_ADDRESS 0x0DB7 // I2C address for motor controller.
#define MOTOR_MAX 4000 // Maximum value to send to motors.

enum // Possible commands to send to motor controller.
{
  PWM_left = 1, // Speed of left motor (0 to 4000).
  PWM_right, // Pulse width to right motor (0 to 4000).
  enc_left_and_right, // Encoder values of left and right motor (16-bit left, 16-bit-right).
  motor_temp, // Current operating temperature.
  motor_current, // System current usage (* 5 / 1024 to convert to Amp).
  flag_motor_direction, // Motor direction (0 = forward, 1 = reverse, 2 = clock-wise, 3 = counter-clock-wise).
  enc_time, // Time since encoders last read (* 32 to convert to us).
  enc_overflow, // Did encoder timing overflow (0 = no, 255 = yes).
  motor_enable // Enable motors (0 = no, 255 = yes).
};

const String MOTOR_DRIVE_TOPIC_STRING = ROBOT_IDENTIFIER + SETTER_IDENTIFIER + "/motor_drive"; // ROS topic for motor set points.
const char * MOTOR_DRIVE_TOPIC = MOTOR_DRIVE_TOPIC_STRING.c_str();
const String MOTOR_STATUS_TOPIC_STRING = ROBOT_IDENTIFIER + GETTER_IDENTIFIER + "/motor_status"; // ROS topic for PWM value sent to motor (maximum 4000).
const char * MOTOR_STATUS_TOPIC = MOTOR_STATUS_TOPIC_STRING.c_str();

uint8_t motor_direction = 0; // Direction of MechBot (0 = forward, 1 = reverse, 2 = clock-wise, 3 = counter clock-wise).
uint8_t motor_left = 0; // Desired speed of left wheel (maximum 255).
uint8_t motor_right = 0; // Desired speed of right wheel.
int32_t motor_msg[3]; // Holder for motor control message.

std_msgs::Int32MultiArray ROS_motor_msg; // ROS topic for motors.


// ------------------------ Loop timing values ------------------------

#define LOOP_DELAY 20000 // Loop delay in us. This is the desired time between drive calls. Stability ensures accurate velocity calculations and motor control.

// Can put variables for checking accurate loop timing here, eg interupt time and scale factor.
int loop_number = 0; // variable to hold the loop number. Used to keep track of which topics to publish and when
unsigned long loop_time = micros(); // Loop time place holder for motor velocity calculation.
unsigned long last_loop_micros = micros();

// ------------------------ Encoder specific section ------------------------

const String ENCODER_STATUS_TOPIC_STRING = ROBOT_IDENTIFIER + GETTER_IDENTIFIER + "/encoder_status";
const char * ENCODER_STATUS_TOPIC = ENCODER_STATUS_TOPIC_STRING.c_str();

unsigned char read_encoders[16] = {0}; // Buffer for encoder values from motor controller.
long encoder_data[4]; // Holds encoder information (left velocity, right velocity, left displacement, right displacement).

std_msgs::Int32MultiArray encoder_msg; // Encoder message on ROS.


// ------------------------ Timing specific section ------------------------

const String TIME_STATUS_TOPIC_STRING = ROBOT_IDENTIFIER + GETTER_IDENTIFIER + "/time_status";
const char * TIME_STATUS_TOPIC = TIME_STATUS_TOPIC_STRING.c_str();

int32_t time_data[1]; // Current set up for 1 time value, can be changed.
unsigned long previous_time = 0; // Value that holds the last time time update was called for publication of loop period (ms).
unsigned long tic = micros(); // Temporary timing variable.

std_msgs::Int32MultiArray time_msg; // ROS message variable for timing data.


// ------------------------ Servo specific section ------------------------

// Servo starting values.
// Warning! If the servo is not positioned correctly then it will push into the chassis or
// try to extend the servo beyond its limits and cause problems, possibly damage.
#define ARM_PIN 7 // Define pin for arm.
#define ARM_START 2300 // Arm down.
#define GRIP_PIN 6 // Define pin for gripper.
#define GRIP_START 1000 // Gripper wide open.

Servo arm_servo, grip_servo; // We only need 2 servos for now, arm and gripper.

const String SERVO_DRIVE_TOPIC_STRING = ROBOT_IDENTIFIER + SETTER_IDENTIFIER + "/servo_drive";
const char * SERVO_DRIVE_TOPIC = SERVO_DRIVE_TOPIC_STRING.c_str();

int arm_PWM = ARM_START; // Variable to hold servo value.
int grip_PWM = GRIP_START;

std_msgs::Int32MultiArray servo_msg; // ROS message that holds the status of each of the servos.


// ------------------------ Ultrasonic sensor specific section ------------------------

#define US_PING_PIN_0 26
#define US_PING_PIN_1 27
#define US_PWR_PIN_0 22
#define US_PWR_PIN_1 23
#define US_RETURN_PIN_0 A9
#define US_RETURN_PIN_1 A10
#define US_WAIT 2000 // Start up delay on ultrasonics, do not ping them until finished initialising (milliseconds).

const String ULTRASONIC_STATUS_TOPIC_STRING = ROBOT_IDENTIFIER + GETTER_IDENTIFIER + "/US_status";
const char * ULTRASONIC_STATUS_TOPIC = ULTRASONIC_STATUS_TOPIC_STRING.c_str();

boolean US_on = false; // Flag to state if ultrasonics have started yet.
boolean US_refresh = false; // Flag to say if new ultrasonic data available.
float US_data[2]; // Current set up for 2 ultrasonic sensors, can be changed.
unsigned long US_start_time[2] = {micros(), micros()}; // Variable for start point of ultrasonic pulses.

std_msgs::Float32MultiArray US_msg; // ROS message variable for US data.


// ------------------------ IR Temperature specific section ------------------------

//#define IR_TEMP_ENABLED

#ifdef IR_TEMP_ENABLED
static const byte PIN_DATA    = 2; // Choose any pins you like for these
static const byte PIN_CLOCK   = 3;
static const byte PIN_ACQUIRE = 4;

static const TempUnit SCALE=CELSIUS;  // Options are CELSIUS, FAHRENHEIT

IRTemp irTemp(PIN_ACQUIRE, PIN_CLOCK, PIN_DATA);

float IR_T_data[2];  //ambient temperature, IR temperature

const String IR_TEMP_STATUS_TOPIC_STRING = ROBOT_IDENTIFIER + GETTER_IDENTIFIER + "/IR_TEMP_status";
const char * IR_TEMP_STATUS_TOPIC = IR_TEMP_STATUS_TOPIC_STRING.c_str();

std_msgs::Float32MultiArray IR_T_msg; // ROS message variable for IR Temp data.

#endif

//////////////////////////////////////////////////////////////////////////////////////////
// 3. Function prototypes.
// Function prototypes need to go here, as per normal C/C++ definitions.
//////////////////////////////////////////////////////////////////////////////////////////

// Useful functions that you may need to edit.
void battery_message();
void loop_timing();
void drive();
void encoder_message();
void force_message();
void IMU_message();
void IR_get();
void IR_message();
void time_message();
void US_message();

// Helper functions that you should probably leave alone.
void battery_display();
void clear_screen();
void get_encoders();
int get_bytes_from_n(unsigned char *data, unsigned char n);
void motor_publish();
void print_bottom_LCD_line(String message);
void print_top_LCD_line(String message);
void send_command(int target_address, unsigned char cmd_ID, int cmd_data);
void send_request(int target_address, unsigned char *data, unsigned int n);
void set_motors(unsigned char motor_direction, int left, int right);
void US_interrupt_0();
void US_interrupt_1();


//////////////////////////////////////////////////////////////////////////////////////////
// 4. Publishers.
// Publishers data generated on the Arduino over the ROS network.
//////////////////////////////////////////////////////////////////////////////////////////

ros::Publisher battery_publisher(BATTERY_STATUS_TOPIC, &battery_msg);
ros::Publisher encoder_publisher(ENCODER_STATUS_TOPIC, &encoder_msg);
ros::Publisher force_publisher(FORCE_STATUS_TOPIC, &force_msg);
ros::Publisher IMU_publisher(IMU_STATUS_TOPIC, &IMU_msg);
ros::Publisher IR_publisher(IR_STATUS_TOPIC, &IR_msg);
ros::Publisher LCD_publisher(LCD_STATUS_TOPIC, &LCD_msg);
ros::Publisher motor_publisher(MOTOR_STATUS_TOPIC, &ROS_motor_msg);
ros::Publisher time_publisher(TIME_STATUS_TOPIC, &time_msg);
ros::Publisher US_publisher(ULTRASONIC_STATUS_TOPIC, &US_msg);
#ifdef IR_TEMP_ENABLED
ros::Publisher IR_T_publisher(IR_TEMP_STATUS_TOPIC, &IR_T_msg);
#endif

//////////////////////////////////////////////////////////////////////////////////////////
// 5. Subscriber callbacks.
// These will only run if the associated data on the ROS network changes.
//////////////////////////////////////////////////////////////////////////////////////////

void control_callback(const std_msgs::Float32MultiArray& cmd_msg) // Run if changes to the motor control variables requested.
{
  for (int i = 0; i < 5; i++)
  {
    control_data[i] = cmd_msg.data[i]; // Put newly requested control variables into array for use in drive function.
  }
}


void IP_callback(const std_msgs::String& cmd_msg)
{
  for (int i = 0; i < LCD_NUM_CHARS; i++) // Copy the uint8_t array into a char array but only for size of LCD line.
  {
    IP_address_msg[i] = cmd_msg.data[i];
  }

  // If there is no difference between the incoming message and the magic string.
  if (strcmp(IP_address_msg, "NO CONNECTION!") == 0)
  {
    // The system has responded that there aren't any available network connections, set the flag false.
    got_connection = false;
  }
  else
  {
    // The system has found an IP address, so set the flag to be true.
    got_connection = true;
  }

  print_top_LCD_line(IP_address_msg);
}


void LCD_callback(const std_msgs::String& cmd_msg) // Subroutine run when LCD display message received.
{
  // Can put check for new LCD message here.
  for (int i = 0; i < LCD_NUM_CHARS; i++) // Copy the uint8_t array into a char array but only for size of LCD line.
  {
    LCD_incoming_msg[i] = cmd_msg.data[i];
  }
  print_bottom_LCD_line(LCD_incoming_msg); // Print message to LCD.

  LCD_msg.data = LCD_incoming_msg; // Allocate message to publisher node.
  LCD_publisher.publish(&LCD_msg); // Publish the received command to confirm current display.

  if (strcmp(LCD_incoming_msg, "SHUTTING DOWN...") == 0) // Check to see if a shutdown request is being processed by the system.
  {
    print_top_LCD_line("POWERING OFF IN");
    print_bottom_LCD_line("15 SECONDS");

    send_command(BMS_ADDRESS, SHUTDOWN, 0); // Request shutdown of battery management system.
  }
}


void motor_callback(const std_msgs::UInt8MultiArray& cmd_msg) // Get new motor command values.
{
  if (cmd_msg.data_length <= 4 && cmd_msg.data_length > 0)
  {
    motor_direction = cmd_msg.data[0]; // First element is direction.

    if (cmd_msg.data[3] == 0) // Fourth element defines if the left and right speeds both equal the second element.
    {
      motor_left = cmd_msg.data[1];
      motor_right = cmd_msg.data[1];
    }
    else // Or independent left and right speeds from second and third elements.
    {
      motor_left = cmd_msg.data[1];
      motor_right = cmd_msg.data[2];
    }
  }
}


void servo_callback(const std_msgs::Int32MultiArray& cmd_msg) // Get new servo values.
{
  if (cmd_msg.data_length <= 2 && cmd_msg.data_length > 0) // Sanity check the input.
  {
    grip_PWM = cmd_msg.data[0];
    arm_PWM = cmd_msg.data[1];
    grip_servo.writeMicroseconds(grip_PWM); // Write updated value to Servo.
    arm_servo.writeMicroseconds(arm_PWM); // Pro-tip: sending an invalid ID will trigger a status return regardless.
  }
}


//////////////////////////////////////////////////////////////////////////////////////////
// 6. Subscribers.
// Subscribes to variables on the ROS network and only interrupts when a message arrives.
//////////////////////////////////////////////////////////////////////////////////////////

ros::Subscriber<std_msgs::Float32MultiArray> control_subscriber(MOTOR_CONTROL_TOPIC, control_callback);
ros::Subscriber<std_msgs::String> IP_subscriber(IP_ADDRESS_TOPIC, IP_callback);
ros::Subscriber<std_msgs::String> LCD_subscriber(LCD_WRITE_TOPIC, LCD_callback);
ros::Subscriber<std_msgs::UInt8MultiArray> motor_subscriber(MOTOR_DRIVE_TOPIC, motor_callback);
ros::Subscriber<std_msgs::Int32MultiArray> servo_subscriber(SERVO_DRIVE_TOPIC, servo_callback);


//////////////////////////////////////////////////////////////////////////////////////////
// 7. Arduino structures.
//////////////////////////////////////////////////////////////////////////////////////////

void setup() // This subroutine runs once at start-up and initialises the system.
{
  analogReference(EXTERNAL); // Uses the more accurate external analog reference voltage.
  delay(100); // Delay to ensure analog reference value is properly set.

    // LCD hardware initialisation.
  charLCD.begin(16, 2);

  print_top_LCD_line("STARTING SYSTEM");

  print_bottom_LCD_line("I2C BUS");

  Wire.begin(); // Join I2C bus

  print_bottom_LCD_line("SERVOS");

  grip_servo.attach(GRIP_PIN);
  arm_servo.attach(ARM_PIN);
  arm_servo.writeMicroseconds(1000);// Move arm to get attention.

  print_bottom_LCD_line("IR SENSORS");

  for (int i = 0; i < 4; i++) // Setup IRs.
  {
    pinMode(IR_SENSOR_PINS[i], INPUT); // Define IR input pins.
    pinMode(IR_ENABLE_PINS[i], OUTPUT); // Set IR enable pins.
    digitalWrite(IR_ENABLE_PINS[i], HIGH); // Enable IRs.
  }

  print_bottom_LCD_line("US SENSORS");

  // Ultrasonic hardware initialisation, setting up output (ping) and input (return) pins.
  // Ultrasonic Left
  pinMode(US_PING_PIN_0, OUTPUT);
  digitalWrite(US_PING_PIN_0, LOW);
  pinMode(US_RETURN_PIN_0, INPUT);
  PCintPort::attachInterrupt(US_RETURN_PIN_0, US_interrupt_0, CHANGE); // Attach interrupt function to ultrasonic pin.

  // Ultrasonic Right
  pinMode(US_PING_PIN_1, OUTPUT);
  digitalWrite(US_PING_PIN_1, LOW);
  pinMode(US_RETURN_PIN_1, INPUT);
  PCintPort::attachInterrupt(US_RETURN_PIN_1, US_interrupt_1, CHANGE); // Attach interrupt function to ultrasonic pin.

  pinMode(US_PWR_PIN_0, OUTPUT);
  pinMode(US_PWR_PIN_1, OUTPUT);
  digitalWrite(US_PWR_PIN_0, LOW); // Turn ultrasonics off so they can have a fast transient to start.
  digitalWrite(US_PWR_PIN_1, LOW);

  delay(50);

  // Set up IMU.
  print_bottom_LCD_line("CALIBRATING IMU"); // Write to LCD describing operation.
  accel.init(ADXL345_ADDR_ALT_LOW); // Initialise accelerometer.
  accel.set_bw(ADXL345_BW_12); // Set byte width.
  magn.init(true); // No delay needed as we have already a delay(5) in HMC58X3::init().
  magn.calibrate(1, 10); // Calibrate HMC using self test, not recommended to change the gain after calibration. (Gain, number of sample). Use gain 1 = default, valid (0 - 7), 7 not recommended.
  magn.setMode(0); // Single mode conversion was used in calibration, now set continuous mode.
  gyro.reset();
  gyro.init(ITG3200_ADDR_AD0_LOW);
  gyro.zeroCalibrate(250, 2); // Initial drift correction on gyro. (Total number of samples, sample delay).

  print_bottom_LCD_line("CONNECTING ROS"); // Write to LCD describing operation.

  Serial.begin(115200); // Start communications with main system.

  // Initialise this node on the ROS network.
  node_handle.initNode();

  // Subscribe to the required ROS topics.
  node_handle.subscribe(control_subscriber);
  node_handle.subscribe(IP_subscriber);
  node_handle.subscribe(LCD_subscriber);
  node_handle.subscribe(motor_subscriber);
  node_handle.subscribe(servo_subscriber);

  // Spawn publishers for the required topics.
  node_handle.advertise(battery_publisher);
  node_handle.advertise(encoder_publisher);
  node_handle.advertise(force_publisher);
  node_handle.advertise(IMU_publisher);
  node_handle.advertise(IR_publisher);
  node_handle.advertise(LCD_publisher);
  node_handle.advertise(motor_publisher);
  node_handle.advertise(time_publisher);
  node_handle.advertise(US_publisher);
  
  #ifdef IR_TEMP_ENABLED
  node_handle.advertise(IR_T_publisher);
  #endif

  print_bottom_LCD_line("RESETTING US"); // Write to LCD describing operation.

  delay(100); // Hold to ensure system can give a fast starting transient for US.
  digitalWrite(US_PWR_PIN_0, HIGH); // Turn ultrasonics on.
  digitalWrite(US_PWR_PIN_1, HIGH);

  print_bottom_LCD_line("SYSTEM STARTED!"); // Write to LCD describing operation.

  // Sets the servos at start-up depending on defined constant values.
  // If the servo is not positioned correctly then it will push into the chassis or try to extend the servo beyond its limits and cause problems, possibly damage.
  grip_servo.writeMicroseconds(GRIP_START);
  arm_servo.writeMicroseconds(ARM_START); // Movement of arm will let user know system has started.

  // Request IP address from main system.
  LCD_msg.data = "REQUEST IP"; // Ask system for IP address to be displayed on LCD.
  print_top_LCD_line(LCD_msg.data); // Print to LCD to let user know request is being sent.
  LCD_publisher.publish(&LCD_msg); // Publish the IP request over ROS as LCD message.

  // Get initial battery level.
  battery_message();
  battery_display(); // Print battery capacity on LCD
}


// Must call drive (routine that controls the speed of motors) at reliable and known times or calculations for control system will be wrong.
void loop() // Main system loop. Everything runs from here.
{
  node_handle.spinOnce(); // Start-up code. Only do on first run. Starts ROS communication callbacks.

  if (!got_connection) // Keep requesting IP if no response received on ROS message LCD command.
  {
    LCD_msg.data = "REQUEST IP"; // Ask system for IP address to be displayed on LCD.
    LCD_publisher.publish(&LCD_msg); // Publish the IP request over ROS as LCD message. No need to keep displaying it on the LCD.
  }

  // Functions that need to be published every loop go here.
  IR_get(); // Get and filter IR values.
  loop_timing(); // Ensures accurate loop timing between calls to this function.
  drive(); // Determine values to send to the main drive motors.
  time_message(); // Publish Arduino code timing value(s). Is it needed every loop?
  
  // Functions that do not need to be published every loop go in one of these cases.
  if (loop_number == 0)
  {
    IMU_message(); // Get IMU values and publish on ROS.
    IR_temp_get();
    IR_temp_publish();
  }
  else if (loop_number == 1)
  {
    encoder_message(); // Only publish here if need low speed updates. If high speed needed then put outside of case statement.
    motor_publish(); // Only publish motor values to ROS here. 
  }
  else if (loop_number == 2)
  {
    battery_message(); // Get and publish battery / power values to ROS.
    //battery_display(); // How often do you want to print this to the LCD?
    force_message(); // Get analog value from force sensor and publish on ROS.
    IR_message(); // Only publish IR values to ROS here.
  }
  else if (loop_number == 3)
  {
    if (millis() > US_WAIT || US_on) // Only run ultrasonics after a start up delay. Would turn ultrasonics off when time function resets every 70 minutes so also use flag to say when ultrasonics have started.
    {
      US_on = true; // Flag ultrasonics as having started so they are not paused if timing value resets.
      US_message(); // Publish US data, if there is any new data.
    }
  }
  
  if (loop_number == 4) // See if system is on the last loop.
  {
    loop_number = 0; // Reset the loop counter.
  }
  else
  {
    loop_number++; // Increment the loop counter.
  }
}


//////////////////////////////////////////////////////////////////////////////////////////
// 8. Your functions.
//////////////////////////////////////////////////////////////////////////////////////////

void battery_message() // Need to convert raw values into useful units.
{
  // Find battery capacity remaining.
  send_command(BMS_ADDRESS, MIN_CELL_VOLTAGE, 0); // Request minimum cell voltage.
  delay(1); // Wait for battery management system to respond.
  send_request(BMS_ADDRESS, BMS_msg, 2); // Get minimum cell voltage from the battery management system.
  battery_data[0] = get_bytes_from_n(BMS_msg, 1); // Get signal from battery management system and put in battery data for transmission over ROS.

  // Put model for converting PWM signal from battery management system into battery capacity here.
  battery_data[0] = map(battery_data[0],battery_min_mv,battery_max_mv,0,100);
  // Find current usage
  send_command(BMS_ADDRESS, SYS_CURRENT, 0); // Request current usage.
  delay(1); // Wait for battery management system to respond.
  send_request(BMS_ADDRESS, BMS_msg, 2); // Get current usage from the battery management system.
  battery_data[1] = get_bytes_from_n(BMS_msg, 1); // Convert for transmission. Need to convert to mA.

  //battery_data[0] = round(battery_data[0]); // Round for transmission over ROS. This is needed if formula and filtering are used to convert the values.
  //battery_data[1] = round(battery_data[1]); // Round for transmission over ROS.
  battery_msg.data_length = 2;
  battery_msg.data = battery_data;
  battery_publisher.publish(&battery_msg); // Publish battery values on ROS network.
}

void loop_timing()
{
  while(micros() - last_loop_micros < LOOP_DELAY - 10)
  {
    delayMicroseconds(20); // This loop timing is inaccurate and does not take processing time into account. A better timing function is required.
  }
  last_loop_micros = micros();
}

void drive()
{
  get_encoders(); // Get the encoder values from the motor controller.
  // May need to scale velocity of wheels by actual time taken.
  
  #ifdef PID_HARDCODED
  control_data[0]=1;
  #endif
  
  if (control_data[0] == 1) // Left motor.
  {
    // Left motor speed control goes here.
    // Without code here the motor will not turn if the controller is turned on.
    motor_speed_left = pid_motor_control(0);
    /*
    int error = motor_left * 2 - abs(encoder_data[0]);
    motor_speed_left = 16 * (error * 1);
    */
  }
  else
  {
    motor_speed_left = 16 * motor_left; // No control system code. Just pass value out to motors with gain to scale up output from 8- to 12-bits.
  }

  if (motor_speed_left < 0 || motor_left <= 0) // These conditions set the speed within allowable range.
  {
    motor_speed_left = 0;
    error_sum[0] = 0;
    // TIP: reset intergral error to 0 here in order to reduce intergrator windup.
  }
  else if (motor_speed_left > 4000 || motor_left >= 255) // Make sure not to exceed allowable maximum speed.
  {
    motor_speed_left = 4000;
  }
  else
  {
    // Let the value pass.
  }

  if (control_data[0] == 1) // Right motor.
  {
    motor_speed_right = pid_motor_control(1);
    // Right motor speed control goes here.
    // Without code here the motor will not turn if the controller is turned on.
  }
  else
  {
    motor_speed_right = 16 * motor_right; // No control system code. Just pass value out to motors with gain to scale up output from 8-bits to 12-bits
  }

  if (motor_speed_right < 0 || motor_right <= 0) // These conditions set the speed within allowable range.
  {
    motor_speed_right = 0;
    error_sum[1] = 0;
    // TIP: reset intergral error to 0 here in order to reduce intergrator windup.
  }
  else if (motor_speed_right > 4000 || motor_right >= 255) // Make sure not to exceed allowable maximum speed.
  {
    motor_speed_right = 4000;
  }
  else
  {
    // Let the value pass.
  }

  // Send updated motor speed values to motors, keeping existing direction.
  set_motors(motor_direction, motor_speed_left, motor_speed_right); // Need to cast motorSpeed as controller may force them to be floats.
}

int pid_motor_control(int motor_num) {
  int motor_output, error;
  float kp, ki, kd;
  uint8_t motor_setpoint;
  
  
  kp = control_data[1];  //proportional multiplier
  ki = control_data[2];  //integral multiplier
  kd = control_data[3];  //derivative multiplier
  
  //if we've hardcoded values in, use those instead
  #ifdef PID_HARDCODED
  kp = p;
  ki = i;
  kd = d;
  #endif
  
  //first, get the setpoint for the desired motor
  if(motor_num == 0) {
    motor_setpoint = motor_left;
  } else {
    motor_setpoint = motor_right;
  }
  
  //now, calculate the error between the setpoint and the current speed
  error = motor_setpoint - abs(encoder_data[motor_num]);
  
  error_sum[motor_num] += error;
  
  //next, calculate the desired speed for the motor
  motor_output = 16 * ((error * kp) + (error_sum[motor_num] * ki));
  
  return motor_output;
  
}


void encoder_message() // Publish encoder data on ROS. Must run after drive.
{
  encoder_msg.data_length = 4;
  encoder_msg.data = encoder_data;
  encoder_publisher.publish(&encoder_msg);
}


void force_message() // Need to convert raw values into useful units.
{
  force_data[0] = analogRead(FORCE_PIN); // Gripper force sensor. Experiment to convert to useful units.
  force_msg.data_length = 1;
  force_msg.data = force_data;
  force_publisher.publish(&force_msg); // Publish force values on ROS network.
}


void IMU_message() // Send IMU values over network.
{
  accel.get_Gxyz(acc_data); // Get accelerometer data.
  // Calculate normalisation factor.
  for (int i = 0; i <= 2; i++)
  {
    IMU_data[i] = acc_data[i] / acc_scale[i]; // Normalise accelerometer data and put into IMU vector.
  }

  magn.getValues(&IMU_data[3], &IMU_data[4], &IMU_data[5]); // Get magnetometer values.
  for (int i = 0; i <= 2; i++)
  {
    IMU_data[i + 3] = (IMU_data[i + 3] - mag_offset[i]) / mag_scale[i]; // Remove magnetometer offsets and apply scale.
  }

  gyro.readGyro(gyro_data); // Get gyro data.
  for (int i = 0; i <= 2; i++)
  {
    // Put drift calculation for gyro here.
    IMU_data[i + 6] = LOOP_DELAY * (gyro_data[i] - gyro_offset[i]) / (1000 * gyro_scale); // Put gyro data into IMU vector with drift removed. Scale data to rad.
  }

  IMU_msg.data_length = 9;
  IMU_msg.data = IMU_data;
  IMU_publisher.publish(&IMU_msg); // Publish IMU values on ROS network.
}


void IR_get() // Acquire and filter IR data.
{
  for (int i = 0; i < 4; i++)
  {
    IR_data[i] = analogRead(IR_SENSOR_PINS[i]); // Read IR values.
    // Apply LPF here.
  }
}


void IR_message() // Publish IR values on ROS network.
{
  IR_msg.data_length = 4;
  IR_msg.data = IR_data;

  for (int i = 0; i < IR_msg.data_length; i++)
  {
    IR_msg.data[i] = round(IR_data[i]);
  }

  IR_publisher.publish(&IR_msg);
}


void time_message() // Send timing values over network.
{
  time_data[0] = micros() - previous_time; // Find time since last run.
  previous_time = micros(); // Save current time as previous for next loop.
  time_msg.data_length = 1;
  time_msg.data = time_data;
  time_publisher.publish(&time_msg); // Publish time values on ROS network.
}


void US_message()
{
  if (digitalRead(US_RETURN_PIN_0) == LOW) // Ensure there is sufficient time between each ultrasonic ping so no interaction, only ping when not waiting for a response.
  {
    //Left ultrasonic
    digitalWrite(US_PING_PIN_0, HIGH);
    delayMicroseconds(21); // Width of ping pulse.
    digitalWrite(US_PING_PIN_0, LOW);
  }

  if (digitalRead(US_RETURN_PIN_1) == LOW)
  {
    //Right ultrasonic
    digitalWrite(US_PING_PIN_1, HIGH);
    delayMicroseconds(21); // Width of ping pulse.
    digitalWrite(US_PING_PIN_1, LOW);
  }

  if (US_refresh) // Check to see if new ultrasonic data present.
  {
    US_msg.data_length = 2; // Only publish when all data present.
    US_msg.data = US_data;
    US_publisher.publish(&US_msg); // Publish values on ROS network.
    US_refresh = false;
  }
}

void IR_temp_get() {
  #ifdef IR_TEMP_ENABLED
  IR_T_data[0] = irTemp.getAmbientTemperature(SCALE);
  IR_T_data[1] = irTemp.getIRTemperature(SCALE);
  #endif
}

void IR_temp_publish() {
  #ifdef IR_TEMP_ENABLED
  IR_T_msg.data_length = 2;
  IR_T_msg.data = IR_T_data;
    
  IR_T_publisher.publish(&IR_T_msg); // Publish IR_T values on ROS network.
  #endif
}

//////////////////////////////////////////////////////////////////////////////////////////
// 9. Helper functions.
// You probably shouldn't play with these, unless you actively want to break something.
//////////////////////////////////////////////////////////////////////////////////////////

void battery_display()
{
  String string_one = "BATTERY: ";
  String string_two = String(int(battery_data[0]), DEC);
  String string_three = "%";
  print_bottom_LCD_line(string_one + string_two + string_three);
  LCD_incoming_msg[0] = (char)0; // reset LCD message so no user messages will be ignored
}


void clear_screen()
{
  // This routine ensures that new sentences erase previous ones.
  // New sentences alternate lines on screen.
  if (!refresh_screen)
  {
    charLCD.setCursor(0, 1);
    charLCD.print("                ");
    charLCD.setCursor(0, 1);
  }
  else
  {
    charLCD.setCursor(0, 0);
    charLCD.print("                ");
    charLCD.setCursor(0, 0);
  }

  refresh_screen = !refresh_screen;
}


void get_encoders()
{
  send_command(MOTOR_ADDRESS, enc_left_and_right, 0); // Request encoder values.
  delay(1); // Wait for motor controller to respond.
  send_request(MOTOR_ADDRESS,read_encoders, 4); // Get encoder values (4 bytes or 2 x 16-bit) from the motor controller.
  encoder_data[0] = get_bytes_from_n(read_encoders, 1); // Get first 16-bit number from encoder values.
  encoder_data[1] = -get_bytes_from_n(read_encoders, 2); // Get second 16-bit number from encoder values. Right motor is mounted backwards so need to reverse encoder counts.
  encoder_data[2] = encoder_data[2] + encoder_data[0]; // Update total distance travelled.
  encoder_data[3] = encoder_data[3] + encoder_data[1];
}


int get_bytes_from_n(unsigned char *data, unsigned char n) // Gets 16 bits from data starting at position n.
{
  int result = 0;

  result = data[(n - 1) * 2]; // Get MSB.
  result = result << 8; // Logical shift to make room for LSB.
  result |= data[2 * n - 1]; // Put LSB into result.

  return result;
}


void print_bottom_LCD_line(String message)
{
  charLCD.setCursor(0, 1);
  charLCD.print("                ");
  charLCD.setCursor(0, 1);
  charLCD.print(message);
}


void print_top_LCD_line(String message)
{
  charLCD.setCursor(0, 0);
  charLCD.print("                ");
  charLCD.setCursor(0, 0);
  charLCD.print(message);
}


void motor_publish() // Publish commands sent to the motor controller via ROS.
{
  motor_msg[0] = motor_direction;
  motor_msg[1] = motor_speed_left;
  motor_msg[2] = motor_speed_right;
  ROS_motor_msg.data_length = 3;
  ROS_motor_msg.data = motor_msg;
  motor_publisher.publish(&ROS_motor_msg);
}


void send_command(int target_address, unsigned char cmd_ID, int cmd_data) // Send command to I2C slave.
{
  unsigned char cmd[3]; // Convert command data into 8-bit array.
  cmd[0] = cmd_ID;
  cmd[1] = cmd_data >> 8;
  cmd[2] = cmd_data;

  unsigned char i;
  Wire.beginTransmission(target_address);

  for (i = 0; i < 3; i++)
  {
    Wire.write(cmd[i]);
  }

  Wire.endTransmission();
}


void send_request(int target_address, unsigned char *data, unsigned int n) // Requests data of length n bytes from I2C device.
{
  Wire.requestFrom(target_address, n); // Send request.

  while (Wire.available() < 0)
  {
    // Block until the transmission is received.
  }

  for (int i = 0; i < n; i++)
  {
    data[i] = Wire.read(); // Get the data one byte at a time and put into read array.
  }
}


void set_motors(unsigned char motor_direction, int left, int right)
{
  send_command(MOTOR_ADDRESS, motor_enable, 0); // Disable motors so changes are not enacted.
  send_command(MOTOR_ADDRESS, flag_motor_direction, motor_direction);
  send_command(MOTOR_ADDRESS, PWM_left, left);
  send_command(MOTOR_ADDRESS, PWM_right, right);
  send_command(MOTOR_ADDRESS, motor_enable, 255); // Enable motors again so changes take affect at the same time.
}


void US_interrupt_0()
{
  // Determine if rising or falling.
  if (digitalRead(US_RETURN_PIN_0) == HIGH)
  {
    US_start_time[0] = micros(); // Begin counting if rising.
  }
  else // Stop counting.
  {
    US_data[0] = micros() - US_start_time[0]; // Find difference between rising and falling edge of pulse.
    US_refresh = true; // Flag new ultrasonic data available.
  }
}


void US_interrupt_1()
{
  // Determine if rising or falling.
  if (digitalRead(US_RETURN_PIN_1) == HIGH)
  {
    US_start_time[1] = micros(); // Begin counting if rising.
  }
  else // Stop counting.
  {
    US_data[1] = micros() - US_start_time[1]; // Find difference between rising and falling edge of pulse.
    US_refresh = true; // Flag new ultrasonic data available.
  }
}
