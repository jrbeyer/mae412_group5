// JJROBOTS Two Axis.
// Author: Jose Julio & Juan pedro & Jonathan (JJROBOTS)
// Hardware: New JJROBOTS DEVIA M0 Board with Arduino M0 & ESP8266
// Date: 13/02/2018
// Last updated: 218/04/2020
// Version: 0.1
// Project page : http://jjrobots.com/
// License: Open Software GPL License v2

// Hardware: JJROBOTS DEVIA M0 board
// Board: Arduino/Genuine Zero (Native USB Port)

// Motor1:
// Enable: Arduino pin D11 (PA16)
// Step: Arduino pin 5 (PA15)
// Dir: Arduino pin 6 (PA20)

// Motor2:
// Enable: Arduino pin D11 (PA16)
// Step: Arduino pin 7 (PA21)
// Dir: Arduino pin 8 (PA06)

// Mircrostepping : Arduino pin A4 (PA05) (default 1/16)

#define VERSION "Scara v0.16"
//#define DEBUG 0

// ROBOT and USER configuration parameters
#include "Configuration.h"
#include <Servo.h>
#include <Wire.h>

Servo servo1; // Servo1
Servo servo2; // Servo2

// Configuration: Pins, servos, Steppers, Wifi...
void setup()
{
  // STEPPER PINS ON JJROBOTS DEVIA M0 BOARD
  pinMode(11, OUTPUT); // ENABLE MOTORS   ATSAMD21:PA16
  pinMode(5, OUTPUT); // STEP MOTOR 1 ATSAMD21:PA15
  pinMode(6, OUTPUT); // DIR MOTOR 1  ATSAMD21:PA20
  pinMode(7, OUTPUT); // STEP MOTOR 2 ATSAMD21:PA21
  pinMode(8, OUTPUT); // DIR MOTOR 2  ATSAMD21:PA06
  pinMode(9, OUTPUT); // STEP MOTOR 3 ATSAMD21:PA07
  pinMode(10, OUTPUT); // DIR MOTOR 3  ATSAMD21:PA18

  pinMode(A4, OUTPUT);    // Microstepping output
  digitalWrite(A4, HIGH); // 1/16 (default config)

  // Servos
  pinMode(3, OUTPUT); // SERVO1  ATSAMD21:PA09
  pinMode(4, OUTPUT); // SERVO2  ATSAMD21:PA08

  pinMode(12, OUTPUT); // Electromagnet output (PA19) [optional]
  digitalWrite(12, LOW); //Disabled

  pinMode(RED_LED, OUTPUT); // RED LED
  pinMode(GREEN_LED, OUTPUT); // GREEN LED
  pinMode(SWITCH_IN, INPUT_PULLUP);  // Input Switch
  digitalWrite(SWITCH_IN, OUTPUT); // PULLUP

  digitalWrite(11, HIGH);  // Disbale stepper motors
  digitalWrite(RED_LED, HIGH);  // RED LED ON

  // Serial ports initialization
  delay(100);
  SerialUSB.begin(115200); // Serial output to console
  Serial1.begin(115200); // Wifi initialization
  Wire.begin();
  delay(1000);

#ifdef DEBUG
  delay(10000);  // Only needed for serial debug
  SerialUSB.println(VERSION);
#endif

  // WIFI MODULE INITIALIZATION PROCESS
  SerialUSB.println("WIFI init");
  Serial1.flush();
  Serial1.print("+++");  // To ensure we exit the transparent transmision mode
  delay(100);

  ESPsendCommand(String("AT"), String("OK"), 1);
  //ESPsendCommand(String("AT+RST"), String("OK"), 2); // ESP Wifi module RESET
  //ESPwait(String("ready"), 6);
  ESPsendCommand(String("AT+GMR"), String("OK"), 5);

  Serial1.println("AT+CIPSTAMAC?");
  ESPgetMac();
  SerialUSB.print("MAC:");
  SerialUSB.println(MAC);
  delay(100);
  ESPsendCommand(String("AT+CWMODE=2"), String("OK"), 3); // Soft AP mode
  //SerialUSB.println("Aqui tambien");
  // Generate Soft AP. SSID=JJROBOTS_XX (XX= user MAC last characters), PASS=87654321
  String cmd =  String("AT+CWSAP=\"JJROBOTS_") + MAC.substring(MAC.length() - 2, MAC.length()) + String("\",\"87654321\",5,3");
  ESPsendCommand(cmd, String("OK"), 6);

  // Start UDP SERVER on port 2222, telemetry port 2223
  SerialUSB.println("Start UDP server");
  ESPsendCommand(String("AT+CIPMUX=0"), String("OK"), 3);  // Single connection mode
  delay(10);
  ESPsendCommand(String("AT+CIPMODE=1"), String("OK"), 3); // Transparent mode
  delay(10);
  String Telemetry = String("AT+CIPSTART=\"UDP\",\"") + String(TELEMETRY) + String("\",2223,2222,0");
  ESPsendCommand(Telemetry, String("CONNECT"), 3);
  SerialUSB.println(Telemetry);
  delay(200);
  ESPsendCommand(String("AT+CIPSEND"), String('>'), 2); // Start transmission (transparent mode)

  // Start TCP SERVER on port 2222, telemetry port 2223
  //SerialUSB.println("Start TCP server");
  //ESPsendCommand("AT+CIPCLOSE=0","OK",3);
  //ESPsendCommand("AT+CIPSERVER=0","OK",3);
  //ESPsendCommand("AT+CIPMUX=1", "OK", 3);  // Multiple connection mode
  //ESPsendCommand("AT+CIPMODE=1", "OK", 3); // Transparent mode
  //ESPsendCommand("AT+CIPSERVER=1,2222", "OK", 3); // TCP server
  delay(100);

  // Init servos
  initServo();
  moveServo1(SERVO1_NEUTRAL);
  moveServo2(SERVO2_NEUTRAL);

  // Debug: Output parameters
  //SerialUSB.print("Max_acceleration_x: ");
  //SerialUSB.println(acceleration_x);
  //SerialUSB.print("Max_acceleration_y: ");
  //SerialUSB.println(acceleration_y);
  //SerialUSB.print("Max speed X: ");
  //SerialUSB.println(MAX_SPEED_X);
  //SerialUSB.print("Max speed Y: ");
  //SerialUSB.println(MAX_SPEED_Y);

  // STEPPER MOTORS INITIALIZATION
  SerialUSB.println("Steper motors initialization...");
  SerialUSB.print("Motor1 AXIS per STEP unit:");
  SerialUSB.println(M1_AXIS_STEPS_PER_UNIT);
  SerialUSB.print("Motor2 AXIS per STEP unit:");
  SerialUSB.println(M2_AXIS_STEPS_PER_UNIT);
  
  timersConfigure();
  SerialUSB.println("Timers initialized");
  timersStart(); //starts the timers
  SerialUSB.println("Timers started");
  delay(100);
  SerialUSB.println("Moving to initial position...");

  //Initializing init position
  target_angleA1 = ROBOT_INITIAL_POSITION_M1;
  target_angleA2 = ROBOT_INITIAL_POSITION_M2;
  position_M1 = target_angleA1 * M1_AXIS_STEPS_PER_UNIT;
  position_M2 = target_angleA2 * M2_AXIS_STEPS_PER_UNIT;

  //target_angleA1 = ROBOT_INITIAL_POSITION_M1;
  //target_angleA2 = ROBOT_INITIAL_POSITION_M2 + ROBOT_INITIAL_POSITION_M1*AXIS2_AXIS1_correction;

  configSpeed(MAX_SPEED_M1, MAX_SPEED_M2);
  configAcceleration(MAX_ACCEL_M1, MAX_ACCEL_M2);
  setSpeedAcc();
  target_position_M1 = position_M1;
  target_position_M2 = position_M2;

  SerialUSB.println("Initial position configured!");
 
  SerialUSB.println(" Ready...");
  SerialUSB.print(" JJROBOTS 2 Axis platform ");
  SerialUSB.println(VERSION);
  timer_old = micros();
  slow_timer_old = millis();
  laser_timer_old = millis();
  timeout_counter = 0;

  digitalWrite(RED_LED, LOW);  // RED LED OFF
  digitalWrite(GREEN_LED, HIGH);  // GREEN LED ON

  // Enable motors
  digitalWrite(11, LOW);  // Enable motors
}

// *************** APLICATION MAIN LOOP ***********************
void loop()
{
  MsgRead();     // Read network messages
  USBMsgRead();  // Read USB messages
  if (newMessage)
  {
    newMessage = 0;
    //debugMsg();
    if (mode == 1) {
      // Manual mode: Direct kinematics
      if ((iCH1 != NODATA) || (iCH2 != NODATA)) {
        setAxis1(iCH1 / 100.0);
        setAxis2(iCH2 / 100.0);
        setSpeedAcc();
      }
      
      // Servos:
      if (iCH4 != NODATA)
        moveServo1(iCH4 * SERVO1_RANGE / 1000.0 + SERVO1_MIN_PULSEWIDTH);
      if (iCH5 != NODATA)
        moveServo2(iCH5 * SERVO2_RANGE / 1000.0 + SERVO2_MIN_PULSEWIDTH);
    }
    else if (mode == 2) {
      
    }
    else if (mode == 3) {
      
    }

    else if (mode == 4) { // Emergency stop
      digitalWrite(11, HIGH);  // Disable motors
    }

    else if (mode == 5) { // Robot calibration
      SerialUSB.println("->Motors calibration...");
      motorsCalibration();
      setSpeedAcc();
      working = false;
    }
  }

  timer_value = micros();
  dt = timer_value - timer_old;
  if (dt >= 1000) { // 1Khz loop for position,speed and acceleration control
    if (dt > 1500) {
      SerialUSB.print("!!");  // Timing warning
      SerialUSB.println(dt);
    }
    timer_old = timer_value;

    positionControl(1000);   // position, speed and acceleration control of stepper motors

    loop_counter += 1;

    // Debug loop counter
    if (loop_counter % 100 == 0) {
      char message[80];
      // Calculate actual robot angles based on internal motor positions (steps)
      float dA1 = (float)position_M1 / M1_AXIS_STEPS_PER_UNIT;
      float dA2 = (float)position_M2 / M2_AXIS_STEPS_PER_UNIT;
      //sprintf(message, "#%d:%.2f,%.2f,%d,%d", loop_counter/100,dA1, dA2,speed_M1,speed_M2);
      sprintf(message, "#%.2f,%.2f,%d,%d",dA1, dA2,speed_M1,speed_M2);
      //sprintf(message, "#%d:%d,%d,%d,%d", loop_counter/100,position_M1, position_M2,speed_M1,speed_M2);
      SerialUSB.println(message);
    }

    slow_timer_value = millis();
    if ((slow_timer_value - slow_timer_old) >= 50) {  // Slow loop (20hz)
      char message[80];
      slow_timer_old = slow_timer_value;

      // Check if robot is stopped (reach final position)
      diff_M1 = myAbs(target_position_M1 - position_M1);
      diff_M2 = myAbs(target_position_M2 - position_M2);
      
      if ((diff_M1 < STOP_TOLERANCE) && (diff_M2 < STOP_TOLERANCE))
        working = false;
      else
        working = true;
      

      // Timestamp for status message...
      timestamp += 1;
      if (timestamp > 999)
        timestamp = 0;

      // Calculate actual robot angles based on internal motor positions (steps)
      actual_angleA1 = (position_M1 / M1_AXIS_STEPS_PER_UNIT) * 10;
      actual_angleA2 = (position_M2 / M2_AXIS_STEPS_PER_UNIT) * 10;

      //SerialUSB.println(message);
      //if (enable_udp_output) {       // Output UDP messages if we detect an UDP external interface
      //  Serial1.println(message);
      //}
    } // 20hz loop
  } // 1Khz loop
}
