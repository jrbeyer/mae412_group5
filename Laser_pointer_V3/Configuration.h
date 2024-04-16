// 2 Axis robot project 
// JJROBOTS

#include <stdint.h>
#include <math.h>

#define ROBOT_ABSOLUTE_MAX_M1 121 //121 //114 //121   // max degrees
#define ROBOT_ABSOLUTE_MAX_M2 142 //146 //146 //142   // max degress

#define MICROSTEPPING 16
#define MOTOR_STEPS 200 //1.8 degree motor = 360/1.8=200

#define M1_REDUCTION 4
#define M2_REDUCTION 3.2

// This depends on your motor type and reduction
#define M1_AXIS_STEPS_PER_UNIT (((MOTOR_STEPS*MICROSTEPPING)/360.0)*M1_REDUCTION)
#define M2_AXIS_STEPS_PER_UNIT (((MOTOR_STEPS*MICROSTEPPING)/360.0)*M2_REDUCTION)

// Maximun motor acceleration in (steps/seg2)/1000 [acceleration in miliseconds] 30 => 30.000 steps/sec2
#define MAX_ACCEL_M1 30  //50
#define MAX_ACCEL_M2 30  //50 

#define MIN_ACCEL_M1 50
#define MIN_ACCEL_M2 50

// Maximun speed in steps/seg (max 32765)
#define MAX_SPEED_M1 16000 //16000
#define MAX_SPEED_M2 16000 //16000

#define MIN_SPEED_M1 200  //1000
#define MIN_SPEED_M2 200  //1000

// Servo definitions
// Servo1:
#define SERVO1_NEUTRAL 1500  // Servo neutral position Gripped angle
#define SERVO1_MIN_PULSEWIDTH 900
#define SERVO1_MAX_PULSEWIDTH 2100
#define SERVO1_RANGE (SERVO1_MAX_PULSEWIDTH-SERVO1_MIN_PULSEWIDTH)

// Servo2: 
#define SERVO2_NEUTRAL 1500  // Servo neutral position
#define SERVO2_MIN_PULSEWIDTH 900
#define SERVO2_MAX_PULSEWIDTH 2100
#define SERVO2_RANGE (SERVO2_MAX_PULSEWIDTH-SERVO2_MIN_PULSEWIDTH)

#define SERVO_SPEED 3

// UNCOMMENT THIS LINES TO INVERT MOTORS (by default 1)
//#define INVERT_M1_AXIS 1
//#define INVERT_M2_AXIS 1

// Robot
#define ROBOT_MIN_A1 -3600.0
#define ROBOT_MIN_A2 -360.0
#define ROBOT_MAX_A1 360.0
#define ROBOT_MAX_A2 360.0


// Initial robot position in angle/mm
// The robot must be set at this position at start time (steps initialization)
#define ROBOT_INITIAL_POSITION_M1 0
#define ROBOT_INITIAL_POSITION_M2 0

#define TELEMETRY "192.168.4.2"

#define MINIMUN_TIMER_PERIOD 32000 // For timer counters 

#define MSGMAXLEN 20  // Max message length. Message from the APP (parameters)
#define NODATA -20000

#define STOP_TOLERANCE 5  // Tolerance to check if the robot arrive at point

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#define GREEN_LED A1
#define RED_LED A2
#define SWITCH_IN 30

// Structure definition
struct angles {
  float A1;
  float A2;
};


// Variable definitions

// Log and Timer variables
long loop_counter;
int16_t slow_loop_counter;
long timeout_counter;
long wait_counter;
int16_t timestamp=0;
int dt;
long timer_old;
long timer_value;
long slow_timer_old;
long slow_timer_value;
long laser_timer_old;
long laser_timer_value;
int debug_counter;
bool enable_udp_output = false;

// kinematic variables
// position, speed and acceleration are in step units
volatile int16_t position_M1;  // This variables are modified inside the Timer interrupts
volatile int16_t position_M2;
bool working = false;
bool M1stopping = false;
bool M2stopping = false;

int8_t dir_M1;     //(dir=1 positive, dir=-1 negative)
int8_t dir_M2;
float target_angleA1;
float target_angleA2;
int16_t target_position_M1;
int16_t target_position_M2;
int16_t diff_M1;
int16_t diff_M2;
int16_t speed_M1;   // Real motor speed (change dinamically during movements)
int16_t speed_M2;
int16_t config_speed_M1 = MAX_SPEED_M1;    // configured max speed
int16_t config_speed_M2 = MAX_SPEED_M2;
int16_t target_speed_M1 = config_speed_M1;       // target max speed for the actual movement                
int16_t target_speed_M2 = config_speed_M2;

int16_t acceleration_M1;    // Real motor acceleration
int16_t acceleration_M2;    // Real motor acceleration
int16_t config_acceleration_M1 = MAX_ACCEL_M1;   // Acceleration configuration
int16_t config_acceleration_M2 = MAX_ACCEL_M2;
float target_acceleration_M1 = MAX_ACCEL_M1;        // Acceleration for the actual movement
float target_acceleration_M2 = MAX_ACCEL_M2;

int16_t pos_stop_M1;
int16_t pos_stop_M2;
int16_t overshoot_compensation = 20;


int16_t actual_angleA1;
int16_t actual_angleA2;

long servo_counter;
int16_t servo_pos1;
int16_t servo_pos2;
bool servo1_ready = false;
bool servo2_ready = false;

int16_t iCH1;
int16_t iCH2;
int16_t iCH3;
int16_t iCH4;
int16_t iCH5;
int16_t iCH6;
int16_t iCH7;
int16_t iCH8;
uint8_t mode = 0;

uint8_t newMessage;
uint8_t MsgBuffer[MSGMAXLEN];

String MAC;


int16_t myAbs(int16_t param)
{
  if (param < 0)
    return -param;
  else
    return param;
}

long myAbsLong(long param)
{
  if (param < 0)
    return -param;
  else
    return param;
}

int sign(int val)
{
  if (val < 0)
    return (-1);
  else
    return (1);
}
