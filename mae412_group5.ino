 /********************************************
 *  mae412_group5.ino
 *  Description: implements all functionality
 *  for Arduino-based two-stage targeting
 *  system.
 *
 *  Authors: Jacob Beyer, Vikash Modi, Jae Yoon
 *
 *
 ********************************************/

/********************************************
  Pin Defines
*********************************************/
// for testing
#define PIN_kill_in 8
#define PIN_VBTA_in 9
#define PIN_PTF_in 10
#define PIN_CTS_out 11
#define PIN_CTL_out 12
#define PIN_CTA_out 13
// end for testing


 // PINOUTS
 // Physical pin  | Arduino pin number    | Function
 // 1               ~                       RESET (ICSP connector, for PixyCam) (might not be used? will need to check that pixycam uses 3-wire SPI)
 // 17              11                      MOSI (ICSP yellow)
 // 18              12                      MISO (ICSP orange)
 // 19              13                      SCK (ICSP brown)
 // 27              A4                      SDA (i2c to ADC)
 // 28              A5                      SCL (i2c)
 // NOTE: no pin defines needed for this block (taken care of by libraries)

 // 1               ~                       RTS (for FTDI comms cable)
 // 2               0                       TxD/Orange (FTDI)
 // 3               1                       RxD/Yellow (FTDI)
 // 7,8                                     Vcc/GND
 // NOTE: no pin defines needed for this block

 //                 10                      stepper enable
 //                 2,3                     pixy az dir,step
 //                 4,5                     pixy el dir,step
 //                 6,7                     laser az dir,step
 //                 8,9                     laser el dir,step
 //                 A0/14 (same pin)        laser diode on/off
 //                 A1/15 (same pin)        Kill switch
 #define P_motor_enable 10
 #define P_track_yaw_dir  2
 #define P_track_yaw_step 3
 #define P_track_pitch_dir  4
 #define P_track_pitch_step 5

 #define P_target_yaw_dir   6
 #define P_target_yaw_step  7
 #define P_target_pitch_dir   8
 #define P_target_pitch_step  9

 #define P_laser_on       14
 #define P_kill_switch    15
 
 
/********************************************
 Timer Things (mostly from docs)
*********************************************/
// DO NOT TOUCH
// These define's must be placed at the beginning before #include "TimerInterrupt.h" 
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4 
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system. 
#define TIMER_INTERRUPT_DEBUG         0 
#define _TIMERINTERRUPT_LOGLEVEL_     0 
#define USE_TIMER_2     true 
// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error 
#include "TimerInterrupt.h" 
// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error 
#include "ISR_Timer.h" 

// end DO NOT TOUCH

/********************************************
 Stepper driver things (mostly from docs)
*********************************************/
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120 //TODO: is this what value we want?
// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1


/********************************************
  Libraries
*********************************************/

#include <Wire.h> // I2C library
#include <I2C_16Bit.h>
#include <SPI.h>
#include <Pixy.h>
#include <VL53L0X.h>
#include "BasicStepperDriver.h"


/********************************************
  Typedefs
*********************************************/

typedef enum State {
  STATE_inactive,
  STATE_search,
  STATE_lock,
  STATE_num_state
};

typedef struct PID_params {
  double curr_target;   // the current desired position
  double curr_error;    // the current error 
  double kp;
  double ki;
  double kd;
  double integrator;      // integrates error over time
  double integrator_sat;
  double prior_error;
  double clip;
};

/********************************************
  Global Variables
*********************************************/

// counter
volatile uint16_t counter_240_hz = 0;             // counts 240Hz timer increments
volatile bool counter_new_val_available = false;  // asserted in timer ISR to tell loop that new counter is ready

// FSM signals
State control_state         = STATE_inactive; // state of control FSM
bool EXT_kill               = false;          // source: external switch, reset to STATE_inactive
bool VB_train_available     = false;          // source: vector board, asserted when valid train on tracks
bool PC_train_found         = false;          // source: pixycam, asserted whenever train is in line-of-sight of pixycam
bool CTRL_tracking_search   = false;          // OUTPUT: asserted when tracking system (pixycam) should scan for a target
bool CTRL_tracking_lock     = false;          // OUTPUT: asserted when tracking system should LOCK IN to the target
bool CTRL_targeting_active  = false;          // OUTPUT: asserted when targeting system (laser) should be activated
bool CTRL_laser_active      = false;          // !! currently unused OUTPUT: asserted when laser diode should be on

// PID elements
PID_params track_pitch_params;
PID_params track_yaw_params;
PID_params target_pitch_params;
PID_params target_yaw_params;

// External measurements
VL53L0X  rangefinder;
uint16_t distance_sensor_raw  = 0;    // ADC ticks, 0-1023
double   distance_train       = 0.0;  // cm, 10-150
Pixy pixy;
uint16_t pixy_train_x         = 0;    // pixels, 0-319
uint16_t pixy_train_y         = 0;    // pixels, 0-199

// stepper motor controllers
BasicStepperDriver track_yaw(MOTOR_STEPS, P_track_yaw_dir, P_track_yaw_step, P_motor_enable);
BasicStepperDriver track_pitch(MOTOR_STEPS, P_track_pitch_dir, P_track_pitch_step, P_motor_enable);
// BasicStepperDriver target_yaw(MOTOR_STEPS, P_target_yaw_dir, P_target_yaw_step, P_motor_enable);
// BasicStepperDriver target_pitch(MOTOR_STEPS, P_target_pitch_dir, P_target_pitch_step, P_motor_enable);


// stepper motor feedback (TODO)

// computed values
double train_x = 0.0;
double train_y = 0.0;

/********************************************
  Interrupt Handlers
*********************************************/

// highest-frequency clock
// #define TIMER_FREQ_HZ 240.0
#define TIMER_FREQ_HZ 3.0       // low-frequency for testing
// hanldes the 240 Hz timer
void HighFrequencyTimerHandler()
{
  // Doing something here inside ISR
  counter_new_val_available = true;
  counter_240_hz++;
  // Serial.print(".");
}



/********************************************
  Communications
*********************************************/

#define ADC_BASE_ADDRESS    0x50 // 101 0000
#define ADC_RESULT_ADDRESS  0x00
#define ADC_CONFIG_ADDRESS  0x02

// set up ADC (no alert pin BUT need to float ADDR pin)
void init_ADC() {
  Serial.println("Initializing ADC...");
  // set CycleTime to 3'b001 (contained in D[7:5]), keep remaining bits 0
  // 8'b0010 0000 = 0x20
  I2C_16Bit_writeToModule(ADC_BASE_ADDRESS, ADC_CONFIG_ADDRESS, 0x20);
  uint16_t response = I2C_16Bit_readFromModule(ADC_BASE_ADDRESS, ADC_RESULT_ADDRESS);
  if (response != 0) {
    Serial.println("ADC initialized!");
  }
  else {
    Serial.println("ADC failed initialization!");
  }
}


/********************************************
  Helper Functions
*********************************************/

// service 60Hz PixyCam update
void loop_pixycam_update(){
  Serial.println("executed pixycam update, counter: " + String(counter_240_hz));
  // while (true) {
  // uint16_t i = 0;
  // uint16_t blocks = 0;
  // uint16_t time = millis();
  // while (!blocks) {
  //   i++;
  //   blocks = pixy.getBlocks();
  // }
  // time = millis() - time;

  // Serial.println("Took " + String(i) + " grabs");
  // delay(100);
  // }
  // Serial.println("test 1");
  uint16_t watchdog = 0;
  uint16_t watchdog_max = 150; // from short experiment: should be ~5 samples on average to acquire blocks, takes 2-4ms
  uint16_t blocks = 0;
  while (!blocks && watchdog < watchdog_max) {
    // Serial.println("test 2");
    watchdog++;
    // delay(100);
    blocks = pixy.getBlocks();
  }

  if (blocks) {
    pixy_train_x = pixy.blocks[0].x;
    pixy_train_y = pixy.blocks[0].y;
  }
  else {
    Serial.println("PIXYCAM: didn't see a train!");
  }


  // compute location of train in global coordinates
  // TODO:
  // - account for offset due to position of pixycam, rotation of the mount, etc.
  // - do all the actual computation
}




// service 60Hz rangefinder update
void loop_rangefinder_update(){
  Serial.println("executed rangefinder update, counter: " + String(counter_240_hz));

  distance_sensor_raw = rangefinder.readRangeContinuousMillimeters();
  distance_train = (double)distance_sensor_raw / 10.0;

  // // convert raw measurement to distance measurement
  // double distance_sensor_volts = (double)(5.0 * distance_sensor_raw)/1024;

  // // TODO: implement interpolation
  // distance_train = 32.1351/(distance_sensor_volts - 0.41288);
}

// service 240Hz position loop updates and update state!
void loop_position_update(){
  Serial.println("executed position update, counter: " + String(counter_240_hz));

  // update state
  State next_state = STATE_inactive;
  switch (control_state) {
    case STATE_inactive:
      if (!VB_train_available || EXT_kill) {
        next_state = STATE_inactive;
      }
      else if (VB_train_available) {
        next_state = STATE_search;
      }
      break;
    case STATE_search:
      if (!VB_train_available || EXT_kill) {
        next_state = STATE_inactive;
      }
      else if (!PC_train_found) {
        next_state = STATE_search;
      }
      else if (PC_train_found) {
        next_state = STATE_lock;
      }
      break;
    case STATE_lock:
      if (!VB_train_available || EXT_kill) {
        next_state = STATE_inactive;
      }
      else if (!PC_train_found) {
        next_state = STATE_search;
      }
      else if (PC_train_found) {
        next_state = STATE_lock;
      }
      break;
  }
  control_state = next_state;
  // update outputs from state machine
  switch (control_state) {
    case STATE_inactive:
      CTRL_tracking_search  = false;
      CTRL_tracking_lock    = false;
      CTRL_targeting_active = false;
      break;
    case STATE_search:
      CTRL_tracking_search  = true;
      CTRL_tracking_lock    = false;
      CTRL_targeting_active = false;
      break;
    case STATE_lock:
      CTRL_tracking_search  = false;
      CTRL_tracking_lock    = true;
      CTRL_targeting_active = true;
      break;
  }

  // execute control calculations
}


/********************************************
  UNIT TESTING
*********************************************/

#define IN_TEST
#ifdef IN_TEST
// tests go here...

void utest_loop_pixycam_update() {
  Serial.println("Beginning loop_pixycam_update test...");

  // TEST 1: Train in front of camera
  Serial.println("\tPlace a train in front of pixycam!");
  delay(500);

  loop_pixycam_update();
  if (pixy_train_x==0 || pixy_train_y==0) {
    Serial.println("\tTEST FAILED: Pixycam didn't see train!");
    return;
  }
  else {
    Serial.println("\tGot (x, y) = (" + String(pixy_train_x) + ", " + String(pixy_train_y) + ")");
  }


  // TEST 2: No train in front of camera
  Serial.println("\tRemove train from in front of pixycam!");
  delay(500);

  loop_pixycam_update();
  Serial.println("\tIf message \"PIXYCAM: didn't see a train!\" didn't appear, then test failed!");
}

void utest_loop_rangefinder_update() {
  Serial.println("Beginning loop_rangefinder_update test...");
  init_ADC();
  Serial.println("\tPlace object 10cm from rangefinder, then send a byte!");
  while (!Serial.available()) {}
  Serial.read(); // flush

  loop_rangefinder_update();
  Serial.println("\tGot raw reading = " + String(distance_sensor_raw));
  Serial.println("\tGot distance    = " + String(distance_train));

}
void utest_loop_position_update() {
  Serial.println("Beginning loop_position_update test...");
}


void calibrate_rangefinder() {

}

#endif


/********************************************
  Setup and Loop
*********************************************/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial.available()){}
  Serial.read(); // flush
  Serial.flush();
  Wire.begin();
  delay(500);

  ITimer2.init();
  // Frequency in float Hz
  if (ITimer2.attachInterrupt(TIMER_FREQ_HZ, HighFrequencyTimerHandler))
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or timer");

  
  // initialize I2C communications
  // I2C_16Bit_begin();
  // init_ADC();

  // initialize sensors
  pixy.init();
  if (!rangefinder.init()) {
    Serial.println("ERROR: Rangefinder not initialized");
    while(true){}
  }
  rangefinder.startContinuous();

  // initialize motor controllers
  track_yaw.begin(RPM, MICROSTEPS);
  track_pitch.begin(RPM, MICROSTEPS);
  // target_yaw.begin(RPM, MICROSTEPS);
  // target_pitch.begin(RPM, MICROSTEPS);

  track_yaw.setEnableActiveState(LOW);
  track_pitch.setEnableActiveState(LOW);
  // target_yaw.setEnableActiveState(LOW);
  // target_pitch.setEnableActiveState(LOW);

  track_yaw.enable();
  track_pitch.enable();
  // target_yaw.enable();
  // target_pitch.enable();

  #define KP 1.0
  #define KI 1.0
  #define KD 1.0
  #define INTEGRATOR_SAT 1.0
  #define CLIP 1.0
  track_pitch_params = {
    .curr_target = 0.0,
    .curr_error = 0.0,
    .kp = KP,
    .ki = KI,
    .kd = KD,
    .integrator = 0.0,
    .integrator_sat = INTEGRATOR_SAT,
    .prior_error = 0.0,
    .clip = CLIP
  };
  track_yaw_params = {
    .curr_target = 0.0,
    .curr_error = 0.0,
    .kp = KP,
    .ki = KI,
    .kd = KD,
    .integrator = 0.0,
    .integrator_sat = INTEGRATOR_SAT,
    .prior_error = 0.0,
    .clip = CLIP
  };
  target_pitch_params = {
    .curr_target = 0.0,
    .curr_error = 0.0,
    .kp = KP,
    .ki = KI,
    .kd = KD,
    .integrator = 0.0,
    .integrator_sat = INTEGRATOR_SAT,
    .prior_error = 0.0,
    .clip = CLIP
  };
  target_yaw_params = {
    .curr_target = 0.0,
    .curr_error = 0.0,
    .kp = KP,
    .ki = KI,
    .kd = KD,
    .integrator = 0.0,
    .integrator_sat = INTEGRATOR_SAT,
    .prior_error = 0.0,
    .clip = CLIP
  };
  


  #ifdef IN_TEST
  Serial.println("Beginning unit tests...");
  utest_loop_pixycam_update();
  utest_loop_rangefinder_update();
  utest_loop_position_update();

  Serial.println("Testing complete!");
  while(true){delay(500);}
  #endif

}

void loop() {
  // put your main code here, to run repeatedly:

  // for testing
  VB_train_available  = digitalRead(PIN_VBTA_in);
  PC_train_found      = digitalRead(PIN_PTF_in);
  EXT_kill            = digitalRead(PIN_kill_in);
  digitalWrite(PIN_CTS_out, CTRL_tracking_search);
  digitalWrite(PIN_CTL_out, CTRL_tracking_lock);
  digitalWrite(PIN_CTA_out, CTRL_targeting_active);
  delay(10);
  // end for testing



  if (counter_new_val_available) {
    counter_new_val_available = false;  // clear flag!!!!

    // handle pixycam and rangefinder updates at correct phases
    switch (counter_240_hz % 4) {
      case 0:
        loop_pixycam_update();
        break;
      case 1:
        break;
      case 2:
        loop_rangefinder_update();
        break;
      case 3:
        break;
    }
    // always execute position loops
    loop_position_update();
    Serial.println("========\nNEW STATE: " + String(control_state) + "\n========\n");
  }
}

