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
 Timer Things (mostly from docs)
*********************************************/
// DO NOT TOUCH
 
// These define's must be placed at the beginning before #include "TimerInterrupt.h" 
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4 
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system. 
#define TIMER_INTERRUPT_DEBUG         0 
#define _TIMERINTERRUPT_LOGLEVEL_     0 
#define USE_TIMER_1     true 
// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error 
#include "TimerInterrupt.h" 
// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error 
#include "ISR_Timer.h" 

// Select the timers you're using, here ITimer1
#define USE_TIMER_1     true

// end DO NOT TOUCH

/********************************************
  Libraries
*********************************************/

#include <Wire.h> // I2C library

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
  double curr_target,   // the current desired position
  double curr_error,    // the current error 
  double kp,
  double ki,
  double kd,
  double integrator,      // integrates error over time
  double integrator_sat,
  double prior_error,
  double clip,
};

/********************************************
  Global Variables
*********************************************/

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


/********************************************
  Interrupt Handlers
*********************************************/

// highest-frequency clock
// #define TIMER_FREQ_HZ 240.0
#define TIMER_FREQ_HZ 1.0       // low-frequency for testing
// hanldes the 240 Hz timer
void HighFrequencyTimerHandler()
{
  // Doing something here inside ISR
  counter_new_val_available = true;
  counter_240_hz++;
}

/********************************************
  Helper Functions
*********************************************/

// service 60Hz PixyCam update
void loop_pixycam_update(){
  Serial.println("executed pixycam update, counter: " + String(counter_240_hz));
}

// service 60Hz rangefinder update
void loop_rangefinder_update(){
  Serial.println("executed rangefinder update, counter: " + String(counter_240_hz));
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
  Setup and Loop
*********************************************/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(500);

  ITimer1.init();
  // Frequency in float Hz
  if (ITimer1.attachInterrupt(TIMER_FREQ_HZ, HighFrequencyTimerHandler))
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or timer");


  // initialize PID parameters
  


// #define PIN_kill_in 8
// #define PIN_VBTA_in 9
// #define PIN_PTF_in 10
// #define PIN_CTS_out 11
// #define PIN_CTL_out 12
// #define PIN_CTA_out 13

  pinMode(PIN_kill_in, INPUT);
  pinMode(PIN_VBTA_in, INPUT);
  pinMode(PIN_PTF_in, INPUT);
  pinMode(PIN_CTS_out, OUTPUT);
  pinMode(PIN_CTL_out, OUTPUT);
  pinMode(PIN_CTA_out, OUTPUT);

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
