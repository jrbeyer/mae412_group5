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
 // Physical pin  | Arduino pin number    | Function

 // 27              A4 (ESP 8)              SDA (i2c)
 // 28              A5 (ESP 9)              SCL (i2c)
 // NOTE: no pin defines needed for this block (taken care of by libraries)

 // 1               ~                       RTS (for FTDI comms cable)
 // 2               0                       TxD/Orange (FTDI)
 // 3               1                       RxD/Yellow (FTDI)
 // 7,8                                     Vcc/GND
 // NOTE: no pin defines needed for this block

 //                 10                      Pixy stepper enable
 //                 11                      laser stepper enable
 //                 6,7                     track yaw dir,step
 //                 39,40                   track pitch dir,step
 //                 4,5                     target yaw dir,step
 //                 2,3                     target pitch dir,step
 //                 A0/14 (same pin)        laser diode on/off
 //                 16                      Kill switch
 #define P_motor_enable 10
 #define P_laser_motor_enable 11
 #define P_track_yaw_dir  6 
 #define P_track_yaw_step 7 
 #define P_track_pitch_dir  39
 #define P_track_pitch_step 40

 #define P_target_yaw_dir   4
 #define P_target_yaw_step  5
 #define P_target_pitch_dir   2
 #define P_target_pitch_step  3

 #define P_laser_on       14
 #define P_kill_switch    16

 #define PIN_LED 15
 
 
/********************************************
 Timer Things (mostly from docs)
*********************************************/
// DO NOT TOUCH
// These define's must be placed at the beginning before #include "TimerInterrupt.h" 
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4 
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system. 
#define TIMER_INTERRUPT_DEBUG         0 
#define _TIMERINTERRUPT_LOGLEVEL_     0 
// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error 
#include "TimerInterrupt_Generic.h" 
// end DO NOT TOUCH

/********************************************
 Stepper driver things (mostly from docs)
*********************************************/
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 100 
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

#include <esp_now.h>
#include <WiFi.h>

#include "mae412_group5_defines.h"

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
  double command_clip;
  long   count_est;       // estimate of how many counts the stepper has travelled
  long   count_clip;      // clip (keep pitch within [-36, 36]deg, keep yaw within [-720, 720]deg) (but in encoder counts)
};

/********************************************
  Global Variables
*********************************************/

// ESP NOW
// message passing
String success;
message_S inbound_message;
uint16_t wifi_watchdog;

// configure ESP NOW
esp_now_peer_info_t peerInfo;
// callback for data sent
// shouldn't be sending data
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}
// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&inbound_message, incomingData, sizeof(inbound_message));
  wifi_watchdog = 0;

  
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  // if (inbound_message.pixy_saw_train) {
  //     Serial.println("Got train! clearing this flag...");
  //     inbound_message.pixy_saw_train = false;

  //     Serial.println("train_x: " + String(inbound_message.pixy_train_x));
  //     Serial.println("train_y: " + String(inbound_message.pixy_train_y));
  //   }
  //   if (inbound_message.rangefinder_got_range) {
  //     Serial.println("Got distance! clearing this flag...");
  //     inbound_message.rangefinder_got_range = false;

  //     Serial.println("distance (mm): " + String(inbound_message.rangefinder_range_mm));
  //   }
}


ESP32Timer ITimer1(1);

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

uint16_t PC_train_watchdog  = 0;              // don't want FSM to change immediately in case of momentary glitch, so keep watchdog
uint16_t PC_train_watchdog_max = 10;          // but after 10 times in a row something's probably wrong, should search again

// PID elements
PID_params track_pitch_params;
PID_params track_yaw_params;
PID_params target_pitch_params;
PID_params target_yaw_params;

// External measurements
VL53L0X  rangefinder;
double   distance_train     = 0.0;  // mm
Pixy pixy;
double pixy_train_x         = 0;    // pixels, 0-319
double pixy_train_y         = 0;    // pixels, 0-199

// stepper motor controllers
BasicStepperDriver track_yaw(MOTOR_STEPS, P_track_yaw_dir, P_track_yaw_step, P_motor_enable);
BasicStepperDriver track_pitch(MOTOR_STEPS, P_track_pitch_dir, P_track_pitch_step, P_motor_enable);
BasicStepperDriver target_yaw(MOTOR_STEPS, P_target_yaw_dir, P_target_yaw_step, P_laser_motor_enable);
BasicStepperDriver target_pitch(MOTOR_STEPS, P_target_pitch_dir, P_target_pitch_step, P_laser_motor_enable);

bool steppers_enabled = false;  // keep track of if we have enabled or disabled the steppers
bool sweep_ccw = false;   // keep track of which direction we are sweeping in if true then ccw if false then cw

// stepper motor feedback (TODO)

// computed values
double x_train = 0.0;
double y_train = 0.0;

long theta_laser_command_count = 0;
long phi_laser_command_count   = 0;

/********************************************
  Interrupt Handlers
*********************************************/

// highest-frequency clock
// #define TIMER_FREQ_HZ 240.0
#define TIMER_FREQ_HZ 80.0
// #define TIMER_FREQ_HZ 120.0
// hanldes the 240 Hz timer
bool HighFrequencyTimerHandler(void* timerNo)
{
  // Doing something here inside ISR
  counter_new_val_available = true;
  counter_240_hz++;
  return true;
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


// request information from the VB arduino
#define BIT_VBTA (0x01) // bit 1: vector board says train available
void request_arduino_comms() {
  // Serial.println("Requesting from arduino...");
  byte RxByte;
  Wire.requestFrom(0x87, 1); // Request from arduino @ 0x87, Data Length = 1Byte
  while(Wire.available()) {  // Read receive from arduino
    RxByte = Wire.read();
  }
  VB_train_available = (bool)(RxByte & BIT_VBTA);

  Serial.println("Got raw byte: " + String(RxByte));
}


/********************************************
  Helper Functions
*********************************************/

// service 60Hz PixyCam update
void loop_pixycam_update(){
  // // Serial.println("executed pixycam update, counter: " + String(counter_240_hz));


  if (inbound_message.pixy_saw_train) {
    inbound_message.pixy_saw_train = false;
    // smooth update
    const double alpha = 0.9;
    pixy_train_x = (alpha*inbound_message.pixy_train_x) + (1-alpha)*pixy_train_x;
    pixy_train_y = (alpha*inbound_message.pixy_train_y) + (1-alpha)*pixy_train_y;
    // reset watchdog
    PC_train_watchdog = 0;
    PC_train_found = true;
    // Serial.println("Found train!");
  }
  else { 
    // TODO: make this more robust; slowly move perceived location to center of frame?
    pixy_train_x = (PIXY_MAX_X/2);
    pixy_train_y = (PIXY_MAX_Y/2);
    PC_train_watchdog++;
  }

  // compute location of train in global coordinates
  // TODO:
  // - account for offset due to position of pixycam, rotation of the mount, etc.
  // - do all the actual computation
  const double DELTA_X = 412.75; //mm
  const double DELTA_Y = 203.2;
  const double DELTA_Z = 241.3;
  const double h = 190.5;

  double theta_pixy_rad = 0.45*PI/180.0*track_yaw_params.count_est;
  double phi_pixy_rad   = 0.5625*PI/180.0*track_pitch_params.count_est;
  double d_range_proj = distance_train*cos(phi_pixy_rad);

  x_train = d_range_proj*sin(theta_pixy_rad);
  y_train = d_range_proj*cos(theta_pixy_rad);

  double x_laser = DELTA_Y - y_train;
  double y_laser = DELTA_X + x_train;

  double theta_laser_rad = atan2(x_laser, y_laser);
  double d_laser_proj = sqrt(x_laser*x_laser + y_laser*y_laser);

  double z_abs = h - distance_train*sin(phi_pixy_rad);
  double z_laser = z_abs - DELTA_Z;

  double phi_laser_rad = atan2(z_laser, d_laser_proj);

  theta_laser_command_count = (long)(180.0*theta_laser_rad/(0.45*PI));
  phi_laser_command_count   = (long)(180.0*phi_laser_rad /(0.5625*PI));

  if (counter_240_hz % 100 == 0) {
  // if (0) {
    Serial.println("Position estimate: (" + String(x_train) + ", " + String(y_train) + ")");
    Serial.println("Theta command:      " + String(theta_laser_command_count));
    Serial.println("Phi command:        " + String(phi_laser_command_count));
  }
  
}


// service 60Hz rangefinder update
// DEPRECATED
void loop_rangefinder_update(){
  // Serial.println("executed rangefinder update, counter: " + String(counter_240_hz));

  if (inbound_message.rangefinder_got_range) {
    inbound_message.rangefinder_got_range = false;
    distance_train = inbound_message.rangefinder_range_mm;
  }
}

// reset PID parameters so motors don't go crazy
void reset_PID(PID_params* params) {
  params->integrator = 0.0;
  params->curr_error = 0.0;
  params->prior_error = 0.0;
}


// enable or disable all steppers
void enable_disable_steppers(bool enable) {
  if (enable) {
    track_yaw.enable();
    track_pitch.enable();
    target_yaw.enable();
    target_pitch.enable();
    steppers_enabled = true;
  }
  else {
    track_yaw.disable();
    track_pitch.disable();
    target_yaw.disable();
    target_pitch.disable();
    steppers_enabled = false;
  }
}
// Have the steppers search for the target by sweeping the straight lines of track from entry to exit
void search() {
  if (track_pitch_params.count_est != 0) {
    unwind_stepper(&track_pitch_params, &track_pitch);
  }
  sweep_steppers(&track_yaw_params, &track_yaw);
}


//sweep steppers to find target panning right and left in certain range, 
// until pixie cam sees the target
void sweep_steppers(PID_params* params, BasicStepperDriver* driver) {

  const long cw_sweep_limit = 200; // will be 7 steps to the right of home (105 counts)
  const long ccw_sweep_limit = -200; // will be 7 steps to the left of home (105 counts)
  
  long step_size = (sweep_ccw ? -15 : 15); // sweep in steps of 15 or more neg if ccw pos if cw

  if ((sweep_ccw && params->count_est < ccw_sweep_limit) || (!sweep_ccw && params->count_est > cw_sweep_limit)) {
    sweep_ccw = !sweep_ccw;
    step_size = -step_size; // reverse direction
  }
  driver->move(step_size);
  params->count_est += step_size;
}

// unwind if we hit -720 or 720 degrees
void unwind_stepper(PID_params* params, BasicStepperDriver* driver) {
  // Serial.println("Unwinding!");
  noInterrupts();
  long step_size = (params->count_est < 0) ? 15 : -15; // step slowly back to home


  while (abs(params->count_est) >= abs(step_size)) {
    driver->move(step_size);
    params->count_est += step_size;
    delay(30); // drop this down with testing
  }
  
  // step the rest of the way
  driver->move(-params->count_est);
  params->count_est = 0;

  interrupts();
}

// bring all steppers back to home
void home_steppers() {
  unwind_stepper(&track_pitch_params, &track_pitch);
  delay(100);
  unwind_stepper(&track_yaw_params, &track_yaw);
  delay(100);
  unwind_stepper(&target_pitch_params, &target_pitch);
  delay(100);
  unwind_stepper(&target_yaw_params, &target_yaw);
  delay(100);
}

// generic PID execution functions (can execute once for each stepper motor)
#define CONTROL_PERIOD (1.0 / TIMER_FREQ_HZ)
void execute_PID(PID_params* params, BasicStepperDriver* driver, int i) {
  // update integrator
  params->integrator += params->curr_error;
  if (params->integrator >= params->integrator_sat) {
    params->integrator = params->integrator_sat;
  }
  else if (params->integrator <= -params->integrator_sat) {
    params->integrator = -params->integrator_sat;
  }
  // PID lead out
  double command =  (params->kp * params->curr_error) 
                  + (params->kd * (params->curr_error - params->prior_error)/CONTROL_PERIOD) 
                  + (params->ki * params->integrator * CONTROL_PERIOD);
  // update priors and clip command
  params->prior_error = params->curr_error;
  if (command >= params->command_clip) {
    command = params->command_clip;
  }
  else if (command <= -params->command_clip) {
    command = -params->command_clip;
  }

  // bounds clipping
  long command_l = (long)command;
  if (params->count_est + command_l > params->count_clip || params->count_est + command_l < -params->count_clip) {
    Serial.println("Hit count limit: " + String(params->count_est));
    command_l = 0;
    // TODO: unwinding procedure untested in this context
    if (params == &track_yaw_params) {
      unwind_stepper(params, driver);
    }
  }

  // issue command
  driver->move(command_l);
  params->count_est += command_l;
  
  if (i%10 == 0) {
    Serial.println("Executed PID loop, new params: ");
    Serial.println("\tError:      " + String(params->curr_error));
    Serial.println("\tLast Error: " + String(params->prior_error));
    Serial.println("\tIntegrator: " + String(params->integrator));
    Serial.println("\tCommand:    " + String(command_l));
    Serial.println("\tEstimate:   " + String(params->count_est));
  }
}

void move_laser_pointer() {

  long laser_delta_theta = theta_laser_command_count - target_yaw_params.count_est;

  if (laser_delta_theta >= target_yaw_params.command_clip) {
    laser_delta_theta = target_yaw_params.command_clip;
  }
  else if (laser_delta_theta <= -target_yaw_params.command_clip) {
    laser_delta_theta = -target_yaw_params.command_clip;
  }

  if (theta_laser_command_count > 67 
    ||theta_laser_command_count < -220) {
    
    laser_delta_theta = 0;
    unwind_stepper(&target_yaw_params, &target_yaw);
  }
  target_yaw.move(laser_delta_theta);
  target_yaw_params.count_est += laser_delta_theta;

  long laser_delta_phi   = phi_laser_command_count - target_pitch_params.count_est;

  if (laser_delta_phi >= target_pitch_params.command_clip) {
    laser_delta_phi = target_pitch_params.command_clip;
  }
  else if (laser_delta_phi <= -target_pitch_params.command_clip) {
    laser_delta_phi = -target_pitch_params.command_clip;
  }

  if (phi_laser_command_count >  0 
    ||phi_laser_command_count < -107) {
    Serial.println("Hit laser pitch clip: " + String(target_pitch_params.count_est));
    laser_delta_phi = 0;
  }

  target_pitch.move(laser_delta_phi);
  target_pitch_params.count_est += laser_delta_phi;
}

// service 240Hz position loop updates and update state!
void loop_position_update(){
  // Serial.println("executed position update, counter: " + String(counter_240_hz));


  // check this watchdog first to make sure we have up-to-date control signals
  if (PC_train_watchdog >= PC_train_watchdog_max) {
    PC_train_found = false;
    PC_train_watchdog = PC_train_watchdog_max;
  }
  
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
  // TODO: deprecated
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
  switch (control_state) {
    case STATE_inactive:
      if (steppers_enabled) {
        home_steppers();
        enable_disable_steppers(false);
      }
      digitalWrite(P_laser_on, LOW);
      break;
    case STATE_search:
      if (!steppers_enabled) {
        enable_disable_steppers(true);
      }
      digitalWrite(P_laser_on, LOW);
      search();
      break;
    case STATE_lock: 
      if (!steppers_enabled) {
        enable_disable_steppers(true);
      }
      digitalWrite(P_laser_on, HIGH);
      track_yaw_params.curr_error =   ((PIXY_MAX_X/2.0) - pixy_train_x);
      track_pitch_params.curr_error = ((PIXY_MAX_Y/2.0) - pixy_train_y);
      execute_PID(&track_yaw_params, &track_yaw, 1);
      execute_PID(&track_pitch_params, &track_pitch, 1);
      //TODO: move laser pointer too
      move_laser_pointer();
      break;
  }



  // laser pointer control update
}



/********************************************
  Setup and Loop
*********************************************/

#include "mae412_group5_utests.h"

void setup() {
  // initialize motor controllers
  track_yaw.begin(RPM, MICROSTEPS);
  track_pitch.begin(RPM, MICROSTEPS);
  target_yaw.begin(RPM, MICROSTEPS);
  target_pitch.begin(RPM, MICROSTEPS);

  
  track_yaw.setEnableActiveState(LOW);
  track_pitch.setEnableActiveState(LOW);
  target_yaw.setEnableActiveState(LOW);
  target_pitch.setEnableActiveState(LOW);
  Serial.begin(115200);

  pinMode(P_kill_switch, INPUT);
  enable_disable_steppers(false);

  pinMode(P_laser_on, OUTPUT);
  digitalWrite(P_laser_on, LOW);

  // I2C
  Wire.begin();
  delay(500);
  WiFi.mode(WIFI_STA);
  delay(5000);


  // Init ESP-NOW
  // slow flash is bad
  pinMode(PIN_LED, OUTPUT);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while (true) {
      delay(200);
      digitalWrite(PIN_LED, LOW);
      delay(200);
      digitalWrite(PIN_LED, HIGH);
    }
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, pixyESPAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    // return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  // blue light on when done with ESP NOW config
  digitalWrite(PIN_LED, HIGH);
  wifi_watchdog = 0;


  // Frequency in float Hz
  if (ITimer1.attachInterrupt(TIMER_FREQ_HZ, HighFrequencyTimerHandler))
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or timer");


  #define KP 0.025
  #define KI 0.5
  #define KD 0.001
  #define INTEGRATOR_SAT 100.0
  #define COMMAND_CLIP 40.0
  #define PITCH_COUNT_CLIP 64
  #define YAW_COUNT_CLIP 1600
  #define LASER_YAW_COUNT_CLIP 800
  #define SCALE (80.0/80.0)

  // TODO Why are pitch and yaw params defined twice?
  track_pitch_params = {
    .curr_target = 0.0,
    .curr_error = 0.0,
    .kp = KP*SCALE,
    .ki = KI*SCALE,
    .kd = KD*SCALE,
    .integrator = 0.0,
    .integrator_sat = INTEGRATOR_SAT,
    .prior_error = 0.0,
    .command_clip = COMMAND_CLIP,
    .count_est = 0,
    .count_clip = PITCH_COUNT_CLIP,
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
    .command_clip = COMMAND_CLIP,
    .count_est = 0,
    .count_clip = YAW_COUNT_CLIP,
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
    .command_clip = COMMAND_CLIP,
    .count_est = 0,
    .count_clip = PITCH_COUNT_CLIP,
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
    .command_clip = COMMAND_CLIP,
    .count_est = 0,
    .count_clip = LASER_YAW_COUNT_CLIP,
  };
  
  int i;
  for (i = 0; i < 10; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(500);
    digitalWrite(PIN_LED, LOW);
    delay(500);
  }
  digitalWrite(PIN_LED, HIGH);


  // #define IN_TEST
  #ifdef IN_TEST
  Serial.println("Beginning unit tests...");
  // utest_loop_pixycam_update();
  // utest_loop_rangefinder_update();
  // utest_loop_position_update();
  // utest_execute_PID();
  // utest_stepper_motor();
  // utest_receive_esp_now();
  // utest_request_arduino_comms();


  Serial.println("Testing complete!");
  while(true){delay(500);}
  #endif

}

void loop() {
  // put your main code here, to run repeatedly:


  if (counter_new_val_available) {
    counter_new_val_available = false;  // clear flag!!!!

    // important: get kill switch first! Kill when the switch is OFF
    EXT_kill = !digitalRead(P_kill_switch);

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
        // request_arduino_comms();
        VB_train_available = true;
        break;
    }

    // enforce ESP-NOW watchdog
    #define WIFI_WATCHDOG_MAX 20
    wifi_watchdog++;
    if (wifi_watchdog > WIFI_WATCHDOG_MAX) {
      // Serial.println("Lost ESP-NOW connection!");
      // reset all control signals so we don't do anything stupid
      reset_PID(&track_pitch_params);
      reset_PID(&track_yaw_params);
      // reset_PID(&target_pitch_params);
      // reset_PID(&target_yaw_params);
      wifi_watchdog--; // don't let it overflow
    }
    else {
      // always execute position loops
      loop_position_update();
    }

    if (counter_240_hz % 50 == 0) {
      String state_string = (control_state == STATE_inactive) ? "inactive" :
                            (control_state == STATE_search)   ? "search"   :
                            (control_state == STATE_lock)     ? "lock"     : "bad state";
      Serial.println("========\nSTATE: " + state_string + "\n========");

    }
  }
}

