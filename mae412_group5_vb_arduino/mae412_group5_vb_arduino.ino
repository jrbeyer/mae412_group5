 /********************************************
 *  mae412_group5_vb_arduino.ino
 *  Description: code for arduino mounted on
 *  vector board.
 *
 *  Authors: Jacob Beyer, Vikash Modi, Jae Yoon
 *
 *
 ********************************************/


 // PINOUTS
 // Physical pin  | Arduino pin number    | Function
 // 27              A4 (ESP 8)              SDA (i2c) Vector board I/O 13, purple
 // 28              A5 (ESP 9)              SCL (i2c) Vector board I/O 14, dark blue

 // 16              10                      Serial Rx (ACIA)
 // 17              11                      Serial Tx (ACIA)

 // 11              5                       Switch Direction A
 // 12              6                       Switch Direction B
 // 13              7                       Switch Trigger

 // 4               2                       Hall Effect B Input
 // 5               3                       Hall Effect C Input

#define PIN_Rx          10
#define PIN_Tx          11
#define PIN_DIR_A       5
#define PIN_DIR_B       6
#define PIN_SWITCH_TRIG 7
#define PIN_HE_B        2
#define PIN_HE_C        3

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


#include <Wire.h>
#include <SoftwareSerial.h>

// straight_turnout = top switch straight, bottom switch turnout
// turnout_straight = top switch turnout, bottom switch straight
// where top = 1,2; bottom = 3,4 in diagram :)

// TODO: how to set in order to actually throw in the right direction?
typedef enum switch_direction {
  DIR_straight_turnout = HIGH,
  DIR_turnout_straight = LOW,
};

// FSM states
typedef enum State_VB {
  STATE_nominal,
  STATE_delay,
  STATE_throw_to_inverse,
  STATE_inverse,
  STATE_throw_to_nominal,
  num_state,
};





/********************************************
  Global Variables
*********************************************/

// Switch directions
switch_direction a_direction = DIR_straight_turnout;
switch_direction b_direction = DIR_turnout_straight;

// Hall effect flags
bool hall_b_tripped = false;
bool hall_c_tripped = false;
int16_t hall_b_debounce_count = 0;
int16_t hall_c_debounce_count = 0;
const uint16_t hall_debounce_saturate = 8;

// Comms to ESP
SoftwareSerial aciaSerial(PIN_Rx, PIN_Tx); // RX, TX
bool VB_train_available;

// Mealy machine: throw switches on transitions between states
State_VB last_state = STATE_nominal;
State_VB curr_state = STATE_nominal;
State_VB next_state = STATE_nominal;

// counters
volatile uint16_t counter_100_hz = 0;             // counts 10Hz timer increments
volatile bool counter_new_val_available = false;  // asserted in timer ISR to tell loop that new counter is ready

volatile uint16_t delay_counter = 0;       // hardcoded delay between hall B trigger and switch to inverse
uint16_t delay_counter_max = 500; // hundredths of a second

volatile uint16_t throw_delay_counter = 0;     // short delay to power relay coils long enough to throw switch
uint16_t throw_delay_counter_max = 25; // hundredths of a second, x10 = milliseconds TODO: tune value to make sure switches throw


/********************************************
  Interrupt Handlers
*********************************************/

// highest-frequency clock
#define TIMER_FREQ_HZ 100.0
void HighFrequencyTimerHandler()
{
  // Doing something here inside ISR
  counter_new_val_available = true;
  counter_100_hz++;

  // hardcoded delay between hall B trigger and switch to inverse
  if (curr_state == STATE_delay) {
    delay_counter++;
  }
  else {
    delay_counter = 0;
  }

  // short delay to power relay coils long enough to throw switch
  if (curr_state == STATE_throw_to_inverse || curr_state == STATE_throw_to_nominal) {
    throw_delay_counter++;
  }
  else {
    throw_delay_counter = 0;
  }
}


void I2C_handler() {
  byte TxByte = 0;
  TxByte |= (byte)VB_train_available;
  Wire.write(TxByte);
  // TESTING
  // Serial.println("Sent : " + String(TxByte));
  // VB_train_available = !VB_train_available;
}

#define BIT_VBTA (0x01) // bit 1: vector board says train available
void ACIA_handler() {
  char RxByte = aciaSerial.read();
  VB_train_available = false;
  if (RxByte != -1) {
    VB_train_available = (bool)(RxByte & BIT_VBTA);
  }
}


void hall_b_handler() {
  if (!digitalRead(PIN_HE_B)){
    hall_b_debounce_count++;
    if (hall_b_debounce_count >= hall_debounce_saturate) {
      hall_b_debounce_count = hall_debounce_saturate;
    }
    if (hall_b_debounce_count >= hall_debounce_saturate/2 + 1 ) { // +1 hysteresis
      Serial.println("Hall B tripped!");
      hall_b_tripped = true;
    }
  }
  else {
    hall_b_debounce_count--;
    if (hall_b_debounce_count <= 0) {
      hall_b_debounce_count = 0;
    }
    if (hall_b_debounce_count <= hall_debounce_saturate/2 - 1) { // -1 hysteresis
      hall_b_tripped = false;
    }
  }
}

void hall_c_handler() {
  if (!digitalRead(PIN_HE_C)){
    hall_c_debounce_count++;
    if (hall_c_debounce_count >= hall_debounce_saturate) {
      hall_c_debounce_count = hall_debounce_saturate;
    }
    if (hall_c_debounce_count >= hall_debounce_saturate/2 + 1 ) { // +1 hysteresis
      Serial.println("Hall C tripped!");
      hall_c_tripped = true;
      VB_train_available = false;
    }
  }
  else {
    hall_c_debounce_count--;
    if (hall_c_debounce_count <= 0) {
      hall_c_debounce_count = 0;
    }
    if (hall_c_debounce_count <= hall_debounce_saturate/2 - 1) { // -1 hysteresis
      hall_c_tripped = false;
    }
  }
}


// TODO: deprecated? would be nice to get working
void hall_handler(int PIN, int16_t *debounce_count, bool *trip_flag) {
  if (!digitalRead(PIN)){
    *debounce_count++;
    if (*debounce_count >= hall_debounce_saturate) {
      *debounce_count = hall_debounce_saturate;
    }
    if (*debounce_count >= hall_debounce_saturate/2 + 1 ) { // +1 hysteresis
      *trip_flag = true;
    }
  }
  else {
    *debounce_count--;
    if (*debounce_count <= 0) {
      *debounce_count = 0;
    }
    if (*debounce_count <= hall_debounce_saturate/2 - 1) { // -1 hysteresis
      *trip_flag = false;
    }
  }

}



/********************************************
  Helper Functions
*********************************************/
// TODO: throw switches
void start_throw_switches() {
  digitalWrite(PIN_DIR_A, a_direction);
  digitalWrite(PIN_DIR_B, b_direction);

  delay(10);

  // relays active low
  digitalWrite(PIN_SWITCH_TRIG, LOW);
  
  // testing only
  Serial.println("Starting to throw...");
  Serial.println("A switches: " + String(a_direction));
  Serial.println("B switches: " + String(b_direction));
}
void stop_throw_switches() {
  // relays active low
  digitalWrite(PIN_SWITCH_TRIG, HIGH);
  Serial.println("Stopping throw!");
}

// compute state machine
void update_state() {
  // default condition: keep current state so we don't throw switches by accident
  next_state = curr_state;

  // compute next state
  switch (curr_state) {
    case STATE_nominal:
      if (hall_b_tripped) {
        next_state = STATE_delay;
      }
      break;
    case STATE_delay:
      if (delay_counter >= delay_counter_max) {
        next_state = STATE_throw_to_inverse;
      }
      break;
    case STATE_throw_to_inverse:
      if (throw_delay_counter >= throw_delay_counter_max) {
        next_state = STATE_inverse;
      }
      break;
    case STATE_inverse:
      if (hall_c_tripped) {
        next_state = STATE_throw_to_nominal;
      }
      break;
    case STATE_throw_to_nominal:
      if (throw_delay_counter >= throw_delay_counter_max) {
        next_state = STATE_nominal;
      }
      break;
  }

  last_state = curr_state;
  curr_state = next_state;

  // TODO: testing only
  if (curr_state - last_state != 0) {
    // testing only
    String state_string = curr_state == STATE_nominal ? "nominal" :
                          curr_state == STATE_delay ? "delay" :
                          curr_state == STATE_throw_to_inverse ? "throw to inverse" :
                          curr_state == STATE_inverse ? "inverse" :
                          curr_state == STATE_throw_to_nominal ? "throw to nominal" : "bad state";
    Serial.println("==================\n NEW STATE: \n\t" + state_string + "\n==================\n");
  }

  // output logic, output on transition (Mealy)
  
  // always keep track of which direction to throw switches
  switch (curr_state) {
    case STATE_nominal:
    case STATE_throw_to_nominal:
      a_direction = DIR_straight_turnout;
      b_direction = DIR_turnout_straight;
      break;
    case STATE_inverse:
    case STATE_throw_to_inverse:
    case STATE_delay:
      a_direction = DIR_turnout_straight;
      b_direction = DIR_straight_turnout;
      break;
  }

  // start throwing switches on transition to throw states
  if ( (curr_state == STATE_throw_to_inverse && last_state == STATE_delay) 
    || (curr_state == STATE_throw_to_nominal && last_state == STATE_inverse)) {
    start_throw_switches();
  }
  // stop throwing switches on transition away from throw states
  if ( (last_state == STATE_throw_to_inverse && curr_state == STATE_inverse)
    || (last_state == STATE_throw_to_nominal && curr_state == STATE_nominal)) {
    stop_throw_switches();
  }

}


void setup() {
  pinMode(PIN_Rx, INPUT);
  pinMode(PIN_Tx, OUTPUT);

  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);
  pinMode(PIN_SWITCH_TRIG, OUTPUT);

  // start high
  digitalWrite(PIN_DIR_A, HIGH);
  digitalWrite(PIN_DIR_B, HIGH);
  digitalWrite(PIN_SWITCH_TRIG, HIGH);
  
  pinMode(PIN_HE_B, INPUT);
  pinMode(PIN_HE_C, INPUT);


  Serial.begin(115200); // TESTING ONLY
  aciaSerial.begin(9600);
  Wire.begin(0x87);
  Wire.onRequest(I2C_handler);

  Serial.println("Startup routine, waiting 10 seconds");
  delay(10000);
  Serial.println("Throwing switches to default state");
  start_throw_switches();
  delay(15);
  stop_throw_switches();
  Serial.println("Switches reset!");


  ITimer1.init();
  // Frequency in float Hz
  if (!ITimer1.attachInterrupt(TIMER_FREQ_HZ, HighFrequencyTimerHandler)) {
  //  won't start if timer fails to init
    while (true) {
      start_throw_switches();
      delay(50);
      stop_throw_switches();
      delay(1000);
    }
  }
  // else
  //   Serial.println("Can't set ITimer. Select another freq. or timer");

  VB_train_available = false;
}

void loop() {
  // constantly read hall effects
  // hall_handler(PIN_HE_B, &hall_b_debounce_count, &hall_b_tripped);
  // hall_handler(PIN_HE_C, &hall_c_debounce_count, &hall_c_tripped);
  hall_b_handler();
  hall_c_handler();

  // TESTING: 
  VB_train_available = true;

  // handle base counter
  if (counter_new_val_available) {
    if (counter_100_hz % 100 == 0) {
      Serial.println(".");
    }
    counter_new_val_available = false;  // clear flag!!!!
    update_state(); // also throws switches as needed
  }

  if (aciaSerial.available()) {
    ACIA_handler();
  }
}
