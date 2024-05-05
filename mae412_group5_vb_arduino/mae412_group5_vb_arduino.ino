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



#include <Wire.h>
#include <SoftwareSerial.h>

#define PIN_Rx          10
#define PIN_Tx          11
#define PIN_DIR_A       5
#define PIN_DIR_B       6
#define PIN_SWITCH_TRIG 7
#define PIN_HE_B        2
#define PIN_HE_C        3



// straight_turnout = top switch straight, bottom switch turnout
// turnout_straight = top switch turnout, bottom switch straight
// where top = 1,2; bottom = 3,4 in diagram :)

// TODO: how to set in order to actually throw in the right direction?
typedef enum switch_direction {
  DIR_straight_turnout = 0,
  DIR_turnout_straight = 1,
};

switch_direction a_direction = DIR_straight_turnout;
switch_direction b_direction = DIR_turnout_straight;

SoftwareSerial aciaSerial(PIN_Rx, PIN_Tx); // RX, TX
bool VB_train_available;

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

}

void hall_c_handler() {
  VB_train_available = false;
}

// TODO: throw switches
void throw_switches() {

}

void setup() {
  pinMode(PIN_Rx, INPUT);
  pinMode(PIN_Tx, OUTPUT);

  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);
  pinMode(PIN_SWITCH_TRIG, OUTPUT);
  
  pinMode(PIN_HE_B, INPUT);
  pinMode(PIN_HE_C, INPUT);

  // Serial.begin(115200);
  aciaSerial.begin(9600);
  Wire.begin(0x87);
  Wire.onRequest(I2C_handler);
  VB_train_available = false;
}

void loop() {
  if (aciaSerial.available()) {
    ACIA_handler();
  }

  if (digitalRead(PIN_HE_B) == HIGH) {
        hall_b_handler();
  }

  // Check if Hall Effect sensor C detected a magnetic field
  if (digitalRead(PIN_HE_C) == HIGH) {
        hall_c_handler();
  }

}
