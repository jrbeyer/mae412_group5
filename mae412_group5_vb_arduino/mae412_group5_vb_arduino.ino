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
 // 27              A4 (ESP 8)              SDA (i2c)
 // 28              A5 (ESP 9)              SCL (i2c)
 // 16              10                      Serial Rx (ACIA)
 // 17              11                      Serial Tx (ACIA) TODO: PULLUP RESISTOR!!!!!!!!


#include <Wire.h>
#include <SoftwareSerial.h>

#define PIN_Rx 10
#define PIN_Tx 11

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

void setup() {
  pinMode(PIN_Rx, INPUT);
  pinMode(PIN_Tx, OUTPUT);
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
}
