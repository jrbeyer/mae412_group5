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


#include <Wire.h>

bool VB_train_available;

void I2C_handler() {
  byte TxByte = 0;
  TxByte |= (byte)VB_train_available;
  Wire.write(TxByte);
  // TESTING
  Serial.println("Sent : " + String(TxByte));
  VB_train_available = !VB_train_available;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(0x87);
  Wire.onRequest(I2C_handler);
  VB_train_available = false;
}

void loop() {
  delay(1000);
  Serial.println("alive");
}
