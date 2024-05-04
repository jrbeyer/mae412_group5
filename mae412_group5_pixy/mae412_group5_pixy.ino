/********************************************
 *  mae412_group5_pixy.ino
 *  Description: implements wireless data
 *  transfer for the pixy-mounted ESP32
 *
 *  Authors: Jacob Beyer, Vikash Modi, Jae Yoon
 *
 *
 ********************************************/

#include <arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>
#include <I2C_16Bit.h>
#include <SPI.h>
#include <Pixy.h>
#include <VL53L0X.h>

#include "mae412_group5_defines.h"

// defines
#define PIN_LED 15

// sensors
Pixy pixy;
VL53L0X rangefinder;


// message passing
String success;
message_S outbound_message;

// USED FOR DEBUGGING, DON'T REMOVE
// #define DEBUG_NOW

// configure ESP NOW
esp_now_peer_info_t peerInfo;
// callback for data sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  #ifdef DEBUG_NOW
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  #endif
  if (status == 0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}
// Callback when data is received (no incoming data)
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  // inboundCounter = incomingReadings.count;
  // Serial.println("Received: " + String(inboundCounter));
}



void loop_pixycam_update(){
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
    outbound_message.pixy_saw_train = true;
    outbound_message.pixy_train_x = pixy.blocks[0].x;
    outbound_message.pixy_train_y = pixy.blocks[0].y;
  }
  else {
    outbound_message.pixy_saw_train = false;
    Serial.println("PIXYCAM: didn't see a train!");
  }
}


// service 60Hz rangefinder update
void loop_rangefinder_update(){
  // Serial.println("executed rangefinder update, counter: " + String(counter_240_hz));

  outbound_message.rangefinder_range_mm = rangefinder.readRangeContinuousMillimeters();
  // define a good reading with 50 cm
  if (outbound_message.rangefinder_range_mm > 5000 || outbound_message.rangefinder_range_mm == 0) {
    outbound_message.rangefinder_got_range = false;
  }
  else {
    outbound_message.rangefinder_got_range = true;
  }
}


void setup() {
  Wire.begin();
  // Init Serial Monitor
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  delay(5000);

  // Init ESP-NOW
  // slow flash is bad
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
  memcpy(peerInfo.peer_addr, baseESPAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);


  // set up sensors
  pixy.init();
  if (!rangefinder.init()) {
    Serial.println("ERROR: Rangefinder not initialized");
  }
  else {
    rangefinder.startContinuous();
  }


  // Solid light indicates success! One small flash indicates successful message send,
  // five small flashes indicate failure to send
  digitalWrite(PIN_LED, HIGH);
  outbound_message.pixy_saw_train = false;
  outbound_message.pixy_train_x = 0;
  outbound_message.pixy_train_y = 0;
  outbound_message.rangefinder_got_range = 0;
  outbound_message.rangefinder_range_mm = 0;
}


void loop() {
  delay(20); // TODO: make more robust timings
  loop_pixycam_update();
  loop_rangefinder_update();

  Serial.println("Got pixy reading: " + String(outbound_message.pixy_saw_train));
  Serial.println("train x: " + String(outbound_message.pixy_train_x));
  Serial.println("train y: " + String(outbound_message.pixy_train_y));

  Serial.println("Got range reading: " + String(outbound_message.rangefinder_got_range));
  Serial.println("distance (mm): " + String(outbound_message.rangefinder_range_mm));


  // outbound_message modified in above two functions
  esp_err_t result = esp_now_send(baseESPAddress, (uint8_t *) &outbound_message, sizeof(outbound_message));

  if (result == ESP_OK) {
    digitalWrite(PIN_LED, LOW);
    delay(10);
    digitalWrite(PIN_LED, HIGH);
  }
  else {
    int del = 50;
    digitalWrite(PIN_LED, LOW);
    delay(del);
    digitalWrite(PIN_LED, HIGH);
    delay(del);
    digitalWrite(PIN_LED, LOW);
    delay(del);
    digitalWrite(PIN_LED, HIGH);
    delay(del);
    digitalWrite(PIN_LED, LOW);
    delay(del);
    digitalWrite(PIN_LED, HIGH);
    delay(del);
    digitalWrite(PIN_LED, LOW);
    delay(del);
    digitalWrite(PIN_LED, HIGH);
    delay(del);
    digitalWrite(PIN_LED, LOW);
    delay(del);
    digitalWrite(PIN_LED, HIGH);
    delay(del);
  }
}


