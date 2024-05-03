#include <esp_now.h>
#include <WiFi.h>


// REPLACE WITH THE MAC Address of your receiver 
// uint8_t broadcastAddress[] = {0x80, 0x65, 0x99, 0x49, 0x64, 0xEC};
uint8_t broadcastAddress[] = {0x80, 0x65, 0x99, 0x4A, 0x10, 0x08};

// Variable to store if sending data was successful
String success;
int inboundCounter;
int outboundCounter;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int count;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message outgoing;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  inboundCounter = incomingReadings.count;
  Serial.println("Received: " + String(inboundCounter));
}
 
void setup() {
  outboundCounter = 0;
  // outboundCounter = 100;
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  delay(1000);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  outboundCounter++;
  outgoing.count = outboundCounter;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  delay(10000);
}