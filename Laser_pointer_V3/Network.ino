// 2 Axis Robotic laser/torch camera Pointer
// JJROBOTS
// License: GPL v2
// Network functions (ESP module)
// And message functions (messages from the APP)

int ESPwait(String stopstr, int timeout_secs)
{
  String response;
  bool found = false;
  char c;
  long timer_init;
  long timer;

  timer_init = millis();
  while (!found) {
    timer = millis();
    if (((timer - timer_init) / 1000) > timeout_secs) { // Timeout?
      SerialUSB.println("!Timeout!");
      return 0;  // timeout
    }
    if (Serial1.available()) {
      c = Serial1.read();
      SerialUSB.print(c);
      response += c;
      if (response.endsWith(stopstr)) {
        found = true;
        delay(10);
        Serial1.flush();
        SerialUSB.println();
      }
    } // end Serial1_available()
  } // end while (!found)
  return 1;
}

// getMacAddress from ESP wifi module
int ESPgetMac()
{
  char c1, c2;
  bool timeout = false;
  long timer_init;
  long timer;
  uint8_t state = 0;
  uint8_t index = 0;

  timer_init = millis();
  while (!timeout) {
    timer = millis();
    if (((timer - timer_init) / 1000) > 5) // Timeout?
      timeout = true;
    if (Serial1.available()) {
      c2 = c1;
      c1 = Serial1.read();
      SerialUSB.print(c1);
      switch (state) {
        case 0:
          if (c1 == ':')
            state = 1;
          break;
        case 1:
          if (c1 == '\r') {
            MAC.toUpperCase();
            state = 2;
          }
          else {
            if ((c1 != '"') && (c1 != ':'))
              MAC += c1;  // Uppercase
          }
          break;
        case 2:
          if ((c2 == 'O') && (c1 == 'K')) {
            SerialUSB.println();
            Serial1.flush();
            return 1;  // Ok
          }
          break;
      } // end switch
    } // Serial_available
  } // while (!timeout)
  SerialUSB.println("!Timeout!");
  Serial1.flush();
  return -1;  // timeout
}

int ESPsendCommand(String command, String stopstr, int timeout_secs)
{
  Serial1.println(command);
  ESPwait(stopstr, timeout_secs);
  delay(250);
}

int32_t ExtractParamInt4b(uint8_t pos) {
  union {
    unsigned char Buff[4];
    int32_t d;
  } u;
  u.Buff[0] = (unsigned char)MsgBuffer[pos + 3];
  u.Buff[1] = (unsigned char)MsgBuffer[pos + 2];
  u.Buff[2] = (unsigned char)MsgBuffer[pos + 1];
  u.Buff[3] = (unsigned char)MsgBuffer[pos];
  return (u.d);
}

int16_t ExtractParamInt2b(uint8_t pos) {
  union {
    unsigned char Buff[2];
    int16_t d;
  } u;
  u.Buff[0] = (unsigned char)MsgBuffer[pos + 1];
  u.Buff[1] = (unsigned char)MsgBuffer[pos];
  return (u.d);
}

// 014500,-04500,,,
int32_t ExtractParamString6b(uint8_t pos) {
  
  char Buff[7];

  for (uint8_t i=0;i<6;i++)
    Buff[i] = (char)MsgBuffer[pos + i];
  Buff[6] = 0;
  //SerialUSB.println(Buff);
  return atoi(Buff);
}

// Messgase: 8 channels (16 bits)
void MsgRead()
{
  uint8_t i;
  // New bytes available to process?
  long t0 = micros();
  // Max 5ms reading messages...
  while ((Serial1.available() > 0)&&((micros()-t0)<5000)) {
    // We rotate the Buffer (we could implement a ring buffer in future)
    for (i = 0; i < (MSGMAXLEN - 1); i++) {
      MsgBuffer[i] = MsgBuffer[i + 1];
    }
    MsgBuffer[MSGMAXLEN - 1] = (unsigned char)Serial1.read();
    //SerialUSB.print((char)MsgBuffer[MSGMAXLEN-1]);
    ParseMsg(1);
  }
}

// Read messages from USB
// Message: 8 channels (16 bits)
void USBMsgRead()
{
  uint8_t i;
  // New bytes available to process?
  while (SerialUSB.available() > 0) {
    // We rotate the Buffer (we could implement a ring buffer in future)
    for (i = 0; i < (MSGMAXLEN - 1); i++) {
      MsgBuffer[i] = MsgBuffer[i + 1];
    }
    MsgBuffer[MSGMAXLEN - 1] = (unsigned char)SerialUSB.read();
    //SerialUSB.print((char)MsgBuffer[MSGMAXLEN-1]);
    ParseMsg(0);
  }
}

void ParseMsg(uint8_t interface)
{
  // Message JJAH: Hello Message (This is a presentation message when the API connect to the robot) => Enable WIFI output messages (if the message comes from a wifi interface)
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'H')) {
    SerialUSB.println("->MSG: JJAH: HELLO!");
    newMessage = 0; // No message to proccess on main code...
    working = false;
    if (interface == 1)
      enable_udp_output = true;
  }

  // Message JJAT: Manual Serial
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'T')) {
    SerialUSB.print("->MSG: JJAT:");
    iCH1 = ExtractParamString6b(4);  // axis1
    iCH2 = ExtractParamString6b(11);  // axis2
    iCH3 = 0;
    iCH4 = 0;
    iCH5 = 0;
    iCH6 = 0;
    iCH7 = 0;
    iCH8 = 0;
    SerialUSB.print(iCH1);
    SerialUSB.print(" ");
    SerialUSB.println(iCH2);
    mode = 1;
    newMessage = 1;
    working = true; // Work to do...
    if (interface == 1)
      enable_udp_output = true;
  }

  // Message JJAM: Manual control mode: Angle 
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'M')) {
    SerialUSB.print("->MSG: JJAM:");
    iCH1 = ExtractParamInt2b(4);  // axis1
    iCH2 = ExtractParamInt2b(6);  // axis2
    iCH3 = ExtractParamInt2b(8);  // axis3 (reserved for future)
    iCH4 = ExtractParamInt2b(10); // servo1
    iCH5 = ExtractParamInt2b(12); // servo2
    iCH6 = ExtractParamInt2b(14);
    iCH7 = ExtractParamInt2b(16);
    iCH8 = ExtractParamInt2b(18);
    SerialUSB.print(iCH1);
    SerialUSB.print(" ");
    SerialUSB.print(iCH2);
    SerialUSB.print(" ");
    SerialUSB.println(iCH3);
    mode = 1;
    newMessage = 1;
    working = true; // Work to do...
    if (interface == 1)
      enable_udp_output = true;
  }
  
  // Setup message "JJAS" Set robot speed and acc
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'S')) {
    SerialUSB.print("->MSG: JJAS:");
    iCH1 = ExtractParamInt2b(4);
    iCH2 = ExtractParamInt2b(6);
    iCH3 = ExtractParamInt2b(8);
    iCH4 = ExtractParamInt2b(10);
    iCH5 = ExtractParamInt2b(12);  // Trajectory speed (default=20) More speed->less accuracy
    SerialUSB.print(" SPEED XY:");
    SerialUSB.print(iCH1);
    SerialUSB.print(" SPEED Z:");
    SerialUSB.print(iCH2);
    SerialUSB.print(" ACC XY:");
    SerialUSB.print(iCH3);
    SerialUSB.print("ACC Z:");
    SerialUSB.print(iCH4);
    SerialUSB.print(" TRAJ S:");
    SerialUSB.print(iCH5);
    
   
    

    configSpeed((MAX_SPEED_M1 * float(iCH1)) / 100.0, (MAX_SPEED_M2 * float(iCH1)) / 100.0);
    configAcceleration((MAX_ACCEL_M1 * float(iCH3)) / 100.0, (MAX_ACCEL_M2 * float(iCH3)) / 100.0);
    if (interface == 1)
      enable_udp_output = true;
  }

  // robot motors calibration message "JJAC" 
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'C')) {
    SerialUSB.println("->MSG: JJAC:");
    working = 1;
    newMessage = 1;
    mode = 5;        // Calibration mode
    if (interface == 1)
      enable_udp_output = true;
  }

  // Emergency stop message "JJAE" Stops the robot and disble motors
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'E')) {
    SerialUSB.println("->MSG: JJAE:");
    working = 1;
    newMessage = 1;
    mode = 4;        // Emergency stop mode
    if (interface == 1)
      enable_udp_output = true;
  }
}


void debugMsg()
{
  SerialUSB.print("->mode:");
  SerialUSB.print(mode);
  SerialUSB.print(" CH:");
  SerialUSB.print(iCH1);
  SerialUSB.print(" ");
  SerialUSB.print(iCH2);
  SerialUSB.print(" ");
  SerialUSB.print(iCH3);
  SerialUSB.print(" ");
  SerialUSB.print(iCH4);
  SerialUSB.print(" ");
  SerialUSB.print(iCH5);
  SerialUSB.print(" ");
  SerialUSB.print(iCH6);
  SerialUSB.print(" ");
  SerialUSB.print(iCH7);
  SerialUSB.print(" ");
  SerialUSB.println(iCH8);
}
