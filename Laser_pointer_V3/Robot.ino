
// STEPPERS MOTOR CONTROL AND KINEMATICS
// SPEED, ACCELERATION AND POSITION CONTROL using M0 internal timers for best accuracy

// We control the speed of the motors with interrupts (TC3,TC5 and TCC2) tested up to 32Khz.
// The position,speed and acc of the motor is controlled at 1Khz (called in the main loop)

// MOTOR1
// TC5 interrupt
void TC5_Handler (void)
{
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
  if (dir_M1 == 0)
    return;
  REG_PORT_OUTSET0 = PORT_PA15; // STEP Motor1
  position_M1 += dir_M1;
  delayMicroseconds(1);
  REG_PORT_OUTCLR0 = PORT_PA15; // STEP Motor1
}

// MOTOR2
// TC3 interrupt
void TC3_Handler (void)
{
  TC3->COUNT16.INTFLAG.bit.MC0 = 1; // Interrupt reset
  if (dir_M2 == 0)
    return;
  REG_PORT_OUTSET0 = PORT_PA21; // STEP Motor2
  position_M2 += dir_M2;
  delayMicroseconds(1);
  REG_PORT_OUTCLR0 = PORT_PA21; // STEP Motor2
}


//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.
void timersConfigure()
{
  // First we need to enable and configure the Generic Clock register
  // Enable GCLK for TC4, TC5, TCC2 and TC3 (timer counter input clock) GCLK_CLKCTRL_ID(GCM_TC4_TC5)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC2_TC3));
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Configure Timer1
  TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
  while (TC3->COUNT16.CTRLA.bit.SWRST);

  // Set Timer counter Mode to 16 bits
  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  // Set TC5 mode as match frequency
  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  //set prescaler and enable TC5
  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16 | TC_CTRLA_ENABLE;  // preescaler 16 48Mhz=>3Mhz
  //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
  TC3->COUNT16.CC[0].reg = (uint16_t) MINIMUN_TIMER_PERIOD;

  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

  // Configure interrupt request
  NVIC_DisableIRQ(TC3_IRQn);
  NVIC_ClearPendingIRQ(TC3_IRQn);
  NVIC_SetPriority(TC3_IRQn, 0);
  NVIC_EnableIRQ(TC3_IRQn);

  // Enable interrupt request
  TC3->COUNT16.INTENSET.bit.MC0 = 1;
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); //wait until syncing

  // Configure Timer2 on TC5
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
  while (TC5->COUNT16.CTRLA.bit.SWRST);

  // Set Timer counter Mode to 16 bits
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  // Set TC5 mode as match frequency
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  //set prescaler and enable TC5
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16 | TC_CTRLA_ENABLE;  // preescaler 16 48Mhz=>3Mhz
  //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
  TC5->COUNT16.CC[0].reg = (uint16_t) MINIMUN_TIMER_PERIOD;

  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); // wait for sync...

  // Configure interrupt request
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable interrupt request
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); //wait until syncing


}

// This function enables Timers TC3 and TC5 and waits for it to be ready
void timersStart()
{
  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); //wait until snyc'd
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); //wait until snyc'd
  //TCC2->CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  //while (TCC2->STATUS.reg & TC_STATUS_SYNCBUSY); //wait until snyc'd
}

//Reset timers TC3 and TC5
void timersReset()
{
  TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
  while (TC3->COUNT16.CTRLA.bit.SWRST);
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

// Disable timers TC3 and TC5
void timersDisable()
{
  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
}

// Experimental! S curve implementation on robot starts from 0 or stops
int16_t adjust_acc_S_curve(int16_t speed, int16_t target_acc)
{
  int16_t absspeed;
  absspeed = abs(speed);
  if (absspeed < 400)
    return (map(absspeed, 0, 400, 5, target_acc));
  else
    return target_acc;
}


// ******************************** //
// POSITION AND SPEED ROBOT CONTROL
// We use a ramp for acceleration and deceleration
// To calculate the point we should start to decelerate we use this formula:
// stop_position = actual_posicion + (actual_speed*actual_speed)/(2*max_deceleration)
// Input parameters:
//    target_position_x
//    target_speed_x
//    max_acceleration_x
void positionControl(int dt)
{
  //int16_t pos_stop;
  int32_t temp;

  //dt = constrain(dt, 500, 4000); // Limit dt (it should be around 1000 most times)
  acceleration_M1 = target_acceleration_M1;
  acceleration_M2 = target_acceleration_M2;
  
  // MOTOR1 control
  // We first check if we need to start decelerating in order to stop at the final position
  // calculate the distance to decelerate until stop at target speed
  // kinematic formula   d = (sqr(v1)-sqr(v0))/(2*a) because v1=0 and acc(a) is negative in deceleration => d = sqr(v1)/2a
  // We calculate the stop position and check if we need to start decelerating right now...
  temp = (long)speed_M1 * speed_M1;
  temp = temp / (2000 * (long)acceleration_M1);
  pos_stop_M1 = position_M1 + sign(speed_M1) * temp;
  //acceleration_M1 = adjust_acc_S_curve(speed_M1,target_acceleration_M1);
  if (target_position_M1 > position_M1) { // Positive move
    if (pos_stop_M1 >= target_position_M1) { // Start decelerating?
      int16_t overshoot = pos_stop_M1 - target_position_M1;
      if (overshoot > overshoot_compensation) //SerialUSB.println("OV1");
        setMotorM1Speed(0, dt, overshoot / overshoot_compensation); // Increase decelaration a bit (overshoot compensation)
      M1stopping = true;
      setMotorM1Speed(0, dt, 0);        // The deceleration ramp is done inside the setSpeed function
    }
    else {
      M1stopping = false;
      setMotorM1Speed(target_speed_M1, dt, 0);  // The aceleration ramp is done inside the setSpeed function
    }
  }
  else {  // Negative move
    if (pos_stop_M1 <= target_position_M1) { // Start decelerating?
      int16_t overshoot = target_position_M1 - pos_stop_M1;
      if (overshoot > overshoot_compensation) //SerialUSB.println("OV2");
        setMotorM1Speed(0, dt, overshoot / overshoot_compensation); // Increase decelaration a bit (overshoot compensation)
      M1stopping = true;
      //adjust_acc_S_curve();
      setMotorM1Speed(0, dt, 0);
    }
    else {
      M1stopping = false;
      setMotorM1Speed(-target_speed_M1, dt, 0);
    }
  }

  // MOTOR2 CONTROL
  temp = (long)speed_M2 * speed_M2;
  temp = temp / (2000 * (long)acceleration_M2);
  pos_stop_M2 = position_M2 + sign(speed_M2) * temp;
  //acceleration_M2 = adjust_acc_S_curve(speed_M2,target_acceleration_M2);
  if (target_position_M2 > position_M2) // Positive move
  {
    if (pos_stop_M2 >= target_position_M2) { // Start decelerating?
      int16_t overshoot = pos_stop_M2 - target_position_M2;
      if (overshoot > overshoot_compensation) //SerialUSB.println("OV3");
        setMotorM2Speed(0, dt, overshoot / overshoot_compensation); // Increase decelaration a bit (overshoot compensation)
      M2stopping = true;
      setMotorM2Speed(0, dt, 0);        // The deceleration ramp is done inside the setSpeed function
    }
    else {
      M2stopping = false;
      setMotorM2Speed(target_speed_M2, dt, 0);  // The aceleration ramp is done inside the setSpeed function
    }
  }
  else   // Negative move
  {
    if (pos_stop_M2 <= target_position_M2) { // Start decelerating?
      int16_t overshoot = target_position_M2 - pos_stop_M2;
      if (overshoot > overshoot_compensation) //SerialUSB.prointln("OV4");
        setMotorM2Speed(0, dt, overshoot / overshoot_compensation); // Increase decelaration a bit (overshoot compensation)
      M2stopping = true;
      setMotorM2Speed(0, dt, 0);
    }
    else {
      M2stopping = false;
      setMotorM2Speed(-target_speed_M2, dt, 0);
    }
  }

  
  if ((dir_M1 == 0) && (dir_M2 == 0))
    working = false;   // Robot is stopped
  else
    working = true;    // Robot is moving...
  //CLR(PORTF,3); // for external timing debug

}

// Speed could be positive or negative
void setMotorM1Speed(int16_t tspeed, int16_t dt, int16_t overshoot_comp)
{
  long timer_period;
  int16_t accel;

  // Limit max speed
  tspeed = constrain(tspeed, -MAX_SPEED_M1, MAX_SPEED_M1);

  // We limit acceleration => speed ramp
  overshoot_comp = constrain(overshoot_comp, -10, 10);
  accel = (((long)acceleration_M1 * dt) / 1000) + overshoot_comp;
  if (((long)tspeed - speed_M1) > accel)
    speed_M1 += accel;
  else if (((long)speed_M1 - tspeed) > accel)
    speed_M1 -= accel;
  else {
    //if (speed_M1!=tspeed)
    //  SerialUSB.println("->M1FS");
    speed_M1 = tspeed;
  }

  // Check if we need to change the direction pins
  if ((speed_M1 == 0) && (dir_M1 != 0)) {
    dir_M1 = 0;
    //SerialUSB.println("->M1 STOP");
  }
  else if ((speed_M1 > 0) && (dir_M1 != 1)) {
#ifdef INVERT_M1_AXIS
    REG_PORT_OUTCLR0 = PORT_PA20;
#else
    REG_PORT_OUTSET0 = PORT_PA20;
#endif
    dir_M1 = 1;
  }
  else if ((speed_M1 < 0) && (dir_M1 != -1)) {
#ifdef INVERT_M1_AXIS
    REG_PORT_OUTSET0 = PORT_PA20;
#else
    REG_PORT_OUTCLR0 = PORT_PA20;
#endif
    dir_M1 = -1;
  }

  if (speed_M1 == 0)
    timer_period = MINIMUN_TIMER_PERIOD;
  else if (speed_M1 > 0)
    timer_period = 3000000 / speed_M1;   // 3Mhz timer
  else
    timer_period = 3000000 / -speed_M1;

  if (timer_period > MINIMUN_TIMER_PERIOD)   // Check for minimun speed (maximun period without overflow)
    timer_period = MINIMUN_TIMER_PERIOD;

  // Change timer
  TC5->COUNT16.CC[0].reg = (uint16_t) timer_period;
  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); // wait for sync...

  // Check  if we need to reset the timer...
  if (TC5->COUNT16.COUNT.reg > (uint16_t)timer_period) {
    TC5->COUNT16.COUNT.reg = (uint16_t)timer_period - 4;
    while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); // wait for sync...
  }
}

// Speed could be positive or negative
void setMotorM2Speed(int16_t tspeed, int16_t dt, int16_t overshoot_comp)
{
  long timer_period;
  int16_t accel;

  tspeed = constrain(tspeed, -MAX_SPEED_M2, MAX_SPEED_M2); // Limit max speed

  // We limit acceleration => speed ramp
  overshoot_comp = constrain(overshoot_comp, -10, 10);
  accel = (((long)acceleration_M2 * dt) / 1000) + overshoot_comp; // We divide by 1000 because dt are in microseconds
  if (((long)tspeed - speed_M2) > accel) // We use long here to avoid overflow on the operation
    speed_M2 += accel;
  else if (((long)speed_M2 - tspeed) > accel)
    speed_M2 -= accel;
  else {
    //if (speed_M2!=tspeed)
    //  SerialUSB.println("->M2FS");
    speed_M2 = tspeed;
  }

  // Check if we need to change the direction pins
  if ((speed_M2 == 0) && (dir_M2 != 0)) {
    dir_M2 = 0;
    //SerialUSB.println("->M2 STOP");
  }
  else if ((speed_M2 > 0) && (dir_M2 != 1)) {
#ifdef INVERT_M2_AXIS
    REG_PORT_OUTCLR0 = PORT_PA06; // M2-DIR
#else
    REG_PORT_OUTSET0 = PORT_PA06;
#endif
    dir_M2 = 1;
  }
  else if ((speed_M2 < 0) && (dir_M2 != -1)) {
#ifdef INVERT_M2_AXIS
    REG_PORT_OUTSET0 = PORT_PA06;  // M2-DIR
#else
    REG_PORT_OUTCLR0 = PORT_PA06;
#endif
    dir_M2 = -1;
  }
  if (speed_M2 == 0)
    timer_period = MINIMUN_TIMER_PERIOD;
  else if (speed_M2 > 0)
    timer_period = 3000000 / speed_M2; // 3Mhz timer (48Mhz / preescaler=16 = 3Mhz)
  else
    timer_period = 3000000 / -speed_M2;
  if (timer_period > MINIMUN_TIMER_PERIOD)   // Check for minimun speed (maximun period without overflow)
    timer_period = MINIMUN_TIMER_PERIOD;

  // Change timer
  TC3->COUNT16.CC[0].reg = (uint16_t) timer_period;
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); // wait for sync...

  // Check  if we need to reset the timer...
  if (TC3->COUNT16.COUNT.reg > (uint16_t)timer_period) {
    TC3->COUNT16.COUNT.reg = (uint16_t)timer_period - 4;
    while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); // wait for sync...
  }
}



// Set speed in steps/sec
void configSpeed(int target_sM1, int target_sM2)
{
  target_sM1 = constrain(target_sM1, 0, MAX_SPEED_M1);
  target_sM2 = constrain(target_sM2, 0, MAX_SPEED_M2);
 
  config_speed_M1 = target_sM1;
  config_speed_M2 = target_sM2;
  
}

// Set acceleration
void configAcceleration(int target_acc1, int target_acc2)
{
  target_acc1 = constrain(target_acc1, MIN_ACCEL_M1, MAX_ACCEL_M1);
  target_acc2 = constrain(target_acc2, MIN_ACCEL_M2, MAX_ACCEL_M2);
  
  config_acceleration_M1 = target_acc1;
  config_acceleration_M2 = target_acc2;
  
}

// Set the max speed and acceleration for a movement from config values
void setSpeedAcc()
{
  // Set speed...
  target_speed_M1 = config_speed_M1;
  target_speed_M2 = config_speed_M2;
  
  // Set accelerations...
  target_acceleration_M1 = config_acceleration_M1;
  target_acceleration_M2 = config_acceleration_M2;
  
}


void motorsCalibration()
{


  // Move servos
  moveServo1(SERVO1_MIN_PULSEWIDTH);
  delay(600);
  moveServo1(SERVO1_MAX_PULSEWIDTH);
  delay(600);
  moveServo1(SERVO1_NEUTRAL);
  delay(600);
  moveServo2(SERVO2_MIN_PULSEWIDTH);
  delay(600);
  moveServo2(SERVO2_NEUTRAL);
}

// Set Robot Axis1:
void setAxis1(float angleA1)
{
  angleA1 = constrain(angleA1, ROBOT_MIN_A1, ROBOT_MAX_A1);
  target_position_M1 = angleA1 * M1_AXIS_STEPS_PER_UNIT;
  SerialUSB.print("A1:");
  SerialUSB.print(angleA1);
  SerialUSB.print(",");
  SerialUSB.println(target_position_M1);
}

// Set Robot Axis2:
void setAxis2(float angleA2)
{
  angleA2 = constrain(angleA2, ROBOT_MIN_A2, ROBOT_MAX_A2);
  target_position_M2 = angleA2 * M2_AXIS_STEPS_PER_UNIT;
  SerialUSB.print("A2:");
  SerialUSB.print(angleA2);
  SerialUSB.print(",");
  SerialUSB.println(target_position_M2);
}


// Servos functions...
void initServo()
{
  servo1.attach(3);
  servo2.attach(4);
  servo1_ready = true;
  servo2_ready = true;
}

void disableServo1()
{
  servo1_ready = false;
  servo1.detach();
}

void disableServo2()
{
  servo2_ready = false;
  servo2.detach();
}

void enableServo1()
{
  servo1_ready = true;
  servo1.attach(3);
}

void enableServo2()
{
  servo2_ready = true;
  servo2.attach(4);
}


// move servo1 on OC4B (pin10)
void moveServo1(int pwm)
{
  pwm = constrain(pwm, SERVO1_MIN_PULSEWIDTH, SERVO1_MAX_PULSEWIDTH);
  servo1.writeMicroseconds(pwm);
}

// move servo2 on OC4A (pin13)
void moveServo2(int pwm)
{
  pwm = constrain(pwm, SERVO2_MIN_PULSEWIDTH, SERVO2_MAX_PULSEWIDTH);
  servo2.writeMicroseconds(pwm);
}
