/********************************************
  UNIT TESTING
*********************************************/

#ifndef MAE412_UTESTS_H
#define MAE412_UTESTS_H

// tests go here...

void utest_loop_pixycam_update() {
  Serial.println("Beginning loop_pixycam_update test...");

  // TEST 1: Train in front of camera
  Serial.println("\tPlace a train in front of pixycam!");
  delay(500);

  loop_pixycam_update();
  if (pixy_train_x==0 || pixy_train_y==0) {
    Serial.println("\tTEST FAILED: Pixycam didn't see train!");
    return;
  }
  else {
    Serial.println("\tGot (x, y) = (" + String(pixy_train_x) + ", " + String(pixy_train_y) + ")");
  }


  // TEST 2: No train in front of camera
  Serial.println("\tRemove train from in front of pixycam!");
  delay(500);

  loop_pixycam_update();
  Serial.println("\tIf message \"PIXYCAM: didn't see a train!\" didn't appear, then test failed!");
}

void utest_loop_rangefinder_update() {
  Serial.println("Beginning loop_rangefinder_update test...");
  init_ADC();
  Serial.println("\tPlace object 10cm from rangefinder, then send a byte!");
  while (!Serial.available()) {}
  Serial.read(); // flush

  loop_rangefinder_update();
  Serial.println("\tGot raw reading = " + String(distance_sensor_raw));
  Serial.println("\tGot distance    = " + String(distance_train));

}

void utest_execute_PID() {
  Serial.println("Beginning loop_position_update test...");
  Serial.println("Setting error to 1.0...");
  track_yaw_params.curr_error = 1.0;
  for (int i = 0; i < 100; i++) {
    execute_PID(&track_yaw_params, &track_yaw, i);
    delay((long)CONTROL_PERIOD * 1000);
  }
  
  Serial.println("Setting error to 0.0...");
  track_yaw_params.curr_error = 0.0;
  for (int i = 0; i < 100; i++) {
    execute_PID(&track_yaw_params, &track_yaw, i);
    delay((long)CONTROL_PERIOD * 1000);
  }

  Serial.println("Setting error to -0.5...");
  track_yaw_params.curr_error = -0.5;
  for (int i = 0; i < 100; i++) {
    execute_PID(&track_yaw_params, &track_yaw, i);
    delay((long)CONTROL_PERIOD * 1000);
  }

  Serial.println("Resetting PID parameters...");
  reset_PID(&track_yaw_params);
  for (int i = 0; i < 100; i++) {
    execute_PID(&track_yaw_params, &track_yaw, i);
    delay((long)CONTROL_PERIOD * 1000);
  }  
}

void utest_stepper_motor() {
  Serial.println("Stepper motor unit tests...");
  delay(4000);
  int i;
  int max = 20;
  int step = 1;
  int del = 50;

  // delay testing
  if (1) {
    max = 40;
    for (del = 6; del >= 2; del -= 2) {
      delay(1000);
      step = 1;
      for (i = 0; i < max; i++) {
        Serial.println("Delay: " + String(del) + "\t step: " + String(step) + "\t" + String(i));
        track_yaw.move(step);
        delay(del);
      }
      for (i = 0; i < max; i++) {
        Serial.println("Delay: " + String(del) + "\t step: " + String(-step) + "\t" + String(i));
        track_yaw.move(-step);
        delay(del);
      }
      delay(1000);
      step = 5;
      for (i = 0; i < max; i++) {
        Serial.println("Delay: " + String(del) + "\t step: " + String(step) + "\t" + String(i));
        track_yaw.move(step);
        delay(del);
      }
      for (i = 0; i < max; i++) {
        Serial.println("Delay: " + String(del) + "\t step: " + String(-step) + "\t" + String(i));
        track_yaw.move(-step);
        delay(del);
      }
      delay(1000);
      step = 10;
      for (i = 0; i < max; i++) {
        Serial.println("Delay: " + String(del) + "\t step: " + String(step) + "\t" + String(i));
        track_yaw.move(step);
        delay(del);
      }
      for (i = 0; i < max; i++) {
        Serial.println("Delay: " + String(del) + "\t step: " + String(-step) + "\t" + String(i));
        track_yaw.move(-step);
        delay(del);
      }
    }
  }

  // open loop positioning test
  if (0){
  for (i = 0; i < max; i++) {
    Serial.println("step: " + String(step) + "\t" + String(i));
    track_yaw.move(step);
    delay(del);
  }
  for (i = 0; i < max; i++) {
    Serial.println("step: " + String(-step) + "\t" + String(i));
    track_yaw.move(-step);
    delay(del);
  }
  delay(1000);
  step = 2;  
  for (i = 0; i < max; i++) {
    Serial.println("step: " + String(step) + "\t" + String(i));
    track_yaw.move(step);
    delay(del);
  }
    for (i = 0; i < max; i++) {
    Serial.println("step: " + String(-step) + "\t" + String(i));
    track_yaw.move(-step);
    delay(del);
  }
  delay(1000);
  step = 5;  
  for (i = 0; i < max; i++) {
    Serial.println("step: " + String(step) + "\t" + String(i));
    track_yaw.move(step);
    delay(del);
  }
    for (i = 0; i < max; i++) {
    Serial.println("step: " + String(-step) + "\t" + String(i));
    track_yaw.move(-step);
    delay(del);
  }
  delay(1000);
  step = 5;  
  max = 40;
  for (i = 0; i < max; i++) {
    Serial.println("step: " + String(step) + "\t" + String(i));
    track_yaw.move(step);
    delay(del);
  }
    for (i = 0; i < max; i++) {
    Serial.println("step: " + String(-step) + "\t" + String(i));
    track_yaw.move(-step);
    delay(del);
  }
  delay(1000);
  step = 5;  
  max = 40;
  del = 10;
  for (i = 0; i < max; i++) {
    Serial.println("step: " + String(-step) + "\t" + String(i));
    track_yaw.move(-step);
    delay(del);
  }
  for (i = 0; i < max; i++) {
    Serial.println("step: " + String(step) + "\t" + String(i));
    track_yaw.move(step);
    delay(del);
  }
  }

  delay(100);
}

void utest_receive_esp_now() {
  Serial.println("Starting ESP NOW unit test...");
  while (true) {
    if (inbound_message.pixy_saw_train) {
      Serial.println("Got train! clearing this flag...");
      inbound_message.pixy_saw_train = false;

      Serial.println("train_x: " + String(inbound_message.pixy_train_x));
      Serial.println("train_y: " + String(inbound_message.pixy_train_y));
    }
    if (inbound_message.rangefinder_got_range) {
      Serial.println("Got distance! clearing this flag...");
      inbound_message.rangefinder_got_range = false;

      Serial.println("distance (mm): " + String(inbound_message.rangefinder_range_mm));
    }
  }
}

void utest_loop_position_update() {
  Serial.println("Beginning loop_position_update test...");
}


void calibrate_rangefinder() {

}


#endif
