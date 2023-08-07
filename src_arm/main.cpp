
#include <Arduino.h>
#include "math.h"
#include <TMCStepper.h>
#include <iostream>
#include<array>

using namespace std;

#include "../common/driver_setup.cpp"
#include "../common/arm_model.cpp"
#include "../common/arm_controller.cpp"

#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f
#define VERSION "1.0.0"

// int mode = 1;  // Simple arm move test
// int mode = 2;  // Homing code
int mode = 3;  // Controller code


// TMC2209Stepper driver(&SERIAL_PORT, R_SENSE);
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

int EN_PIN = 25;
uint8_t motor1DirPin = 26;
uint8_t motor1StepPin = 27;
uint8_t motor1HomingPin = 32;


uint8_t motor2DirPin = 14;
uint8_t motor2StepPin = 13;
uint8_t motor2HomingPin = 33;

#define K     STEPS_PER_REV * MICROSTEPS/ (2.0*PI)
#define R     0.63/2


double target_q1 = 0.0;
double target_q2 = 0.0;


ArmModel *arm = new ArmModel(R, K);

ArmController *controller = new ArmController(arm);


void setup() {
  delay(50);

  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Version: " + String(VERSION));


  sleep(1);


  setup_driver(driver, EN_PIN);
  arm->setup(EN_PIN, motor1DirPin, motor1StepPin, motor1HomingPin, motor2DirPin, motor2StepPin, motor2HomingPin);


  if(mode == 1) {
    arm->setSpeedInHz(600, 600);
    arm->moveByAcceleration(100.0, 100.0);
  }
  if(mode == 3) {
    arm->setSpeedInHz(1200, 1200);
  }

  delay(2000);
}

double points[5][3] = {
  {0.0, 0.0},
  {-1.0, 1.0},
  {1.0, 1.5},
  {3.0, 0.0},
  {3.14, 0.0}
};
int current_index = 0;

void loop() {
  // Serial.println("ok");
  if(mode == 2) {
    // Homing code

    if(arm->isHomed()) {
      Serial.println("Arm is homed");
      mode=3;
      return;
    }
    arm->home();
  }
  if(mode == 3) {
    // controller code
    int should_read_next = controller->follow_trajectory();
    // long int delta[2] = {0, 0};
    // bool should_read_next = move_arm(delta, target_q1, target_q2);
    if(should_read_next == 1) {
      if(current_index < 5) {
        double* point = points[current_index];
        current_index = current_index + 1;
        target_q1 = point[0];
        target_q2 = point[1];
      }
      controller->add_point_to_trajectory(target_q1, target_q2);
      // target_q1 = points[current_index][1];
      // target_q2 = points[current_index][2];
      // current_index = (current_index + 1) % 5;
    }
    if(should_read_next == 2) {
      // should_play_next = true;
      target_q1 = 0.0;
      target_q2 = 0.0;
    }
  }
  delay(1);
}
