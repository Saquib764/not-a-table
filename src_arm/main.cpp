
#include <Arduino.h>
#include <HardwareSerial.h>
#include "math.h"
#include <TMCStepper.h>
#include <iostream>
#include<array>

using namespace std;

#include "../common/driver_setup.cpp"
#include "../common/arm_model.cpp"
#include "../common/arm_controller.cpp"

// #define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
HardwareSerial mySerial(1);

#define R_SENSE 0.11f
#define VERSION "1.0.0"

// int mode = 1;  // Simple arm move test
// int mode = 2;  // Homing code
int mode = 3;  // Controller code

// TMC2209Stepper driver(&SERIAL_PORT, R_SENSE);
TMC2209Stepper driver(&mySerial, R_SENSE, DRIVER_ADDRESS);

int EN_PIN = 7;
#define RXD_PIN 17 // RX pin for UART1
#define TXD_PIN 18 // TX pin for UART1

uint8_t motor1DirPin = 35;
uint8_t motor1StepPin = 36;
uint8_t motor1HomingPin = 10;


uint8_t motor2DirPin = 37;
uint8_t motor2StepPin = 38;
uint8_t motor2HomingPin = 9;


#define K     STEPS_PER_REV * MICROSTEPS/ (2.0*PI)
#define ARM     0.63/2


double target_q1 = 0.0;
double target_q2 = 0.0;


ArmModel *arm = new ArmModel(ARM, K);

ArmController *controller = new ArmController(arm);


void setup() {
  delay(50);

  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Version: " + String(VERSION));

  sleep(1);

  mySerial.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN);
  mySerial.begin(115200);
  setup_driver(driver, EN_PIN);
  arm->setup(EN_PIN, motor1DirPin, motor1StepPin, motor1HomingPin, motor2DirPin, motor2StepPin, motor2HomingPin);


  if(mode == 1) {
    arm->setSpeedInHz(600, 600);
    // arm->moveByAcceleration(100.0, 100.0);
    Serial.println("Steps: " + String(K * 2 * PI) + ", Microstepping: " + String(MICROSTEPS)+ ", STEPS_PER_REV: " + String(STEPS_PER_REV));
    arm->moveToPositionInSteps(K * 2 * PI, 0);
  }
  if(mode == 3) {
    arm->setSpeedInHz(1200, 1200);
  }

  delay(2000);
}

double points[10][2] = {
  {0.0, 0.0},
  {2*3.14, 0.0},
  {2*3.14, 2*3.14},
  {2*3.14, 2*3.14},
  {2*3.14, 2*3.14},
  {2*3.14, 2*3.14},
  {2*3.14, 2*3.14},
  {2*3.14, 2*3.14},
  {2*3.14, 0},
  {2*3.14, 2*3.14},
};
int current_index = 0;

void loop() {
  // Serial.println("ok");
  if(mode == 2) {
    // Homing code

    if(arm->isHomed()) {
      Serial.println("Arm is homed");
      controller->reset();
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
      if(current_index < 10) {
        double* point = points[current_index];
        current_index = current_index + 1;
        target_q1 = point[0];
        target_q2 = point[1];
        controller->add_point_to_trajectory(target_q1, target_q2);
      }else{
        controller->has_all_targets = true;
      }
      // target_q1 = points[current_index][1];
      // target_q2 = points[current_index][2];
      // current_index = (current_index + 1) % 5;
    }
    if(should_read_next == 2) {
      // should_play_next = true;
      target_q1 = 0.0;
      target_q2 = 0.0;
      Serial.println("Stop design print.");
    }
  }
  delay(1);
}
