/**
 * Author Teemu Mäntykallio
 * Initializes the library and runs the stepper
 * motor in alternating directions.
 */
#include "Arduino.h"
#include "SStepper.h"


SStepper::SStepper(int DIR_PIN, int STEP_PIN, int HOMING_PIN) {
  this->DIR_PIN = DIR_PIN;
  this->STEP_PIN = STEP_PIN;
  this->HOMING_PIN = HOMING_PIN;
  pinMode(this->DIR_PIN, OUTPUT);
  pinMode(this->STEP_PIN, OUTPUT);
  pinMode(HOMING_PIN, INPUT_PULLUP);
}

void SStepper::one_step(int direction, int wait) {
  // direction: 1 for forward, 0 for backward
  digitalWrite(this->DIR_PIN, direction);
  digitalWrite(this->STEP_PIN, HIGH);
  delayMicroseconds(wait);
  digitalWrite(this->STEP_PIN, LOW);
  delayMicroseconds(wait);
}
