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
  this->direction = 1;
  pinMode(this->DIR_PIN, OUTPUT);
  pinMode(this->STEP_PIN, OUTPUT);
  pinMode(HOMING_PIN, INPUT_PULLUP);

  digitalWrite(this->DIR_PIN, this->direction);
}

void SStepper::one_step(int direction, int wait) {
  // direction: 1 for forward, 0 for backward
  if(direction != this->direction) {
    digitalWrite(this->DIR_PIN, direction);
  }
  digitalWrite(this->STEP_PIN, HIGH);
  delayMicroseconds(wait);
  digitalWrite(this->STEP_PIN, LOW);
  delayMicroseconds(wait);
}
