/**
 * Author Teemu Mäntykallio
 * Initializes the library and runs the stepper
 * motor in alternating directions.
 */
#include "Arduino.h"
#include "SStepper.h"


SStepper::SStepper(int DIR_PIN, int STEP_PIN) {
  this->DIR_PIN = DIR_PIN;
  this->STEP_PIN = STEP_PIN;
  pinMode(this->DIR_PIN, OUTPUT);
  pinMode(this->STEP_PIN, OUTPUT);
}

void SStepper::one_step(int direction, int wait) {
  digitalWrite(this->DIR_PIN, direction);
  digitalWrite(this->STEP_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(this->STEP_PIN, LOW);
  delayMicroseconds(100);
}
