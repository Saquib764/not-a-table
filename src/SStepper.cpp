/**
 * Author Teemu Mäntykallio
 * Initializes the library and runs the stepper
 * motor in alternating directions.
 */
#include "Arduino.h"
#include "SStepper.h"

#define CLOCKWISE 1
#define COUNTERCLOCKWISE 0


SStepper::SStepper(int DIR_PIN, int STEP_PIN, int HOMING_PIN) {
  this->DIR_PIN = DIR_PIN;
  this->STEP_PIN = STEP_PIN;
  this->HOMING_PIN = HOMING_PIN;
  this->direction = 1;

  // this->stepper = AccelStepper(1, STEP_PIN, DIR_PIN);
  // this->stepper.setPinsInverted(true);
  // this->stepper.setMaxSpeed(1000);
  pinMode(this->DIR_PIN, OUTPUT);
  pinMode(this->STEP_PIN, OUTPUT);
  pinMode(HOMING_PIN, INPUT_PULLUP);

  digitalWrite(this->DIR_PIN, this->direction);
  this->reset();
}
void SStepper::set_acceleration(double acceleration) {
  this->acceleration = acceleration;
}
void SStepper::set_target_speed(double speed) {
  this->target_speed = speed;
}
void SStepper::set_speed(double speed) {
  if(speed == this->speed) {
    return;
  }
  this->speed = speed;
  // this->stepper.setSpeed(speed);
  if(this->speed == 0) {
    this->step_interval = 0;
    return;
  }
  this->step_interval = fabs(1000000.0 / this->speed);
  set_direction(speed > 0?CLOCKWISE:COUNTERCLOCKWISE);
}
void SStepper::compute_speed() {
  if(this->speed == this->target_speed) {
    return;
  }
  unsigned long time = micros();
  long dt = time - this->last_speed_update_time;
  if(dt < 1.0/(this->acceleration + 0.001)) {
    return;
  }
  this->last_speed_update_time = time;
  double speed = this->speed + this->acceleration * dt;
  if(abs(this->target_speed - this->speed) < abs(this->acceleration * dt)) {
    speed = this->target_speed;
  }
  this->set_speed(speed);
}
void SStepper::set_target(long int target) {
  if(target == this->target) {
    return;
  }
  this->target = target;
  // this->stepper.moveTo(target);
  // this->stepper.setSpeed(this->speed);
  // Serial.print("Ta: ");
  // Serial.print(target);
  // Serial.print(" ");
  // Serial.println(this->speed);
}
void SStepper::set_position(long int position) {
  this->position = position;
  // this->stepper.setCurrentPosition(position);
}
void SStepper::set_step_delay(int step_delay) {
  this->step_delay = step_delay;
}
void SStepper::set_direction(int direction) {
  if(direction != this->direction) {
    this->direction = direction;
    digitalWrite(this->DIR_PIN, direction);
  }
}
long int SStepper::distance_to_go() {
  return this->target - this->position;
}

void SStepper::reset() {
  this->position = 0;
  this->target = 0;
  this->speed = 0;
  this->target_speed = 0;
  this->acceleration = 0;
  this->last_step_time = 0;
  this->step_delay = 60;
  this->step_interval = 0;
  this->last_speed_update_time = 0;
}

void SStepper::one_step(int direction, int wait) {
  // direction: 1 for forward, 0 for backward
  if(direction != this->direction) {
    this->direction = direction;
    digitalWrite(this->DIR_PIN, direction);
  }
  digitalWrite(this->STEP_PIN, HIGH);
  delayMicroseconds(wait);
  digitalWrite(this->STEP_PIN, LOW);
  delayMicroseconds(wait);
}
bool SStepper::one_step() {
  this->compute_speed();
  unsigned long time = micros();
  if(!this->step_interval) {
    return false;
  }
  long dt = time - this->last_step_time;
  if( dt < this->step_interval ) {
    return false;
  }
  if(this->position == this->target) {
    return false;
  }
  if(this->direction == CLOCKWISE) {
    this->position++;
  } else {
    this->position--;
  }
  
  // Serial.print(1000000.0 / dt);
  this->last_step_time = time;
  digitalWrite(this->STEP_PIN, HIGH);
  delayMicroseconds(this->step_delay);
  digitalWrite(this->STEP_PIN, LOW);
  delayMicroseconds(10);
  return true;
}
void SStepper::force_step() {
  digitalWrite(this->STEP_PIN, HIGH);
  delayMicroseconds(this->step_delay);
  digitalWrite(this->STEP_PIN, LOW);
  delayMicroseconds(10);
}
