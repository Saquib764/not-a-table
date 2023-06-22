#ifndef MOTOR_CONTROL_FUNCTIONS_H
#define MOTOR_CONTROL_FUNCTIONS_H

#include <Arduino.h>
#include <TMCStepper.h>
#include "SStepper.h"
#include <FastLED.h>
#include "FastAccelStepper.h"

void setup_driver(TMC2209Stepper &driver, int EN_PIN, int MS1, int MS2);
void setup_arm(uint8_t EN_PIN, uint8_t DIR_1, uint8_t STEPPER_1, uint8_t HOMING_1, uint8_t DIR_2, uint8_t STEPPER_2, uint8_t HOMING_2);
void move_arm(long int * delta, double theta1=0.0, double theta2=0.0);

#endif