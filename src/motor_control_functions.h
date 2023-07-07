#ifndef MOTOR_CONTROL_FUNCTIONS_H
#define MOTOR_CONTROL_FUNCTIONS_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <FastLED.h>
#include "FastAccelStepper.h"

double mod(double x, double y);
void setup_driver(TMC2209Stepper &driver, int EN_PIN);
void home_motor(FastAccelStepper *m, u_int8_t homing_pin, int multiplier);
void home_arm();
void setup_arm(uint8_t EN_PIN, uint8_t DIR_1, uint8_t STEPPER_1, uint8_t HOMING_1, uint8_t DIR_2, uint8_t STEPPER_2, uint8_t HOMING_2);
void force_stop();

void reset();
bool follow_trajectory();
bool add_target_to_trajectory(double theta1, double theta2);

#endif