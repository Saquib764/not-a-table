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
void to_xy(float a1, float a2, float& x, float& y);
bool follow_trajectory();
void add_point_to_trajectory(float theta1, float theta2);

#endif