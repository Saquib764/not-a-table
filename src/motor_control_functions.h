#ifndef MOTOR_CONTROL_FUNCTIONS_H
#define MOTOR_CONTROL_FUNCTIONS_H

#include <Arduino.h>
#include <TMCStepper.h>
#include "SStepper.h"

void setup_driver(TMC2209Stepper &driver, int EN_PIN, int MS1, int MS2);
void move_stepper(SStepper &motor, double theta);
void move_arm(double delta[2], SStepper &motor1, SStepper &motor2, double theta1=0.0, double theta2=0.0);

#endif