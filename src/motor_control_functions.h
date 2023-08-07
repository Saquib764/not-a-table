#ifndef MOTOR_CONTROL_FUNCTIONS_H
#define MOTOR_CONTROL_FUNCTIONS_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <FastLED.h>
#include "FastAccelStepper.h"
#include "arm_model.h"

double mod(double x, double y);

void force_stop();

void set_arm(ArmModel arm);

void reset();
void to_xy(double a1, double a2, double& x, double& y);
int follow_trajectory();
void add_point_to_trajectory(double theta1, double theta2);

#endif