#ifndef Driver_Setup_H
#define Driver_Setup_H

#include <Arduino.h>
#include <TMCStepper.h>
#include "FastAccelStepper.h"

// #define MAX_SPEED                 0.01  // m/s
#define MAX_ANGULAR_SPEED         4000.0  // steps/s
#define MICROSTEPS                32
#define MAX_TARGET_DISTANCE       50
#define ACCELERATION_TIME         1.0  // s


// Total steps per revolution = 200 * 16 = 3200

// #define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port

void setup_driver(TMC2209Stepper &driver, int EN_PIN);


#endif