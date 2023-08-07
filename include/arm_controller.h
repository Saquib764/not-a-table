
#ifndef Arm_Controller_h
#define Arm_Controller_h

#include <Arduino.h>
#include "arm_model.h"
#include <cmath> 

// #define MAX_SPEED                 0.01  // m/s
#define MAX_ANGULAR_SPEED         4000.0  // steps/s
#define MICROSTEPS                32
#define STEPS_PER_REV             200
#define MAX_TARGET_DISTANCE       50
#define ACCELERATION_TIME         1.0  // s

#define K     STEPS_PER_REV * MICROSTEPS/ (2.0*PI)
#define R     0.63/2

class ArmController{
  private:
    /* data */
  public:
    ArmController(ArmModel arm);
    ArmModel* arm;

    double mod(double x, double y);

    void force_stop();

    void reset();
    
    int follow_trajectory();
    void add_point_to_trajectory(double a1, double a2);

    // Define constants
    int MAX_SPEED = 200;
    int MAX_ACCELERATION = 3 * MAX_SPEED;

    // Define global variables
    int current_target_indexes[2];

    // Define target arrays
    double keypoints[5][2];
    double targets[5][2];
    double target_speeds_dir[4];
    double target_directions[4][2];
    double angles_at_keypoints[4];
    double max_speeds[4][2];
    bool should_stop[4];

    double target_speeds[2];

    double error;
    double last_error;
    double speed_integral[2];
};

#endif