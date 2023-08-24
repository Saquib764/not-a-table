
#ifndef Arm_Controller_h
#define Arm_Controller_h

#include <Arduino.h>
#include "arm_model.h"
#include <cmath> 
#include "../common/utils.cpp"

// #define MAX_SPEED                 0.01  // m/s
#define MAX_ANGULAR_SPEED         4000.0  // steps/s
#define MICROSTEPS                32
#define STEPS_PER_REV             200.0
#define MAX_TARGET_DISTANCE       50
#define ACCELERATION_TIME         1.0  // s


int MAX_POINTS = 10;
int tracking_index = 4;

class ArmController{
  private:
    /* data */
  public:
    ArmController(ArmModel *arm);
    ArmModel* arm;

    double mod(double x, double y);

    void force_stop();

    void reset();
    
    int follow_trajectory();
    void add_point_to_trajectory(double a1, double a2);

    void get_goal(double *goal);
    int get_current_target_index(double t);
    void get_target_speed(double t, double *speeds);
    void get_target_position(double t, double *target_positions);
    void get_target_acceleration(double t, double *accelerations);

    // Define constants
    int MAX_SPEED;
    int MAX_ACCELERATION;

    bool has_started;
    bool has_finished;
    bool has_all_targets;

    // Define global variables
    int current_target_indexes[2];

    // Define target arrays
    double keypoints[10][2];
    double targets[10][2];
    double time_at_keypoints[10];
    double time_to_target[9];
    double target_speeds_dir[9];
    double target_directions[9][2];
    double angles_at_keypoints[9];
    double max_speeds[9][2];
    bool should_stop[9];

    double target_speeds[2];
    double start_time;

    double error[2];
    double tracing_error;
};

#endif