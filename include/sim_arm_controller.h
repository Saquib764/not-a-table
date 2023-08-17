
#ifndef Sim_Arm_Controller_h
#define Sim_Arm_Controller_h

#include "sim_arm_model.h"
#include <cmath> 

#define PI    3.14159265358979323846
// #define MAX_SPEED                 0.01  // m/s
#define MAX_ANGULAR_SPEED         4000.0  // steps/s
#define MICROSTEPS                32
#define STEPS_PER_REV             200.0
#define MAX_TARGET_DISTANCE       50
#define ACCELERATION_TIME         1.0  // s

class SimArmController{
  private:
    /* data */
  public:
    SimArmController(SimArmModel *arm);
    SimArmModel* arm;

    double mod(double x, double y);

    void reset();
    
    int follow_trajectory();
    void add_point_to_trajectory(double a1, double a2);

    // Define constants
    int MAX_SPEED;
    int MAX_ACCELERATION;

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