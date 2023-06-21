/*
 * Define and control single stepper
 *
 * 
*/

// ensure this library description is only included once
#ifndef SStepper_h
#define SStepper_h
#include "AccelStepper.h"

class SStepper {
  public:
    // constructors:
    SStepper(int DIR_PIN, int STEP_PIN, int HOMING_PIN);

    void one_step(int direction, int wait=20);
    bool one_step();
    void force_step();
    void set_acceleration(double acceleration);
    void set_target_speed(double speed);
    void set_speed(double speed);
    void compute_speed();
    void set_target(long int target);
    void set_position(long int position);
    void set_step_delay(int step_delay);
    void set_direction(int direction);
    long int distance_to_go();
    void reset();

    AccelStepper stepper;

    int direction;
    int DIR_PIN;
    int STEP_PIN;
    int HOMING_PIN;
    long int position;
    long int target;
    double speed;
    double target_speed;
    unsigned int last_speed_update_time;
    double acceleration;
    unsigned int last_step_time;
    unsigned int step_delay;
    double step_interval;
};
#endif