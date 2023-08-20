#ifndef Sim_Arm_Model_h
#define Sim_Arm_Model_h

#include "../simulator/hardware/motor_model.h"
#include <cmath>

class SimArmModel{
  private:
    /* data */
  public:
    SimArmModel(double ARM, double steps_per_radian);
    
    void setup(uint8_t HOMING_1, uint8_t HOMING_2);
    void setRandomPosition();
    MotorModel* stepper1;
    MotorModel* stepper2;

    double ARM;
    double steps_per_radian;
    void setSpeedInHz(double speed1, double speed2);
    void moveByAcceleration(double acceleration1, double acceleration2);
    void stopMove();

    void getJointPositionInSteps(double* pos);
    void getJointPositionInRadians(double* pos);

    void getJointSpeedInSteps(double* speed);
    void getJointSpeedInRadians(double *speed);

    void getJointAccelerationInSteps(double *acceleration);
    void getJointAccelerationInRadians(double *acceleration);

    // void move();

    bool is_homed[2];
    bool is_hall_sensor_detected;
    double position_at_max_speed;
    double max_hall_value;
    double homing_started_at_angle;
    bool is_homing;
    bool has_started_in_hall_region;

    uint8_t homing_pin1;
    uint8_t homing_pin2;

    bool isHomed();

    void moveToPositionInSteps(double pos1, double pos2);

    void resetToPositionInSteps(double pos1, double pos2);

    // void home();
    void to_xy(double a1, double a2, double& x, double& y);
};

#endif

