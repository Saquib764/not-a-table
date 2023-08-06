#ifndef Arm_Model_h
#define Arm_Model_h

#include <Arduino.h>
#include "FastAccelStepper.h"
#include <cmath>

class ArmModel{
  private:
    /* data */
  public:
    ArmModel(double ARM, double steps_per_radian);
    FastAccelStepper* setup_joint(FastAccelStepperEngine engine, uint8_t EN_PIN, uint8_t DIR, uint8_t STEPPER);
    void setup(uint8_t EN_PIN, uint8_t DIR_1, uint8_t STEPPER_1, uint8_t DIR_2, uint8_t STEPPER_2);
    void setRandomPosition();
    FastAccelStepper* stepper1;
    FastAccelStepper* stepper2;

    double ARM;
    double steps_per_radian;
    void setSpeedInHz(double speed1, double speed2);
    void moveByAcceleration(double acceleration1, double acceleration2);

    void getJointPositionInSteps(double* pos);
    void getJointPositionInRadians(double* pos);

    void getJointSpeedInSteps(double* speed);
    void getJointSpeedInRadians(double *speed);

    void getJointAccelerationInSteps(double *acceleration);
    void getJointAccelerationInRadians(double *acceleration);

    // void move();

    bool is_homed[2];

    bool isHomed();

    void moveToPositionInSteps(double pos1, double pos2);

    void resetToPositionInSteps(double pos1, double pos2);
};

#endif

