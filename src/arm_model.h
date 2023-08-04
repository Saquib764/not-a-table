#ifndef Arm_Model_h
#define Arm_Model_h

class ArmModel{
  private:
    /* data */
  public:
    ArmModel(MotorModel* stepper1, MotorModel* stepper2, double ARM, double steps_per_radian);
    void setRandomPosition();
    MotorModel* stepper1;
    MotorModel* stepper2;

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

    void move();

    bool is_homed[2];

    bool isHomed();

    void moveToPositionInSteps(double pos1, double pos2);

    void resetToPositionInSteps(double pos1, double pos2);
};

#endif

