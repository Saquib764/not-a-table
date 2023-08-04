#ifndef Motor_Model_h
#define Motor_Model_h

#include <cmath>

class MotorModel {
  public:
    // constructors:
    MotorModel();
    void setPosition(double position);
    void setSpeedInHz(double speed);
    void moveByAcceleration(double acceleration);
    double getCurrentPosition();
    double getCurrentSpeedInMilliHz();
    double getCurrentAcceleration();

    void resetToPositionInSteps(double pos);
    void moveToPositionInSteps(double pos);

    void move();

    double max_speed;

    double current_position;
    double current_speed;
    double current_acceleration;

    double dt;
};

#endif