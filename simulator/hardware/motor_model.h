#ifndef Motor_Model_h
#define Motor_Model_h

#include <cmath>

class MotorModel {
  public:
    // constructors:
    MotorModel();
    void setCurrentPosition(double position);
    void setSpeedInHz(double speed);
    void moveByAcceleration(double acceleration, bool reverse);
    double getCurrentPosition();
    double getCurrentSpeedInMilliHz();
    double getCurrentAcceleration();

    void reset(double pos);
    void moveTo(double pos);
    void stopMove();

    void move();

    double max_speed;

    double current_position;
    double current_speed;
    double current_acceleration;

    double dt;
};

#endif