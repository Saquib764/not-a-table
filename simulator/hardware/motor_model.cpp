
#include "motor_model.h"


MotorModel::MotorModel() {
  // initialize this instance's variables
  max_speed = 0.0;
  current_position = 0.0;
  current_speed = 0.0;
  current_acceleration = 0.0;
  dt = 0.005;
}

void MotorModel::setCurrentPosition(double position) {
  current_position = position * 1000.0;
}

void MotorModel::setSpeedInHz(double speed) {
  max_speed = std::abs(speed * 1000);
}

void MotorModel::moveByAcceleration(double acceleration, bool reverse) {
  current_acceleration = acceleration;
}

double MotorModel::getCurrentPosition() {
  return current_position / 1000.0;
}

double MotorModel::getCurrentSpeedInMilliHz() {
  return current_speed;
}

double MotorModel::getCurrentAcceleration() {
  return current_acceleration;
}

void MotorModel::move() {
  current_speed += current_acceleration * dt * 1000.0;
  if(current_speed > max_speed) {
    current_speed = max_speed;
  }
  if(current_speed < -max_speed) {
    current_speed = -max_speed;
  }
  current_speed = round(current_speed);
  current_position += round(current_speed * dt);
}

void MotorModel::reset(double pos) {
  current_position = pos;
  current_speed = 0.0;
  current_acceleration = 0.0;
}

void MotorModel::moveTo(double pos) {
  current_position = pos;
  current_speed = 0.0;
  current_acceleration = 0.0;
}