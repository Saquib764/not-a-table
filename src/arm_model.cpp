#include "arm_model.h"

ArmModel::ArmModel(MotorModel* stepper1, MotorModel* stepper2, double ARM, double steps_per_radian) {
  this->stepper1 = stepper1;
  this->stepper2 = stepper2;
  this->ARM = 0.0;
  this->steps_per_radian = steps_per_radian;
  this->is_homed[0] = false;
  this->is_homed[1] = false;

  setSpeedInHz(20000, 20000);
}

void ArmModel::setRandomPosition() {
  srand(time(0));
  // Generate two random numbers between 0 and 1
  cout<< "ArmModel::setRandomPosition" << endl;
  double r1 = 2.0 * 3.14 * (double)rand() / (double)RAND_MAX;
  double r2 = 2.0 * 3.14 * (double)rand() / (double)RAND_MAX;

  cout << "r1: " << r1 << " r2: " << r2 << endl;

  stepper1->setPosition(r1 * 3.0 * steps_per_radian);
  stepper2->setPosition((3.0 * r2 + r1) * 3.0 * steps_per_radian);
}

void ArmModel::setSpeedInHz(double speed1, double speed2) {
  stepper1->setSpeedInHz(speed1);
  stepper2->setSpeedInHz(speed2 + speed1);
}

void ArmModel::moveByAcceleration(double acceleration1, double acceleration2) {
  stepper1->moveByAcceleration(acceleration1);
  stepper2->moveByAcceleration(acceleration2 + acceleration1);
}

void ArmModel::getJointPositionInSteps(double* pos) {
  pos[0] = stepper1->getCurrentPosition();
  pos[1] = stepper2->getCurrentPosition() - pos[0];
}

void ArmModel::getJointPositionInRadians(double* pos) {
  getJointPositionInSteps(pos);
  pos[0] = pos[0] / (3 * steps_per_radian);
  pos[1] = pos[1] / (9 * steps_per_radian);
}

void ArmModel::getJointSpeedInSteps(double* speed) {
  speed[0] = stepper1->getCurrentSpeedInMilliHz() / 1000.0;
  speed[1] = stepper2->getCurrentSpeedInMilliHz()/1000.0 - speed[0];
}

void ArmModel::getJointSpeedInRadians(double *speed) {
  getJointSpeedInSteps(speed);
  speed[0] = speed[0] / (3 * steps_per_radian);
  speed[1] = speed[1] / (9 * steps_per_radian);
}

void ArmModel::getJointAccelerationInSteps(double *acceleration) {
  acceleration[0] = stepper1->getCurrentAcceleration();
  acceleration[1] = stepper2->getCurrentAcceleration() - acceleration[0];
}

void ArmModel::getJointAccelerationInRadians(double *acceleration) {
  getJointAccelerationInSteps(acceleration);
  acceleration[0] = acceleration[0] / (3 * steps_per_radian);
  acceleration[1] = acceleration[1] / (9 * steps_per_radian);
}

void ArmModel::move() {
  stepper1->move();
  stepper2->move();
}

bool ArmModel::isHomed() {
  return is_homed[0] && is_homed[1];
}

void ArmModel::moveToPositionInSteps(double pos1, double pos2) {
  cout << "Home: " << pos1 << " " << pos1 + pos2 << endl;
  stepper1->moveToPositionInSteps(pos1);
  stepper2->moveToPositionInSteps(pos2 + pos1);
}

void ArmModel::resetToPositionInSteps(double pos1, double pos2) {
  stepper1->resetToPositionInSteps(pos1);
  stepper2->resetToPositionInSteps(pos2 + pos1);
  setSpeedInHz(0, 0);
}