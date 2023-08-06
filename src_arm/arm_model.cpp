#include "arm_model.h"

FastAccelStepperEngine engine = FastAccelStepperEngine();

ArmModel::ArmModel(double ARM, double steps_per_radian) {
  this->ARM = ARM;
  this->steps_per_radian = steps_per_radian;
  this->is_homed[0] = false;
  this->is_homed[1] = false;
  this->stepper1 = NULL;
  this->stepper2 = NULL;
}

FastAccelStepper* ArmModel::setup_joint(FastAccelStepperEngine engine, uint8_t EN_PIN, uint8_t DIR, uint8_t STEPPER) {
  FastAccelStepper *stepper = NULL;
  stepper = engine.stepperConnectToPin(STEPPER);
  if (stepper) {
    stepper->setDirectionPin(DIR);
    stepper->setEnablePin(EN_PIN);
    stepper->setAutoEnable(true);

    // stepper1->keepRunning();
  }
  return stepper;
}

void ArmModel::setup(uint8_t EN_PIN, uint8_t DIR_1, uint8_t STEPPER_1, uint8_t DIR_2, uint8_t STEPPER_2) {
  engine.init();
  stepper1 = setup_joint(engine, EN_PIN, DIR_1, STEPPER_1);
  stepper2 = setup_joint(engine, EN_PIN, DIR_2, STEPPER_2);
  
  // setSpeedInHz(20000, 20000);
}


void ArmModel::setRandomPosition() {
  srand(time(0));
  // Generate two random numbers between 0 and 1
  double r1 = 2.0 * 3.14 * (double)rand() / (double)RAND_MAX;
  double r2 = 2.0 * 3.14 * (double)rand() / (double)RAND_MAX;

  stepper1->setCurrentPosition(r1 * 3.0 * steps_per_radian);
  stepper2->setCurrentPosition((3.0 * r2 + r1) * 3.0 * steps_per_radian);
}

void ArmModel::setSpeedInHz(double speed1, double speed2) {
  Serial.println("Set speed " + String(speed1));

  if(stepper1) {
    Serial.println("Yes, steppes");
  }
  stepper1->setSpeedInHz(abs(speed1));
  stepper2->setSpeedInHz(abs(speed2 + speed1));
}

void ArmModel::moveByAcceleration(double acceleration1, double acceleration2) {
  Serial.println(12);
  stepper1->moveByAcceleration(acceleration1, true);
  stepper2->moveByAcceleration(acceleration2 + acceleration1, true);
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

// void ArmModel::move() {
//   stepper1->move();
//   stepper2->move();
// }

bool ArmModel::isHomed() {
  return is_homed[0] && is_homed[1];
}

void ArmModel::moveToPositionInSteps(double pos1, double pos2) {
  stepper1->moveTo(pos1);
  stepper2->moveTo(pos2 + pos1);
}

void ArmModel::resetToPositionInSteps(double pos1, double pos2) {
  stepper1->setCurrentPosition(pos1);
  stepper2->setCurrentPosition(pos2 + pos1);
  setSpeedInHz(0, 0);
}