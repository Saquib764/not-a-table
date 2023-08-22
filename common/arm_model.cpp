#include "arm_model.h"

FastAccelStepperEngine engine = FastAccelStepperEngine();

ArmModel::ArmModel(double ARM, double steps_per_radian) {
  this->ARM = ARM;
  this->steps_per_radian = steps_per_radian;
  this->is_homed[0] = false;
  this->is_homed[1] = false;
  this->stepper1 = NULL;
  this->stepper2 = NULL;

  is_hall_sensor_detected = false;
  position_at_max_speed = 0.0;
  max_hall_value = 0.0;
  homing_started_at_angle = 0.0;
  is_homing = false;
  has_started_in_hall_region = false;
}

void ArmModel::setup_joint(FastAccelStepper *stepper, uint8_t EN_PIN, uint8_t DIR) {
  stepper->setDirectionPin(DIR);
  stepper->setEnablePin(EN_PIN);
  stepper->setAutoEnable(true);
}

void ArmModel::setup(uint8_t EN_PIN, uint8_t DIR_1, uint8_t STEPPER_1, uint8_t HOMING_1, uint8_t DIR_2, uint8_t STEPPER_2, uint8_t HOMING_2) {
  engine.init();
  homing_pin1 = HOMING_1;
  homing_pin2 = HOMING_2;
  stepper1 = engine.stepperConnectToPin(STEPPER_1);
  stepper2 = engine.stepperConnectToPin(STEPPER_2);
  setup_joint(stepper1, EN_PIN, DIR_1);
  setup_joint(stepper2, EN_PIN, DIR_2);
  
  setSpeedInHz(1500, 1500);
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
  stepper1->setSpeedInHz(abs(speed1));
  stepper2->setSpeedInHz(abs(3 * speed2 + speed1));
}

void ArmModel::moveByAcceleration(double acceleration1, double acceleration2) {
  stepper1->moveByAcceleration(acceleration1, true);
  stepper2->moveByAcceleration( 3 * acceleration2 + acceleration1, true);
}

void ArmModel::stopMove() {
  stepper1->stopMove();
  stepper2->stopMove();
}

void ArmModel::getJointPositionInSteps(double* pos) {
  // Serial.println("get 1 : " + String( stepper2->getCurrentPosition() ));
  
  // Serial.println("get : " + String(pos[0]));

  pos[0] = stepper1->getCurrentPosition();
  pos[1] = (stepper2->getCurrentPosition() - pos[0]) / 3.0;
  
  // delay(1000);
  // Serial.println("get 2: " + String(pos[0]));

}

void ArmModel::getJointPositionInRadians(double* pos) {
  getJointPositionInSteps(pos);
  pos[0] = pos[0] / (3 * steps_per_radian);
  pos[1] = pos[1] / (3 * steps_per_radian);
}

void ArmModel::getJointSpeedInSteps(double* speed) {
  speed[0] = stepper1->getCurrentSpeedInMilliHz() / 1000.0;
  speed[1] = (stepper2->getCurrentSpeedInMilliHz()/1000.0 - speed[0]) / 3.0;
}

void ArmModel::getJointSpeedInRadians(double *speed) {
  getJointSpeedInSteps(speed);
  speed[0] = speed[0] / (3 * steps_per_radian);
  speed[1] = speed[1] / (3 * steps_per_radian);
}

void ArmModel::getJointAccelerationInSteps(double *acceleration) {
  acceleration[0] = stepper1->getCurrentAcceleration();
  acceleration[1] = (stepper2->getCurrentAcceleration() - acceleration[0]) / 3.0;
}

void ArmModel::getJointAccelerationInRadians(double *acceleration) {
  getJointAccelerationInSteps(acceleration);
  acceleration[0] = acceleration[0] / (3 * steps_per_radian);
  acceleration[1] = acceleration[1] / (3 * steps_per_radian);
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
  stepper2->moveTo( 3 * pos2 + pos1);
}

void ArmModel::resetToPositionInSteps(double pos1, double pos2) {
  stepper1->setCurrentPosition(pos1);
  stepper2->setCurrentPosition( 3 * pos2 + pos1);
  setSpeedInHz(0, 0);
}


void ArmModel::home() {
  if(!is_homed[0]) {
    int homing_pin = homing_pin1;
    double pos[2] = {0, 0};
    double pos_steps[2] = {0, 0};
    getJointPositionInRadians(pos);
    getJointPositionInSteps(pos_steps);

    double value = analogRead(homing_pin) - 2000.0;
    for(int i = 1; i<5; i++) {
      value += analogRead(homing_pin) - 2000.0;
    }
    value /= 5.0;

    if(!is_homing) {
      // start homing
      is_homing = true;
      is_hall_sensor_detected = false;
      position_at_max_speed = 0.0;
      max_hall_value = 0.0;
      if( value > 100.0 ) {
        // get out of hall region
        setSpeedInHz(400.0, 400.0);
        moveByAcceleration(-500.0, 500.0);
        has_started_in_hall_region = true;
      }else{
        setSpeedInHz(400.0, -400.0);
        moveByAcceleration(500.0, -500.0);
      }
    }else if(value < 20 && has_started_in_hall_region) {
      // got out of hall region
      // reverse and restart homing
      has_started_in_hall_region = false;
      is_homing = true;
      is_hall_sensor_detected = false;
      position_at_max_speed = 0.0;
      max_hall_value = 0.0;
      setSpeedInHz(50.0, -50.0);
      moveByAcceleration(500.0, -500.0);
    } else {
      if( value > 100.0 && !is_hall_sensor_detected ) {
        // slow down when hall sensor is detected
        setSpeedInHz(50.0, -50.0);
        is_hall_sensor_detected = true;
        position_at_max_speed = pos_steps[0];
        max_hall_value = value;
        homing_started_at_angle = pos[0];
      }
      if(is_hall_sensor_detected && value > max_hall_value) {
        // update max hall value
        max_hall_value = value;
        position_at_max_speed = pos_steps[0];
      }
      if(is_hall_sensor_detected && abs(pos[0] - homing_started_at_angle) > 10.0 * 3.14 / 180.0) {
        // arm out of hall sensor, return to max value
        stepper1->stopMove();
        stepper2->stopMove();
        
        setSpeedInHz(100.0, -100.0);

        delayMicroseconds(1000);
        stepper1->moveTo(position_at_max_speed, true);
        resetToPositionInSteps(0.0, 0.0);

        delayMicroseconds(1000);
        is_homed[0] = true;
        
        is_hall_sensor_detected = false;
        position_at_max_speed = 0.0;
        max_hall_value = 0.0;
        homing_started_at_angle = 0.0;
        is_homing = false;
        has_started_in_hall_region = false;
      }
    }
    // cout << "pos: " << pos[0] << endl;
  } else if(!is_homed[1]) {
    int homing_pin = homing_pin2;
    double pos[2] = {0, 0};
    double pos_steps[2] = {0, 0};
    getJointPositionInRadians(pos);
    getJointPositionInSteps(pos_steps);

    double value = analogRead(homing_pin) - 2000.0;
    for(int i = 1; i<5; i++) {
      value += analogRead(homing_pin) - 2000.0;
    }
    value /= 5.0;

    if(!is_homing) {
      // start homing
      is_homing = true;
      is_hall_sensor_detected = false;
      position_at_max_speed = 0.0;
      max_hall_value = 0.0;
      if( value > 100.0 ) {
        // get out of hall region
        setSpeedInHz(0.0, 400.0);
        moveByAcceleration(0.0, -500.0);
        has_started_in_hall_region = true;
      }else{
        setSpeedInHz(0.0, 400.0);
        moveByAcceleration(0.0, 500.0);
      }
    }else if(value < 20 && has_started_in_hall_region) {
      // got out of hall region
      // reverse and restart homing
      has_started_in_hall_region = false;
      is_homing = true;
      is_hall_sensor_detected = false;
      position_at_max_speed = 0.0;
      max_hall_value = 0.0;
      setSpeedInHz(0.0, 50.0);
      moveByAcceleration(0.0, 500.0);
    } else {
      if( value > 100.0 && !is_hall_sensor_detected ) {
        // slow down when hall sensor is detected
        setSpeedInHz(0.0, 50.0);
        is_hall_sensor_detected = true;
        position_at_max_speed = pos_steps[1];
        max_hall_value = value;
        homing_started_at_angle = pos[1];
      }
      if(is_hall_sensor_detected && value > max_hall_value) {
        // update max hall value
        max_hall_value = value;
        position_at_max_speed = pos_steps[1];
      }
      if(is_hall_sensor_detected && abs(pos[1] - homing_started_at_angle) > 10.0 * 3.14 / 180.0) {
        // arm out of hall sensor, return to max value
        stepper1->stopMove();
        stepper2->stopMove();

        setSpeedInHz(0.0, 100.0);
        delayMicroseconds(1000);
        stepper2->moveTo(position_at_max_speed, true);
        resetToPositionInSteps(0.0, 0.0);

        delayMicroseconds(1000);
        is_homed[1] = true;
        
        is_hall_sensor_detected = false;
        position_at_max_speed = 0.0;
        max_hall_value = 0.0;
        homing_started_at_angle = 0.0;
        is_homing = false;
        has_started_in_hall_region = false;
      }
    }
    // cout << "pos: " << pos[0] << endl;
  }

}

void ArmModel::to_xy(double a1, double a2, double& x, double& y) {
  x = ARM * cos(a1) + ARM * cos(a1 + a2);
  y = ARM * sin(a1) + ARM * sin(a1 + a2);
}