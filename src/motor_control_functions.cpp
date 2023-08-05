#include "motor_control_functions.h"

// #define MAX_SPEED                 0.01  // m/s
#define MAX_ANGULAR_SPEED         4000.0  // steps/s
#define MICROSTEPS                32
#define STEPS_PER_REV             200
#define MAX_TARGET_DISTANCE       50
#define ACCELERATION_TIME         1.0  // s


// Total steps per revolution = 200 * 16 = 3200

#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;
uint8_t homing_pin1;
uint8_t homing_pin2;
double K = STEPS_PER_REV * MICROSTEPS/ (2.0*PI);
double R = 0.63/2;

ArmModel arm = ArmModel(stepper1, stepper2, R, K);

double mod(double x, double y) {
  double r = x - y * floor(x/y);
  if(r < 0) {
    r += y;
    r = mod(r, y);
  }
  return r;
}

void setup_driver(TMC2209Stepper &driver, int EN_PIN) {
  Serial.println("Setting up driver");
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware
                                  // Enable one according to your setup
//SPI.begin();                    // SPI drivers
  // NEVER increase the baud rate, its adds 10ms delay. probably related to buffer overflow and flush cycle
  SERIAL_PORT.begin(9600);      // HW UART drivers
// driver.beginSerial(115200);     // SW UART drivers

  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(3);                 // Enables driver in software
  // driver.pdn_disable(true);
  driver.rms_current(700);        // Set motor RMS current
  driver.microsteps( MICROSTEPS );          // Set microsteps to 1/16th
  // driver.irun(31);

  driver.intpol(true);               // Interpolate to 256 steps, smooth stepping even with 0 microsteps.

//driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop
  Serial.println("Done setting up driver");
}

void setup_arm(uint8_t EN_PIN, uint8_t DIR_1, uint8_t STEPPER_1, uint8_t HOMING_1, uint8_t DIR_2, uint8_t STEPPER_2, uint8_t HOMING_2) {
  engine.init();
  stepper1 = engine.stepperConnectToPin(STEPPER_1);
  if (stepper1) {
    stepper1->setDirectionPin(DIR_1);
    stepper1->setEnablePin(EN_PIN);
    stepper1->setAutoEnable(true);

    // stepper1->keepRunning();
  }
  stepper2 = engine.stepperConnectToPin(STEPPER_2);
  if (stepper2) {
    stepper2->setDirectionPin(DIR_2);
    stepper2->setEnablePin(EN_PIN);
    stepper2->setAutoEnable(true);

    // stepper2->keepRunning();
  }
  homing_pin1 = HOMING_1;
  homing_pin2 = HOMING_2;
  // Set pins to output
  pinMode(homing_pin1, OUTPUT);
  pinMode(homing_pin2, OUTPUT);
}



// double MAX_ACCELERATION = 1000.0;

void force_stop() {
  stepper1->forceStop();
  stepper2->forceStop();
}

// Define constants
const int MAX_SPEED = 200;
const int MAX_ACCELERATION = 3 * MAX_SPEED;

// Define global variables
int current_target_indexes[2] = {2, 2};

// Define target arrays
double keypoints[5][2] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
double targets[5][2] =   {{0.0, 0.0}, {0., 0.0}, {0., 0.0}, {0., 0.0}, {0., 0.0}};
double target_speeds_dir[4] = {0.0, 0.0, 0.0, 0.0};
double target_directions[4][2] = {{1.0, 1.0}, {1.0, 1.0}, {1.0, 1.0}, {1.0, 1.0}};
double angles_at_keypoints[4] = {0, 0, 0, 0};
double max_speeds[4][2] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
bool should_stop[4] = {false, false, false, false};

double target_speeds[2] = {0.0, 0.0};

double error = 0;
double last_error = 0;
double speed_integral[2] = {0.0, 0.0};

void to_xy(double a1, double a2, double& x, double& y) {
  x = R * cos(a1) + R * cos(a1 + a2);
  y = R * sin(a1) + R * sin(a1 + a2);
}

void getJointPositions(double* pt) {
  pt[0] = stepper1->getCurrentPosition();
  pt[1] = stepper2->getCurrentPosition() - pt[0];
}
void getJointAngles(double* pt) {
  pt[0] = 1.0 * stepper1->getCurrentPosition() / (3*K);
  pt[1] = 1.0 * (stepper2->getCurrentPosition() - stepper1->getCurrentPosition()) / (9*K);
}

void getJointSpeeds(double* v) {
  v[0] = stepper1->getCurrentSpeedInMilliHz() / 1000.0;
  v[1] = stepper2->getCurrentSpeedInMilliHz() / 1000.0 - v[0];
}
void getJointSpeedsAngle(double* v) {
  v[0] = stepper1->getCurrentSpeedInMilliHz() / 1000.0 / (3*K);
  v[1] = (stepper2->getCurrentSpeedInMilliHz() / 1000.0 - stepper1->getCurrentSpeedInMilliHz() / 1000.0 ) / (9*K);
}

void getJointAccelerations(double* a) {
  a[0] = stepper1->getCurrentAcceleration();
  a[1] = stepper2->getCurrentAcceleration() - a[0];
}

// Function to follow the trajectory
int follow_trajectory() {
  if (current_target_indexes[0] >= 5 && current_target_indexes[1] >= 5) {
    return 2;
  }
  // wait for other index to catch up
  if(current_target_indexes[0] >= 5) {
    current_target_indexes[0] = 4;
  }
  if(current_target_indexes[1] >= 5) {
    current_target_indexes[1] = 4;
  }

  int current_target_index = max(current_target_indexes[0], current_target_indexes[1]);

  error = 0;
  double current_acceleration[2] = {0, 0};
  double current_position[2] = {0, 0};
  getJointPositions( current_position );

  double current_speed[2] = {0, 0};
  getJointSpeeds( current_speed );


  double displacement_to_target[2] = {
    targets[current_target_index][0] - current_position[0],
    targets[current_target_index][1] - current_position[1]
  };
  double distance_to_go[2] = {
    displacement_to_target[0] * target_directions[current_target_index - 1][0],
    displacement_to_target[1] * target_directions[current_target_index - 1][1]
  };


  target_speeds[0] = MAX_SPEED * (1.0 * displacement_to_target[0]) / (abs(displacement_to_target[0]) + 0.001);
  target_speeds[1] = MAX_SPEED * (1.0 * displacement_to_target[1]) / (abs(displacement_to_target[0]) + 0.001);

  if (abs(target_speeds[1]) > MAX_SPEED) {
    target_speeds[0] = MAX_SPEED * (1.0 * displacement_to_target[0]) / (abs(displacement_to_target[1]) + 0.001);
    target_speeds[1] = MAX_SPEED * (1.0 * displacement_to_target[1]) / (abs(displacement_to_target[1]) + 0.001);
  }

  target_speeds[0] = max_speeds[current_target_indexes[0] - 1][0];
  target_speeds[1] = max_speeds[current_target_indexes[1] - 1][1];


  bool has_reached[2] = {false, false};
  if (distance_to_go[0] < 2.0 ) {
    has_reached[0] = true;
  }
  if (distance_to_go[1] < 2.0) {
    has_reached[1] = true;
  }

  if (has_reached[0] && has_reached[1]) {
    current_target_indexes[0]++;
    current_target_indexes[1]++;
    return 1;
  }

  double original_displacement[2] = {
    targets[current_target_indexes[0]][0] - targets[current_target_indexes[0] - 1][0],
    targets[current_target_indexes[0]][1] - targets[current_target_indexes[0] - 1][1]
  };

  double speed_adjust[2] = {0., 0.};
  // if(current_target_indexes[0] > current_target_indexes[1]) {
  //   speed_adjust[0] = - 0.3 * current_speed[0];
  //   speed_adjust[1] =   0.3 * current_speed[1];
  // }
  // if(current_target_indexes[0] < current_target_indexes[1]) {
  //   speed_adjust[0] =   0.3 * current_speed[0];
  //   speed_adjust[1] =  -0.3 * current_speed[1];
  // }

  if (abs(original_displacement[0]) > 5) {
    // Correct the motor with the smaller distance
    double r1 = abs((current_position[0] - targets[current_target_indexes[0] - 1][0]) / original_displacement[0]);
    
    double expected_position = targets[current_target_indexes[0] - 1][1] + r1 * original_displacement[1];
    error = expected_position - current_position[1];

    // error = max((double)-10.0, min((double)10., error));

    if( error > 1) {
      speed_adjust[1] = max((double)-10.0, min((double)10., error)) * 0.8;
      last_error = error;
      // speed_adjust[0] = -error * 0.1 ;
      // current_acceleration[1] += speed_adjust[1];
    }else if(error < -1) {
      speed_adjust[0] = -max((double)-10.0, min((double)10., error)) * 0.8 ;
      last_error = error;
    }
  }

  double next_speed[2] = {0.0, 0.0};
  if(current_target_indexes[0] < 4 && current_target_indexes[1] < 4) {
    next_speed[0] = max_speeds[current_target_indexes[0]][0];
    next_speed[1] = max_speeds[current_target_indexes[1]][1];
  }

  double D = 10.0;
  if (distance_to_go[0] < D  && distance_to_go[0] > 0) {
    double t = max(0.0, min(distance_to_go[0] / D, 1.0));
    // target_speeds[0] = target_speeds[0] + (next_speed[0] - target_speeds[0]) * (1 - t) * 0.5  ;
  }

  if ( distance_to_go[1] < D && distance_to_go[1] >0) {
    double t = max(0.0, min(distance_to_go[1] / D, 1.0));
    // target_speeds[1] = target_speeds[1] + (next_speed[1] - target_speeds[1]) * (1 - t)  * 0.5 ;
  }

  if( has_reached[0] ) {
    target_speeds[0] = 0.0;
    speed_adjust[0] = 0.0;
  }
  if( has_reached[1] ) {
    target_speeds[1] = 0.0;
    speed_adjust[1] = 0.0;
  }
  
  speed_integral[0] += target_speeds[0] - current_speed[0];
  speed_integral[1] += target_speeds[1] - current_speed[1];
  speed_integral[0] = max(-10.0, min(10.0, speed_integral[0]));
  speed_integral[1] = max(-10.0, min(10.0, speed_integral[1]));

  // if(abs(target_speeds[0] - current_speed[0]) < 5) {
  //   speed_integral[0] = 0.0;
  // }
  // if(abs(target_speeds[1] - current_speed[1]) < 5) {
  //   speed_integral[1] = 0.0;
  // }
  
  // speed_adjust[0] = 0.0;
  // speed_adjust[1] = 0.0;
  current_acceleration[0] = (target_speeds[0] + speed_adjust[0] - current_speed[0]) * 5.0;
  current_acceleration[1] = (target_speeds[1] + speed_adjust[1] - current_speed[1]) * 5.0;
  
  // Last steps push
  double LD = 40.0;
  if (distance_to_go[0] < LD  && !has_reached[0] && abs(current_speed[0]) < 0.1) {
    current_acceleration[0] = displacement_to_target[0] * 2.0;
  }

  // current_acceleration[0] = max(current_acceleration[0], 0.1);

  // setSpeedInHz( target_speeds[0], target_speeds[1] );
  // setSpeedInHz( abs(_max_speeds[0]), abs(_max_speeds[1]) );

  stepper1->moveByAcceleration(current_acceleration[0]);
  stepper2->moveByAcceleration( current_acceleration[1] + current_acceleration[0] );

  // do nothing, chasing target
  return 0;
}

// Function to add a point to the trajectory
void add_point_to_trajectory(double a1, double a2) {
  double _pt[2];
  to_xy(a1, a2, _pt[0], _pt[1]);
  double _lpt[2];
  to_xy(keypoints[4][0], keypoints[4][1], _lpt[0], _lpt[1]);
  double angle_to_new_point = atan2(_pt[1] - _lpt[1], _pt[0] - _lpt[0]) * 180.0 / PI;
  double target_speed_to_new_point = -1;

  if (abs(angle_to_new_point - angles_at_keypoints[3]) > 20) {
    // Stop
    target_speed_to_new_point = 0;
  }

  // Shift all points to the left
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 2; j++) {
      keypoints[i][j] = keypoints[i + 1][j];
      targets[i][j] = targets[i + 1][j];
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      target_directions[i][j] = target_directions[i + 1][j];
      max_speeds[i][j] = max_speeds[i + 1][j];
    }
    angles_at_keypoints[i] = angles_at_keypoints[i + 1];
    target_speeds_dir[i] = target_speeds_dir[i + 1];
    should_stop[i] = should_stop[i + 1];
  }

  // Add new point to the end
  keypoints[4][0] = a1;
  keypoints[4][1] = a2;

  targets[4][0] = int(3 * a1 * K);
  targets[4][1] = int(9 * a2 * K);
  target_speeds_dir[3] = target_speed_to_new_point;

  if (keypoints[4][0] - keypoints[3][0] > 0) {
    target_directions[3][0] = 1;
  } else if (keypoints[4][0] - keypoints[3][0] < 0) {
    target_directions[3][0] = -1;
  }

  if (keypoints[4][1] - keypoints[3][1] > 0) {
    target_directions[3][1] = 1;
  } else if (keypoints[4][1] - keypoints[3][1] < 0) {
    target_directions[3][1] = -1;
  }

  current_target_indexes[0]--;
  current_target_indexes[1]--;

  angles_at_keypoints[3] = angle_to_new_point;

  if (target_directions[3][0] != target_directions[2][0] || target_directions[3][1] != target_directions[2][1]) {
    should_stop[3] = true;
  } else {
    should_stop[3] = target_speed_to_new_point != 0;
  }

  double displacement_to_target[2] = {
    targets[4][0] - targets[3][0],
    targets[4][1] - targets[3][1]
  };
  max_speeds[3][0] = MAX_SPEED * (1.0 * displacement_to_target[0]) / (abs(displacement_to_target[0]) + 0.001);
  max_speeds[3][1] = MAX_SPEED * (1.0 * displacement_to_target[1]) / (abs(displacement_to_target[0]) + 0.001);

  if (abs(max_speeds[3][1]) > MAX_SPEED) {
    max_speeds[3][0] = MAX_SPEED * (1.0 * displacement_to_target[0]) / (abs(displacement_to_target[1]) + 0.001);
    max_speeds[3][1] = MAX_SPEED * (1.0 * displacement_to_target[1]) / (abs(displacement_to_target[1]) + 0.001);
  }
  double current_position[2] = {0, 0};
  getJointPositions( current_position );
}

void reset() {
  // Reinitialise trajectory to zero
  for(int i = 0; i < 5; i++) {
    targets[i][0] = 0;
    targets[i][1] = 0;
  }
}

bool is_hall_sensor_detected = false;
double position_at_max_speed = 0.0;
double max_hall_value = 0.0;
double homing_started_at_angle = 0.0;
bool is_homing = false;
bool has_started_in_hall_region = false;

void home() {
  if(!arm.is_homed[0]) {
    int homing_pin = homing_pin1;
    double pos[2] = {0, 0};
    double pos_steps[2] = {0, 0};
    arm.getJointPositionInRadians(pos);
    arm.getJointPositionInSteps(pos_steps);

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
        arm.setSpeedInHz(100.0, 100.0);
        arm.moveByAcceleration(-500.0, 500.0);
        has_started_in_hall_region = true;
      }else{
        arm.setSpeedInHz(200.0, -200.0);
        arm.moveByAcceleration(500.0, -500.0);
      }
    }else if(value < 20 && has_started_in_hall_region) {
      // got out of hall region
      // reverse and restart homing
      has_started_in_hall_region = false;
      is_homing = true;
      is_hall_sensor_detected = false;
      position_at_max_speed = 0.0;
      max_hall_value = 0.0;
      arm.setSpeedInHz(200.0, -200.0);
      arm.moveByAcceleration(500.0, -500.0);
    } else {
      if( value > 100.0 && !is_hall_sensor_detected ) {
        // slow down when hall sensor is detected
        arm.setSpeedInHz(50.0, -50.0);
        is_hall_sensor_detected = true;
        position_at_max_speed = pos_steps[0];
        max_hall_value = value;
        homing_started_at_angle = pos[0];
      }
      if(is_hall_sensor_detected && abs(pos[0] - homing_started_at_angle) > 45.0 * 3.14 / 180.0) {
        // arm out of hall sensor, return to max value
        arm.stepper1->forceStop();
        arm.stepper2->forceStop();
        delayMicroseconds(25);
        arm.stepper1->moveTo(position_at_max_speed, true);
        arm.is_homed[0] = true;
        is_hall_sensor_detected = false;
        is_homing = false;
      }
    }
    // cout << "pos: " << pos[0] << endl;
  } else if(!arm.is_homed[1]) {
    int homing_pin = homing_pin2;
    double pos[2] = {0, 0};
    double pos_steps[2] = {0, 0};
    arm.getJointPositionInRadians(pos);
    arm.getJointPositionInSteps(pos_steps);

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
        arm.setSpeedInHz(0.0, -100.0);
        arm.moveByAcceleration(0.0, -500.0);
        has_started_in_hall_region = true;
      }else{
        arm.setSpeedInHz(0.0, 200.0);
        arm.moveByAcceleration(0.0, 500.0);
      }
    }else if(value < 20 && has_started_in_hall_region) {
      // got out of hall region
      // reverse and restart homing
      has_started_in_hall_region = false;
      is_homing = true;
      is_hall_sensor_detected = false;
      position_at_max_speed = 0.0;
      max_hall_value = 0.0;
      arm.setSpeedInHz(0.0, 200.0);
      arm.moveByAcceleration(0.0, 500.0);
    } else {
      if( value > 100.0 && !is_hall_sensor_detected ) {
        // slow down when hall sensor is detected
        arm.setSpeedInHz(0.0, 50.0);
        is_hall_sensor_detected = true;
        position_at_max_speed = pos_steps[1];
        max_hall_value = value;
        homing_started_at_angle = pos[1];
      }
      if(is_hall_sensor_detected && abs(pos[1] - homing_started_at_angle) > 45.0 * 3.14 / 180.0) {
        // arm out of hall sensor, return to max value
        arm.stepper1->forceStop();
        arm.stepper2->forceStop();
        delayMicroseconds(25);
        arm.stepper2->moveTo(position_at_max_speed, true);
        arm.is_homed[1] = true;
        is_hall_sensor_detected = false;
        is_homing = false;
      }
    }
    // cout << "pos: " << pos[0] << endl;
  }

}

bool home_arm() {
  if(arm.isHomed()) {
    reset();
    return true;
  }
  home();
  return false;
}
