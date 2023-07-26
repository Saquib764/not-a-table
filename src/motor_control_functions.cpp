#include "motor_control_functions.h"

// #define MAX_SPEED                 0.01  // m/s
#define MAX_ANGULAR_SPEED         4000.0  // steps/s
#define MICROSTEPS                16
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
float R = 0.63/2;

// Define constants
const int MAX_SPEED = 200;
const int MAX_ACCELERATION = 3 * MAX_SPEED;
const float ARM = 0.33;

// Define global variables
int curent_target_index = 2;
float current_position[2] = {0, 0};
float current_acceleration[2] = {0, 0};
float current_speed[2] = {0, 0};
float error = 0;

// Define target arrays
float keypoints[5][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
float targets[5][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
float target_speeds[4] = {0, 0, 0, 0};
float target_directions[4][2] = {{1, 1}, {1, 1}, {1, 1}, {1, 1}};
float angles_at_keypoints[4] = {0, 0, 0, 0};
float max_speeds[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
bool should_stop[4] = {false, false, false, false};

double mod(double x, double y) {
  double r = x - y * floor(x/y);
  if(r < 0) {
    r += y;
    r = mod(r, y);
  }
  return r;
}

void home_motor(FastAccelStepper *m, uint8_t homing_pin, int multiplier) {
  const float hall_effect_reference_value = 2000.0;
  const float hall_effect_threshold = 250;
  Serial.println("Homing motor");
  // Run motor in clockwise direction until home position is reached
  m->setSpeedInHz(0.1 * multiplier * MAX_ANGULAR_SPEED);
  m->setAcceleration(MAX_ANGULAR_SPEED / ACCELERATION_TIME);
  double current_theta = m->getCurrentPosition() / (3*K * multiplier);
  current_theta = mod(current_theta, 2 * PI);
  int dir = 1;
  if(current_theta > PI) {
    dir = -1;
  }
  m->moveTo(m->getCurrentPosition() + dir * 3 * K * multiplier * 2 * PI);

  float value = 0.0;
  while(value < hall_effect_threshold) {
    value = abs(hall_effect_reference_value - analogRead(homing_pin));
    if(value > hall_effect_threshold) {
      // Hall effect sensor is triggered, stop motor
      m->stopMove();
      break;
    }
    delay(10);
  }

  //  move motor step by step, record sensor reading 
  int count = 0;
  float max_value = 0;
  int count_at_max = 0;
  while(count < 3000) {
    // move motor one step
    m->moveTo(m->getCurrentPosition() + dir, true);
    count++;
    float avg_value = analogRead(homing_pin);
    for(int i = 1; i < 5; i++) {
      avg_value += analogRead(homing_pin);
      delay(1);
    }
    avg_value /= 5;
    Serial.println(avg_value);
    if(avg_value > max_value) {
      max_value = avg_value;
      count_at_max = count;
    }
    if(abs(hall_effect_reference_value - avg_value) < hall_effect_threshold) {
      // arm out of range, stop motor
      break;
    }
  }
  // move motor back to max value
  m->moveTo(m->getCurrentPosition() - (count - count_at_max) * dir, true);
  
  // reset motor position to 0
  m->setCurrentPosition(0);
  m->moveTo(0);
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

long trajectory[3][2] = {{0, 0}, {0, 0}, {0, 0}};


void home_arm() {
  home_motor(stepper1, homing_pin1, 1);

  home_motor(stepper2, homing_pin2, 3);
  reset();
}

void reset() {
  // Reinitialise trajectory to zero
  for(int i = 0; i < 3; i++) {
    trajectory[i][0] = 0;
    trajectory[i][1] = 0;
  }
}

// double MAX_ACCELERATION = 1000.0;

void to_xy(float a1, float a2, float& x, float& y) {
  x = ARM * cos(a1) + ARM * cos(a1 + a2);
  y = ARM * sin(a1) + ARM * sin(a1 + a2);
}

// Function to follow the trajectory
bool follow_trajectory() {
  if (curent_target_index >= 5) {
    return false;
  }
  current_position[0] = stepper1->getCurrentPosition();
  current_position[1] = stepper2->getCurrentPosition();

  current_speed[0] = stepper1->getCurrentSpeedInMilliHz()/1000.0;
  current_speed[1] = stepper2->getCurrentSpeedInMilliHz() / 1000.0;

  float displacement_to_target[2] = {targets[curent_target_index][0] - current_position[0], targets[curent_target_index][1] - current_position[1]};

  if (displacement_to_target[0] * target_directions[curent_target_index - 1][0] < 4 && displacement_to_target[1] * target_directions[curent_target_index - 1][1] < 4) {
    curent_target_index++;
    return true;
  }

  float distance_to_target[2] = {abs(displacement_to_target[0]) + 0.001, abs(displacement_to_target[1]) + 0.001};

  current_acceleration[0] = MAX_ACCELERATION;
  current_acceleration[1] = MAX_ACCELERATION * distance_to_target[1] / distance_to_target[0];

  float prev_directions[2] = {1, 1};
  prev_directions[0] = current_speed[0] >= 0 ? 1 : -1;
  prev_directions[1] = current_speed[1] >= 0 ? 1 : -1;

  float original_displacement[2] = {targets[curent_target_index][0] - targets[curent_target_index - 1][0], targets[curent_target_index][1] - targets[curent_target_index - 1][1]};

  float error = 0;
  int error_correction_index = 0;
  if (abs(original_displacement[0]) > 10 && abs(original_displacement[1]) > 10 && abs(displacement_to_target[0]) > 10 && abs(displacement_to_target[1]) > 10) {
    // Correct the motor with the smaller distance
    float r1 = abs(1 - displacement_to_target[0] / original_displacement[0]);
    float r2 = abs(1 - displacement_to_target[1] / original_displacement[1]);
    if (r1 < r2) {
      // Motor 1 is faster, slow it down
      error_correction_index = 0;
    } else {
      // Motor 2 is faster, slow it down
      error_correction_index = 1;
    }
    int reference_index = (error_correction_index == 0) ? 1 : 0;
    float expected_position_of_error_index = original_displacement[error_correction_index] * abs(displacement_to_target[reference_index] / original_displacement[reference_index]);

    error = (expected_position_of_error_index - displacement_to_target[error_correction_index]) * target_directions[curent_target_index - 1][error_correction_index];
  }

  float _max_speed[2] = {
    max(151.0f, max_speeds[curent_target_index - 1][0]),
    max(151.0f, max_speeds[curent_target_index - 1][1])
  };

  current_acceleration[0] = (_max_speed[0] * target_directions[curent_target_index - 1][0] - current_speed[0]) * 2;
  current_acceleration[1] = (_max_speed[1] * target_directions[curent_target_index - 1][1] - current_speed[1]) * 2;

  if (error >= 1) {
    current_acceleration[error_correction_index] = -0.2 * current_speed[error_correction_index];
  }

  if (displacement_to_target[0] * target_directions[curent_target_index - 1][0] < 0.3 * abs(current_speed[0]) || displacement_to_target[1] * target_directions[curent_target_index - 1][1] < 0.3 * abs(current_speed[1])) {
    if (abs(current_speed[0]) > 0.3 * _max_speed[0]) {
      current_acceleration[0] = -0.9 * current_speed[0];
    }
    if (abs(current_speed[1]) > 0.3 * _max_speed[1]) {
      current_acceleration[1] = -0.9 * current_speed[1];
    }
  }
  
  EVERY_N_MILLISECONDS(25) {
    // Serial.print("disp: " + String(displacement_to_target[0] * target_directions[curent_target_index - 1][0]));
    // Serial.println(", " + String(displacement_to_target[1] * target_directions[curent_target_index - 1][1]));
    Serial.println("m1: " + String(current_speed[0]) + ", " + String(_max_speed[0]));
    Serial.println("m2: " + String(current_speed[1]) + ", " + String(_max_speed[1]));
  }

  stepper1->setSpeedInTicks((uint32_t)_max_speed[0]);
  stepper2->setSpeedInTicks((uint32_t)_max_speed[1]);

  stepper1->moveByAcceleration(current_acceleration[0], true);
  stepper2->moveByAcceleration(current_acceleration[1], true);
  return false;
}

// Function to add a point to the trajectory
void add_point_to_trajectory(float a1, float a2) {
  float _pt[2];
  to_xy(a1, a2, _pt[0], _pt[1]);
  float _lpt[2];
  to_xy(keypoints[4][0], keypoints[4][1], _lpt[0], _lpt[1]);
  float angle_to_new_point = atan2(_pt[1] - _lpt[1], _pt[0] - _lpt[0]) * 180.0 / PI;
  float target_speed_to_new_point = -1;

  if (abs(angle_to_new_point - angles_at_keypoints[3]) > 20) {
    // Stop
    target_speed_to_new_point = 0;
  }

  // Shift all points to the left
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 2; j++) {
      keypoints[i][j] = keypoints[i + 1][j];
      targets[i][j] = targets[i + 1][j];
      max_speeds[i][j] = max_speeds[i + 1][j];
      if (i < 3) {
        target_directions[i][j] = target_directions[i + 1][j];
      }
    }
  }

  // Add new point to the end
  keypoints[4][0] = a1;
  keypoints[4][1] = a2;

  to_xy(keypoints[4][0], keypoints[4][1], _pt[0], _pt[1]);
  targets[4][0] = int(_pt[0] * K);
  targets[4][1] = int(_pt[1] * K);
  target_speeds[3] = target_speed_to_new_point;

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

  curent_target_index--;

  angles_at_keypoints[3] = angle_to_new_point;

  if (target_directions[3][0] != target_directions[2][0] || target_directions[3][1] != target_directions[2][1]) {
    should_stop[3] = true;
  } else {
    should_stop[3] = target_speed_to_new_point != 0;
  }

  float distance_to_target[2] = {abs(targets[4][0] - targets[3][0]), abs(targets[4][1] - targets[3][1])};
  max_speeds[3][0] = MAX_SPEED * (1.0 * distance_to_target[0]) / (distance_to_target[0] + 0.001);
  max_speeds[3][1] = MAX_SPEED * (1.0 * distance_to_target[1]) / (distance_to_target[1] + 0.001);

  if (max_speeds[3][1] > MAX_SPEED) {
    max_speeds[3][0] = MAX_SPEED * (1.0 * distance_to_target[0]) / (distance_to_target[1] + 0.001);
    max_speeds[3][1] = MAX_SPEED * (1.0 * distance_to_target[1]) / (distance_to_target[1] + 0.001);
  }

  max_speeds[3][0] = ceil(max_speeds[3][0]);
  max_speeds[3][1] = ceil(max_speeds[3][1]);

  if(false) {
    Serial.print("keypoints: ");
    for (int i = 0; i < 5; i++) {
      Serial.print("[" + String(keypoints[i][0]) + ", " + String(keypoints[i][1]) + "] ");
    }
    Serial.println();

    Serial.print("targets: ");
    for (int i = 0; i < 5; i++) {
      Serial.print("[" + String(targets[i][0]) + ", " + String(targets[i][1]) + "] ");
    }
    Serial.println();

    Serial.print("angles_at_keypoints: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(String(angles_at_keypoints[i]) + " ");
    }
    Serial.println();

    Serial.print("targets speed: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(String(target_speeds[i]) + " ");
    }
    Serial.println();

    Serial.print("directions: ");
    for (int i = 0; i < 4; i++) {
      Serial.print("[" + String(target_directions[i][0]) + ", " + String(target_directions[i][1]) + "] ");
    }
    Serial.println();

    Serial.print("max speed: ");
    for (int i = 0; i < 4; i++) {
      Serial.print("[" + String(max_speeds[i][0]) + ", " + String(max_speeds[i][1]) + "] ");
    }
    Serial.println();

    Serial.print("should_stop: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(should_stop[i] + " ");
    }
    Serial.println();

    Serial.print("curent_target_index: ");
    Serial.println(curent_target_index);

    Serial.println();
  }
}

void force_stop() {
  stepper1->forceStop();
  stepper2->forceStop();
}