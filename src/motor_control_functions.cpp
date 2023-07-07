#include "motor_control_functions.h"

#define MAX_SPEED                 0.01  // m/s
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

double MAX_ACCELERATION = 1000.0;
double dx = 0.00001;

bool follow_trajectory() {
  long current_positions[2] = {stepper1->getCurrentPosition(), stepper2->getCurrentPosition()};

  long distance_to_targets[2] = {
    abs(trajectory[1][0] - current_positions[0]),
    abs(trajectory[1][1] - current_positions[1])
  };
  long displacement_to_targets[2] = {
    trajectory[1][0] - current_positions[0],
    trajectory[1][1] - current_positions[1]
  };

  if( displacement_to_targets[0] < 1 && displacement_to_targets[1] < 1 ) {
    // need next point
    return true;
  }

  /* Move the end affector at constant speed towards the target.
   * The speed and acceleration of each arm is proportional to the 
   * distance to the target for both arm respectively
   * Control the arm using accelration only (no jerk).
   * Compute target speed at each iteration to minimize error in trajectory
   * There is a maximum speed and acceleration for each arm.
   * The arms move at maximum possible speed, with sooth transitions between the speeds.
   * Arms must stop where trajectory is not smooth, i.e non-diffrentialble.
  */

  double theta1 = current_positions[0] / (3 * K);
  double theta2 = current_positions[1] / (9 * K) - theta1/3.0;
  double max_speeds[2] = {
    min( 18.0 * MAX_SPEED * K / (R * (5*abs(cos(theta2/2)) + 1)), MAX_ANGULAR_SPEED),
    min( 18.0 * MAX_SPEED * K / R, MAX_ANGULAR_SPEED)
  };

  long initial_displacement_to_targets[2] = {
    trajectory[1][0] - trajectory[0][0],
    trajectory[1][1] - trajectory[0][1]
  };
  long next_segment_displacement_to_targets[2] = {
    trajectory[2][0] - trajectory[1][0],
    trajectory[2][1] - trajectory[1][1]
  };
  long should_stop_at_target[2] = {
    (initial_displacement_to_targets[0] * next_segment_displacement_to_targets[0] < 0) ? 1 : 0,
    (initial_displacement_to_targets[1] * next_segment_displacement_to_targets[1] < 0) ? 1 : 0
  };
  long initial_distance_to_targets[2] = {
    abs(initial_displacement_to_targets[0]),
    abs(initial_displacement_to_targets[1])
  };

  double desired_speeds[2] = {
    max_speeds[0],
    max_speeds[0] * (initial_distance_to_targets[1] / (initial_distance_to_targets[0] + dx))
  };
  if(desired_speeds[1] > max_speeds[1]) {
    desired_speeds[0] = max_speeds[1] * (initial_distance_to_targets[0] / (initial_distance_to_targets[1] + dx));
    desired_speeds[1] = max_speeds[1];
  }
  // if(should_stop_at_target[0]) {
  //   desired_speeds[0] = 0;
  // }
  // if(should_stop_at_target[1]) {
  //   desired_speeds[1] = 0;
  // }
  double desired_accelerations[2] = {
    MAX_ACCELERATION,
    MAX_ACCELERATION * (initial_distance_to_targets[1] / (initial_distance_to_targets[0] + dx))
  };
  if(desired_accelerations[1] > MAX_ACCELERATION) {
    desired_accelerations[0] = MAX_ACCELERATION * (initial_distance_to_targets[0] / (initial_distance_to_targets[1] + dx));
    desired_accelerations[1] = MAX_ACCELERATION;
  }

  double braking_distance[2] = {
    (desired_speeds[0] * desired_speeds[0]) / (2 * desired_accelerations[0]),
    (desired_speeds[1] * desired_speeds[1]) / (2 * desired_accelerations[1])
  };
  boolean should_brake = false;
  int acceleration_directions[2] = {
    initial_displacement_to_targets[0] > 0 ? 1 : -1,
    initial_displacement_to_targets[1] > 0 ? 1 : -1
  };
  if(braking_distance[0] > distance_to_targets[0]) {
    should_brake = true;
  }

  // Set stepper speed
  stepper1->setSpeedInHz(desired_speeds[0]);
  stepper2->setSpeedInHz(desired_speeds[1]);

  // Reach speed by applying acceleration
  // if(should_stop_at_target[0]) {
  //   acceleration_directions[0] = -1;
  // }
  // if(should_stop_at_target[1]) {
  //   acceleration_directions[1] = -1;
  // }
  
  EVERY_N_MILLISECONDS(2000) {
    Serial.println("Speeds: " + String(desired_speeds[0]) + ", " + String(desired_speeds[1]));
    Serial.println("Accel: " + String(desired_accelerations[0]) + ", " + String(desired_accelerations[1]));
    Serial.println("Dir: " + String(acceleration_directions[0]) + ", " + String(acceleration_directions[1]));
    Serial.println("Displ: " + String(displacement_to_targets[0]) + ", " + String(displacement_to_targets[1]));
    Serial.println("Braking dist: " + String(braking_distance[0]) + ", " + String(braking_distance[1]));
    Serial.println(" ");
  }
  stepper1->moveByAcceleration(desired_accelerations[0] * acceleration_directions[0], false);
  stepper2->moveByAcceleration(desired_accelerations[1] * acceleration_directions[1], false);

  if(displacement_to_targets[0] < 1) {
    stepper1->forceStop();
  }
  if(displacement_to_targets[1] < 1) {
    stepper2->forceStop();
  }


  return false;
}

bool add_target_to_trajectory(double theta1, double theta2) {
  Serial.println("Add: " + String(theta1) + ", " + String(theta2));
  trajectory[0][0] = trajectory[1][0];
  trajectory[0][1] = trajectory[1][1];
  trajectory[1][0] = trajectory[2][0];
  trajectory[1][1] = trajectory[2][1];
  trajectory[2][0] = 3 * theta1 * K;
  trajectory[2][1] = 3 * 3 * (theta2 + theta1/3.0) * K;
  return true;
}

void force_stop() {
  stepper1->forceStop();
  stepper2->forceStop();
}