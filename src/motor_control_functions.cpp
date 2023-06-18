#include "motor_control_functions.h"

#define MAX_SPEED                 100  // mm/s
#define MAX_ANGULAR_SPEED         700  // steps/s
#define MICROSTEPS                8
#define STEPS_PER_REV             200
#define MAX_TARGET_DISTANCE       5


// Total steps per revolution = 200 * 16 = 3200

#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port

void setup_driver(TMC2208Stepper &driver, int EN_PIN, int MS1, int MS2) {
  Serial.println("Setting up driver");
  pinMode(EN_PIN, OUTPUT);
  // pinMode(MS1, OUTPUT);
  // pinMode(MS2, OUTPUT);

  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware
  // digitalWrite(MS1, LOW);
  // digitalWrite(MS2, HIGH);
                                  // Enable one according to your setup
//SPI.begin();                    // SPI drivers
  // SERIAL_PORT.begin(115200);      // HW UART drivers
// driver.beginSerial(115200);     // SW UART drivers

  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(600);        // Set motor RMS current
  driver.microsteps(MICROSTEPS);          // Set microsteps to 1/16th

//driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
//driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop
  Serial.println("Done setting up driver");
}


long current_targets[2] = {0, 0};
long initial_positions[2] = {0, 0};
void move_arm(long int * delta, SStepper &motor1, SStepper &motor2, double theta1, double theta2) {
  long int target1 = 3 * theta1 * STEPS_PER_REV * MICROSTEPS/ (2.0*PI);
  long int target2 = 3 * 3 * (theta2 - 2.0 * theta1/3.0) * STEPS_PER_REV * MICROSTEPS / (2.0*PI);

  if(target1 != current_targets[0]) {
    Serial.print("Target 1: ");
    Serial.print(target1);
    Serial.print(" , ");
    Serial.print(theta1);
    Serial.print(". Current pos: ");
    Serial.println(motor1.position * 360.0 /(STEPS_PER_REV * MICROSTEPS * 3));
    initial_positions[0] = current_targets[0];
    current_targets[0] = target1;
  }
  if(target2 != current_targets[1]) {
    Serial.print("Target 2: ");
    Serial.print(target2);
    Serial.print(" , ");
    Serial.print(theta2);
    Serial.print(". Current pos: ");
    Serial.println(motor2.position * 360.0 /(STEPS_PER_REV * MICROSTEPS * 3 * 3) - 2 * motor1.position * 360.0 /(STEPS_PER_REV * MICROSTEPS * 3 * 2));
    initial_positions[1] = current_targets[1];
    current_targets[1] = target2;
  }
  delta[0] = motor1.distance_to_go();
  delta[1] = motor2.distance_to_go();

  if(delta[0] <= 1 && delta[1] <= 1) {
    long max_distance_to_target = max(
                    abs(current_targets[0] - motor1.position),
                    abs(current_targets[1] - motor2.position) );
    if(max_distance_to_target > MAX_TARGET_DISTANCE) {
      long max_original_distance_to_target = max(
                    abs(current_targets[0] - initial_positions[0]),
                    abs(current_targets[1] - initial_positions[1]) );
      float sections_completed = (max_original_distance_to_target - max_distance_to_target) * 1.0 / MAX_TARGET_DISTANCE;
      float target_section = round(sections_completed + 1);
      target1 = initial_positions[0] + MAX_TARGET_DISTANCE * target_section * (current_targets[0] - initial_positions[0])/max_original_distance_to_target;
      target2 = initial_positions[1] + MAX_TARGET_DISTANCE * target_section * (current_targets[1] - initial_positions[1])/max_original_distance_to_target;
    }
    motor1.set_target(target1);
    motor2.set_target(target2);
  }


  if(abs(delta[0]) > abs(delta[1])) {
    motor1.set_speed(MAX_ANGULAR_SPEED * (delta[0] > 0 ? 1 : -1));
    motor2.set_speed( MAX_ANGULAR_SPEED * 1.0 * delta[1] / abs(delta[0]));
  } else {
    motor2.set_speed(2 * MAX_ANGULAR_SPEED * (delta[1] > 0 ? 1 : -1));
    motor1.set_speed(2 * MAX_ANGULAR_SPEED * 1.0 * delta[0] / abs(delta[1]));
  }
  // Serial.print("Speed: ");
  // Serial.print("1: ");
  motor1.one_step();
  // Serial.print(",");
  // Serial.print("2: ");
  motor2.one_step();
  // Serial.println("");

  delta[0] = abs(current_targets[0] - motor1.position);
  delta[1] = abs(current_targets[1] - motor2.position);
}

