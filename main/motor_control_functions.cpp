#include "motor_control_functions.h"

#define MAX_SPEED         50
#define MICROSTEPS        64
#define STEPS_PER_REV     200


// Total steps per revolution = 200 * 64 = 12800

#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port

void setup_driver(TMC2209Stepper &driver, int EN_PIN, int MS1, int MS2) {
  Serial.println("Setting up driver");
  pinMode(EN_PIN, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);

  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, HIGH);
                                  // Enable one according to your setup
//SPI.begin();                    // SPI drivers
  // SERIAL_PORT.begin(115200);      // HW UART drivers
// driver.beginSerial(115200);     // SW UART drivers

  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(1600);        // Set motor RMS current
  driver.microsteps(MICROSTEPS);          // Set microsteps to 1/16th

//driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
//driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop
  Serial.println("Done setting up driver");
}




void move_arm(long int * delta, SStepper &motor1, SStepper &motor2, double theta1, double theta2) {
  long int target1 = theta1 * STEPS_PER_REV * MICROSTEPS/ (2.0*PI);
  long int target2 = 3 * (theta2 - 2.0 * theta1/3.0) * STEPS_PER_REV * MICROSTEPS / (2.0*PI);

  Serial1.print("Target1: ");
  Serial1.print(target1);
  Serial1.print(" , ");
  Serial1.println(target2);

  motor1.set_target(target1);
  motor2.set_target(target2);

  delta[0] = motor1.distance_to_go();
  delta[1] = motor2.distance_to_go();
  if(delta[0] == 0 && delta[1] == 0) {
    return;
  }
  if(abs(delta[0]) > abs(delta[1])) {
    motor1.set_speed(MAX_SPEED * (delta[0] > 0 ? 1 : -1));
    motor2.set_speed(MAX_SPEED * 1.0 * delta[1] / abs(delta[0]));
  } else {
    motor2.set_speed(MAX_SPEED * (delta[1] > 0 ? 1 : -1));
    motor1.set_speed(MAX_SPEED * 1.0 * delta[0] / abs(delta[1]));
  }
  Serial.print("Speeds: ");
  Serial.print(motor1.speed);
  Serial.print(" , ");
  Serial.println(motor2.speed);

  delta[0] = motor1.distance_to_go();
  delta[1] = motor2.distance_to_go();
}

