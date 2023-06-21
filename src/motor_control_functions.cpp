#include "motor_control_functions.h"

#define MAX_SPEED                 0.1  // m/s
#define MAX_ANGULAR_SPEED         1000.0  // steps/s
#define MICROSTEPS                16
#define STEPS_PER_REV             200
#define MAX_TARGET_DISTANCE       50


// Total steps per revolution = 200 * 16 = 3200

#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port

void setup_driver(TMC2209Stepper &driver, int EN_PIN, int MS1, int MS2) {
  Serial.println("Setting up driver");
  pinMode(EN_PIN, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);

  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
                                  // Enable one according to your setup
//SPI.begin();                    // SPI drivers
  SERIAL_PORT.begin(115200);      // HW UART drivers
// driver.beginSerial(115200);     // SW UART drivers

  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(3);                 // Enables driver in software
  // driver.pdn_disable(true);
  driver.rms_current(500);        // Set motor RMS current
  driver.microsteps( MICROSTEPS );          // Set microsteps to 1/16th
  // driver.irun(31);

  driver.intpol(true);               // Interpolate to 256 steps, smooth stepping even with 0 microsteps.

//driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop
  Serial.println("Done setting up driver");
}

double K = STEPS_PER_REV * MICROSTEPS/ (2.0*PI);
float R = 0.63/2;

long current_targets[2] = {-1, -1};
long initial_positions[2] = {0, 0};
void move_arm(long int * delta, SStepper &motor1, SStepper &motor2, double theta1, double theta2) {
  double max_speed_1 = min(0.5 * 18.0 * MAX_SPEED * K / (R * (6*abs(cos(theta2/2)) + 1)), MAX_ANGULAR_SPEED);
  double max_speed_2 = min(0.5 * 18.0 * MAX_SPEED * K / R, MAX_ANGULAR_SPEED);

  long int target1 = 3 * theta1 * K;
  long int target2 = 3 * 3 * (theta2 + theta1/3.0) * K;

  double speed_1;
  double speed_2;

  if(target1 != current_targets[0]) {
    Serial.print("Target 1: " + String(target1) + ", " + String(theta1) + ". Current pos: ");
    Serial.println(motor1.position / (K * 3));
    initial_positions[0] = current_targets[0];
    current_targets[0] = target1;

    motor1.set_target(target1);
  }
  if(target2 != current_targets[1]) {
    Serial.print("Target 2: " + String(target2) + ", " + String(theta2) + ". Current pos: ");
    Serial.println(motor2.position / (9*K) - motor1.position / (9*K));
    initial_positions[1] = current_targets[1];
    current_targets[1] = target2;
    motor2.set_target(target2);
  }
  delta[0] = motor1.distance_to_go();
  delta[1] = motor2.distance_to_go();

  long initial_delta[2] = {
    current_targets[0] - initial_positions[0],
    current_targets[1] - initial_positions[1]
  };
  // use larger distance as reference
  double expected_delta[2] = { 0.0, 0.0 };
  int force_motor = -1;
  long int max_initial_delta = max(abs(initial_delta[0]), abs(initial_delta[1]));
  // expected_delta[0] = 
  if( abs(initial_delta[0]) > abs(initial_delta[1]) ) {
    expected_delta[0] = delta[0];
    expected_delta[1] = delta[0] * abs(initial_delta[1]) / abs(initial_delta[0]);
    if( abs(delta[1]) - abs(expected_delta[1]) >= 1 ) {
      force_motor = 2;
    }
    if( abs(delta[1]) - abs(expected_delta[1]) <= -1 ) {
      force_motor = 1;
    }
  } else {
    expected_delta[1] = delta[1];
    expected_delta[0] = delta[1] * abs(initial_delta[0]) / abs(initial_delta[1]);

    if( abs(delta[0]) - abs(expected_delta[0]) >= 1 ) {
      force_motor = 1;
    }
    if( abs(delta[0]) - abs(expected_delta[0]) <= -1 ) {
      force_motor = 2;
    }
  }

  
  speed_1 = max_speed_1 * delta[0]/(abs(delta[0]) + 0.00001);
  speed_2 = max_speed_1 * delta[1]/(abs(delta[0]) + 0.00001);
  if( abs(speed_2) > max_speed_2 ) {
    speed_1 = max_speed_2 * delta[0]/(abs(delta[1]) + 0.00001);
    speed_2 = max_speed_2 * delta[1]/(abs(delta[1]) + 0.00001);
  }

  if( abs(delta[0]) < 10) {
    speed_1 = delta[0];
  }
  if( abs(delta[1]) < 10) {
    speed_2 = delta[1];
  }

  // EVERY_N_MILLISECONDS(1000) {
  //   Serial.println("Ex Delta: "+ String(expected_delta[0]) + ", " + String(expected_delta[1]));
  //   Serial.println("Delta: "+ String(delta[0]) + ", " + String(delta[1]));
  //   Serial.println(motor1.position / (K * 3));
  //   Serial.println(force_motor);
  //   Serial.println("Speeds: " + String(motor1.speed) + ", " + String(motor2.speed) );
  // }

  // Serial.println("Speeds: " + String(motor1.speed) + ", " + String(motor2.speed) );
  motor1.set_speed( speed_1);
  motor2.set_speed( speed_2 );
  // motor1.set_acceleration( (speed_1 - motor1.speed) / 10.0);
  // motor2.set_acceleration( (speed_2 - motor2.speed) / 10.0);


  // Serial.print("Speed: ");
  // Serial.print("1: ");
  motor1.one_step();
  // Serial.print(",");
  // Serial.print("2: ");
  motor2.one_step();
  // Serial.println("");

  if(force_motor == 1) {
    motor1.force_step();
  }
  if(force_motor == 2) {
    motor2.force_step();
  }

  delta[0] = abs(current_targets[0] - motor1.position);
  delta[1] = abs(current_targets[1] - motor2.position);
}

