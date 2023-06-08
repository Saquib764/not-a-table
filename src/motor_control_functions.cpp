#include "motor_control_functions.h"

long int odometer[2] = {0, 0};

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
//SERIAL_PORT.begin(115200);      // HW UART drivers
//driver.beginSerial(115200);     // SW UART drivers

  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(600);        // Set motor RMS current
  driver.microsteps(64);          // Set microsteps to 1/16th

//driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
//driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop
  Serial.println("Done setting up driver");
}

void move_stepper(SStepper &motor, double theta) {
  double steps = abs(theta * 200 / (2*PI) * 64);
  Serial.print(theta);
  Serial.print(", ");
  Serial.println(steps);

  for(int i=0; i < steps; i++) {
    motor.one_step(theta<0?HIGH:LOW);
  }
}

void reset_odometer() {
  odometer[0] = 0;
  odometer[1] = 0;
}

void move_arm(long int * delta, SStepper &motor1, SStepper &motor2, double theta1, double theta2) {
  long int steps1 = theta1 * 200.0 * 64.0/ (2.0*PI);
  long int steps2 = 3 * (theta2 - 2.0 * theta1/3.0) * 200.0 * 64.0 / (2.0*PI);

  long int dsteps1 = steps1 - odometer[0];
  long int dsteps2 = steps2 - odometer[1];

  long int _dsteps1 = min(abs(dsteps1), 12.0);
  long int _dsteps2 = min(abs(dsteps2), 12.0);

  // Serial.print(theta1);
  // Serial.print(", ");
  // Serial.println(theta2);


  for(int i=0; i < max(_dsteps1, _dsteps2); i++) {
    if(i < _dsteps1) {
      motor1.one_step(dsteps1<0?HIGH:LOW, 600);
    }
    if(i < _dsteps2) {
      motor2.one_step(dsteps2<0?HIGH:LOW, 500);
    }
  }
  odometer[0] += _dsteps1 * (dsteps1>0?1:-1);
  odometer[1] += _dsteps2 * (dsteps2>0?1:-1);
  delta[0] = steps1 - odometer[0];
  delta[1] = steps2 - odometer[1];
}

