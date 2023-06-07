#include "motor_control_functions.h"

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

void move_arm(double * delta, SStepper &motor1, SStepper &motor2, double theta1, double theta2) {
  double steps1 = abs(theta1 * 200.0 * 64.0/ (2.0*PI));
  double steps2 = 3 * abs(theta2 * 200.0 * 64.0 / (2.0*PI));

  steps1 = min(steps1, 12.0);
  steps2 = min(steps2, 12.0);

  // Serial.print(theta1);
  // Serial.print(", ");
  // Serial.println(theta2);


  for(int i=0; i < max(steps1, steps2); i++) {
    if(i < steps1) {
      motor1.one_step(theta1<0?HIGH:LOW, 600);
      if(i%3 != 0) {
        motor2.one_step(theta1>0?HIGH:LOW, 600);
      }
    }
    if(i < steps2) {
      motor2.one_step(theta2<0?HIGH:LOW, 500);
    }
  }
  int sign1 = theta1>0?1:-1;
  int sign2 = theta2>0?1:-1;
  delta[0] = sign1*steps1 * (2.0*PI) / 64.0 / 200.0;
  delta[1] = sign2*steps2 * (2.0*PI) / 64.0 / 200.0;
}

