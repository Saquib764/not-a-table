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

void move_arm(SStepper &motor1, SStepper &motor2, double theta1, double theta2) {
  double steps1 = 3 * abs(theta1 * 200 / (2*PI) * 64);
  double steps2 = 3 * 3 * abs(theta2 * 200 / (2*PI) * 64);

  Serial.print(theta1);
  Serial.print(", ");
  Serial.println(theta2);

  Serial.print(steps1);
  Serial.print(", ");
  Serial.println(steps2);

  for(int i=0; i < max(steps1, steps2); i++) {
    if(i < steps1) {
      motor1.one_step(theta1<0?HIGH:LOW);
    }
    if(i < steps2) {
      motor2.one_step(theta2<0?HIGH:LOW);
    }
  }
}

