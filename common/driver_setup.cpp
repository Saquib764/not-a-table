#include "driver_setup.h"


void setup_driver(TMC2209Stepper &driver, int EN_PIN) {
  Serial.println("Setting up driver.common");
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware
                                  // Enable one according to your setup
//SPI.begin();                    // SPI drivers
  // NEVER increase the baud rate, its adds 10ms delay. probably related to buffer overflow and flush cycle
  // SERIAL_PORT.begin(9600);      // HW UART drivers
// driver.beginSerial(115200);     // SW UART drivers

  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(3);                 // Enables driver in software
  // driver.pdn_disable(true);
  driver.rms_current(1200);        // Set motor RMS current
  driver.microsteps( MICROSTEPS );          // Set microsteps to 1/16th
  // driver.irun(31);

  driver.intpol(true);               // Interpolate to 256 steps, smooth stepping even with 0 microsteps.

//driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop
  Serial.println("Done setting up driver");
}
