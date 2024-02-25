#include <SpeedyStepper.h> //Simple & good stepper library, get it.

#include <TMCStepper.h>

//Solder EN & MS1 & MS2 to ground on the driver. This enables the driver and sets its UART address as 0. You can use UART to enable & disable the driver.
#define EN_PIN           7
#define DIR_PIN          35 // Direction
#define STEP_PIN         36 // Step
 // Define UART1 TX and RX pins
#define RXD_PIN 17 // RX pin for UART1
#define TXD_PIN 18 // TX pin for UART1

HardwareSerial SERIAL_PORT(1); 
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2.

// UART addresses set by MS1 & MS2:
// 0b00 = 0 MS1 low  MS2 low
// 0b01 = 1 MS1 low  MS2 high
// 0b10 = 2 MS1 high MS2 low
// 0b11 = 3 MS1 high MS2 high

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075


TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

SpeedyStepper stepper;

void setup() {

pinMode(EN_PIN, OUTPUT); // Set the EN pin as an output
digitalWrite(EN_PIN, LOW); // Set the EN pin LOW to ground it


stepper.connectToPins(STEP_PIN, DIR_PIN); // INITIALIZE SpeedyStepper
SERIAL_PORT.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN); 
  
SERIAL_PORT.begin(115200);      // INITIALIZE UART TMC2209
Serial.begin(115200);
delay(500);
Serial.println(F("Serial Initialized"));

  driver.begin();       // Initialize driver
                           
  driver.toff(3);                 // Enables driver in software and 0 disables the driver

  driver.rms_current(600, 0.5);       // Set motor RMS current & hold current divider.
  driver.microsteps(64);            // Set microsteps to 1/2

  driver.intpol(true);               // Interpolate to 256 steps, smooth stepping even with 0 microsteps.

  driver.pwm_autoscale(true);    // Needed for stealthChop
  driver.en_spreadCycle(true);   // false = StealthChop / true = SpreadCycle

  stepper.setCurrentPositionInSteps(0);                   // Set zero position
  stepper.setSpeedInStepsPerSecond(1600);              //Set Speed
  stepper.setAccelerationInStepsPerSecondPerSecond(1600);   //Set acceleration, smaller value for super smooth direction changing

}

void loop() {

uint16_t msread=driver.microsteps();
Serial.print(F("Read microsteps via UART to test UART receive : "));    Serial.println(msread); 
  
Serial.println(F("Move 6400 steps forward at 600ma"));
driver.rms_current(600); 
stepper.moveToPositionInSteps(9600);
Serial.println(F("Yoyo"));

Serial.println(F("Wait 3sec and turn current low so you can turn the motor shaft"));
driver.rms_current(10); 
delay(3000);

Serial.println(F("Move back to 0 position at 300ma"));
driver.rms_current(300); 
stepper.moveToPositionInSteps(0);

//MOVE MOTOR VIA UART AND CHANGE DIRECTION VIA SOFTWARE, IT RUNS AS LONG AS YOU LET IT... PROBABLY ONLY USEFUL WITH ENCODER. THE VALUE SETS ONLY THE SPEED.

//driver.VACTUAL(16000); //SET SPEED OF MOTOR
delay(2000); // MOTOR MOVES 2 SEC THEN STOPS
//driver.VACTUAL(0); //STOP MOTOR BY SETTING SPEED TO 0
//driver.shaft(!driver.shaft); // REVERSE DIRECTION

}