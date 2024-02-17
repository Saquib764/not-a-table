
#include <Arduino.h>
#include "math.h"
#include <TMCStepper.h>
#include <ArduinoJson.h>
#include <iostream>
#include<array>

#include <FastLED.h>

using namespace std;
// #include "SPIFFS.h"

#include <Preferences.h>

const int dummy = 0;

#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f
#define VERSION "1.0.0"


Preferences preferences;


String playlist[3] = {
  "designs/AngularRadiance.thr.txt",
  "designs/circle.thr.txt",
  "designs/spiral.thr.txt"
};


void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // if(!SPIFFS.begin(true)){
  //   Serial.println("An Error has occurred while mounting SPIFFS");
  //   return;
  // }

  Serial.println("Let there be light!");
}

void loop() {

  EVERY_N_MILLISECONDS(2000){
    Serial.println("There is light!");
  }
  // Serial.print("  ");
}
