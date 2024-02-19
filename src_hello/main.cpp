
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

Preferences preferences;

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
