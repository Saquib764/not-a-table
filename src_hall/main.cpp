
#include <Arduino.h>
#include "math.h"
#include <iostream>
#include<array>

using namespace std;



uint8_t motor1HomingPin = 32;

uint8_t motor2HomingPin = 33;


void setup() {
  delay(50);

  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Version: " + String(VERSION));

}


void loop() {
  int v1 = analogRead(motor1HomingPin) - 2000;
  int v2 = analogRead(motor2HomingPin) - 2000;

  Serial.println("Hall: " + String(v1) + ", " + String(v2));
  delay(1);
}
