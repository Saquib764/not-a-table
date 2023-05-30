#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <SPI.h>
#include <FastLED.h>
#define NUM_LEDS 120
#define LED_PIN 4


void setup_led();
void test_led();
void move_led();


#endif