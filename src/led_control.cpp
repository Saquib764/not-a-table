#include "led_control.h"

uint8_t hue = 0;

CRGB leds[NUM_LEDS];

void setup_led() {
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS); 
  FastLED.setBrightness(50);
}

void test_led() {
  leds[0] = CRGB::Red;
  leds[1] = CRGB::Green;
  leds[2] = CRGB::Blue;
  FastLED.show();
}

void move_led() {
  
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(hue + (i * 10), 255, 255);
  }

  EVERY_N_MILLISECONDS(15){
    hue++;
  }

  FastLED.show();
}