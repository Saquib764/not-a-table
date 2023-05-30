#include "led_control.h"

uint8_t hue = 0;
int pos = 0;
CRGB leds[NUM_LEDS];

void setup_led() {
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS); 
  FastLED.setBrightness(255);
}

void test_led() {
  leds[0] = CRGB::Red;
  leds[1] = CRGB::Green;
  leds[2] = CRGB::Blue;
  FastLED.show();
}

void move_led() {
  
  for (int i = 0; i < NUM_LEDS; i++) {
    if(i < pos - 15 || i>pos + 15) {
      leds[i] = CRGB::Black;
      continue;
    }
    leds[i] = CHSV(hue + (( i - pos) * 10), 255, 255);
    leds[i].maximizeBrightness(15*6 + (i - pos) * 5);
  }

  EVERY_N_MILLISECONDS(15){
    hue++;
    pos ++;
    pos = pos%NUM_LEDS;
  }

  FastLED.show();
}
