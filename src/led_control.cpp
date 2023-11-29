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
void init_led() {
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::White;
  }
  FastLED.show();
}
void set_led_status(int status_code) {
  /* 0: No error, GREEN
   * 1: RED
   * 2: Wifi in pairing mode, YELLOW, BLINK
  */
  if(status_code == 0) {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(20, 200, 20);
      leds[i].maximizeBrightness(20);
    }
  } else if(status_code == 1) {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Red;
    }
  } else if(status_code == 2) {
    EVERY_N_MILLISECONDS(200) {
      for(int i = 0; i < NUM_LEDS; i++) {
        if(leds[i] == (CRGB)CRGB::Yellow) {
          leds[i] = CRGB::Black;
          continue;
        }
        leds[i] = CRGB::Yellow;
      }
    }
  }
  FastLED.show();
}

void move_led() {
  int L = 15;
  for (int i = 0; i < NUM_LEDS; i++) {
    int d = abs(pos - i);
    if(d > L) {
      d = abs(NUM_LEDS - i - pos);
    }
    if( d > L) {
          leds[i] = CRGB::Black;
          continue;
    }
    leds[i] = CHSV(hue + (d * 10), 150, 155);
    leds[i].maximizeBrightness(15*2 + d * 2);
  }

  EVERY_N_MILLISECONDS(15){
    hue++;
    pos ++;
    pos = pos%NUM_LEDS;
  }

  FastLED.show();
}
void set_led_color(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(r, g, b);
  }
  FastLED.setBrightness(brightness);
  FastLED.show();
}
