#ifndef LED_CONTROL_H
#define LED_CONTROL_H

// #define FASTLED_ALL_PINS_HARDWARE_SPI
// #define FASTLED_ESP32_SPI_BUS HSPI
// #include <SPI.h>
// #define FASTLED_ESP32_I2S true
// #define FASTLED_FORCE_SOFTWARE_SPI
#include <FastLED.h>

#define NUM_LEDS 220
#define LED_PIN 4


void setup_led();
void test_led();
void init_led();
void set_led_status(int error_code=0);
void move_led();
void set_led_color(uint8_t r=255, uint8_t g=255, uint8_t b=255, uint8_t brightness=255);


#endif
