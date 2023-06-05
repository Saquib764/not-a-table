#include "homing_functions.h"


void perform_homing(SStepper &motor) {
  // This function performs the homing routine for a single motor.
  int steps_while_pin_is_low = 0;
  while (digitalRead(motor.HOMING_PIN) == LOW) {
    motor.one_step(1, 600);
  }
  while (digitalRead(motor.HOMING_PIN) == HIGH) {
    motor.one_step(1, 600);
    steps_while_pin_is_low += 1;
  }
  // Move back half the number of steps while the pin was low.
  for (int i = 0; i < 1.3 * steps_while_pin_is_low/2; i++) {
    motor.one_step(0, 600);
  }
}
